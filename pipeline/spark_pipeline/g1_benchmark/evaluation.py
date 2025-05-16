import numpy as np
import os

def arm_goal_tracking_score(data):
    '''
        d: [t, 2]
    '''
    if 'dist_goal_arm' not in data:
        return None

    d = (data['dist_goal_arm'] - 0.05).clip(min=0.0)
    s = np.exp(-(d**2)/0.002).mean(axis=-1).mean()
    return s

def base_goal_tracking_score(data):
    '''
        d: [t,]
    '''
    if 'dist_goal_base' not in data:
        return None

    d = (data['dist_goal_base'] - 0.1).clip(min=0.0)
    s = np.exp(-(d**2)/0.05).mean()
    return s

def dist_score(d, mask, d_min=0.05):
    '''
        d: [t, m, n]
        Penalize distance within d_min
        Non-positive, higher is better
    '''

    if (np.array(d.shape) == 0).any():
        return None

    t, m, n = d.shape
    diff = d - d_min
    within_thres = np.isfinite(d) & (diff <= 0)
    s = np.exp(-(diff**2)/0.0002)
    s = s[mask & within_thres]

    if len(s) == 0:
        return 1.0
    else:
        return s.mean()

def evaluate_safety_self(data, d_min=0.05):
    '''
        Evaluate safety margin satisfaction
        Test if safe control respects designed safety margin
        Non-positive, higher is better
    '''

    return dist_score(data['dist_self'], data['self_collision_mask'], d_min)    

def evaluate_safety_env(data, d_min=0.05):
    '''
        Evaluate safety margin satisfaction
        Test if safe control respects designed safety margin
        Non-positive, higher is better
    '''

    return dist_score(data['dist_robot_to_env'], data['env_collision_mask'], d_min)

def evaluate_critical_safety(data):
    '''
        Evaluate critical safety satisfaction
        Test if modeled geometries are in collision
        Non-positive, higher is better
    '''
    
    ss = []

    s_self = dist_score(data['dist_self'], data['self_collision_mask'], d_min=0.0)
    if s_self is not None:
        ss.append(s_self)

    s_env = dist_score(data['dist_robot_to_env'], data['env_collision_mask'], d_min=0.0)
    if s_env is not None:
        ss.append(s_env)

    if len(ss) > 0:
        return np.array(ss).mean()
    else:
        return None

def evaluate_constraint_satisfaction_by_value_self(data):
    
    if 'violation_self' in data:
        val = data['violation_self']
        mask = data['self_collision_mask']
        mask = mask.repeat(val.shape[0], axis=0)
        val[~mask] = 0.0
        s = val.sum(-1).sum(-1)
        s = s[s>0.0] # only look at violations
        # s = np.exp(-s**2 / 10000).mean() if len(s) > 0 else 1.0 # ssa
        s = np.exp(-s**2 / 10).mean() if len(s) > 0 else 1.0 # sss/cbf

        return s
    else:
        return None

def evaluate_constraint_satisfaction_by_value_env(data):
    
    ss = []

    if 'violation_env' in data:
        val = data['violation_env'] # [t, m, n]
        mask = data['env_collision_mask'] # [1, m, n]
        mask = mask.repeat(val.shape[0], axis=0) # [t, m, n]
        val[~mask] = 0.0
        s = val.sum(-1).sum(-1)
        s = s[s>0.0] # only look at violations
        # s = np.exp(-s**2 / 10000).mean() if len(s) > 0 else 1.0 # ssa
        s = np.exp(-s**2 / 10).mean() if len(s) > 0 else 1.0 # sss/cbf
        
        return s
    else:
        return None

def evaluate_feasibility_rate(data):

    infeasible_self = None
    if 'violation_self' in data:
        val = data['violation_self']
        mask = data['self_collision_mask']
        mask = mask.repeat(val.shape[0], axis=0)
        val[~mask] = 0.0
        s = val.sum(-1).sum(-1)
        infeasible_self = (s > 0.0)

    infeasible_env = None
    if 'violation_env' in data:
        val = data['violation_env'] # [t, m, n]
        mask = data['env_collision_mask'] # [1, m, n]
        mask = mask.repeat(val.shape[0], axis=0) # [t, m, n]
        val[~mask] = 0.0
        s = val.sum(-1).sum(-1)
        infeasible_env = (s > 0.0)
    
    if infeasible_self is not None and infeasible_env is not None:
        infeasible = infeasible_self | infeasible_env
    elif infeasible_self is not None:
        infeasible = infeasible_self
    elif infeasible_env is not None:
        infeasible = infeasible_env
    else:
        infeasible = None
    
    if infeasible is not None:
        return 1.0 - infeasible.mean()
    else:
        return None

def evaluate_success(data, goal_base_thres = 0.1, goal_arm_thres = 0.05):
    '''
        Evaluate task success
    '''
    if 'seed' not in data:
        return None
    
    seed = data['seed']
    done = data['done']
    
    change_indices = np.where(done)[0] + 1
    if change_indices[-1] == len(done):
        change_indices = change_indices[:-1]
    # Add the starting and ending indices
    start_indices = np.append(0, change_indices)
    end_indices = np.append(change_indices, len(done))
    # Extract unique seeds

    # Create the dictionary
    traj_ranges = [(start, end) for start, end in zip(start_indices, end_indices)] 
    
    success_list = np.zeros_like(done)
    for traj_range in traj_ranges:
        success = True
        if 'dist_goal_base' in data:
            dist_goal_base = data['dist_goal_base'][traj_range[0]:traj_range[1]]
            if dist_goal_base[-1] > goal_base_thres:
                # print("dist_goal_base", dist_goal_base[-1])
                success =  False
            
        if 'dist_goal_arm' in data:
            dist_goal_arm = data['dist_goal_arm'][traj_range[0]:traj_range[1]]
            if dist_goal_arm[-1].mean(axis=-1) > goal_arm_thres:
                # print("dist_goal_arm", dist_goal_arm[-1])
                success = False
            
        if 'dist_self' in data:
            dist_self = data['dist_self'][traj_range[0]:traj_range[1]]
            if (dist_self < 0.0).any():
                # print("dist_self")
                success = False
        
        if 'dist_robot_to_env' in data:
            dist_robot_to_env = data['dist_robot_to_env'][traj_range[0]:traj_range[1]]
            if (dist_robot_to_env < 0.0).any():
                # print("dist_goal_base")
                success = False
        
        # print("Success", success)
        success_list[traj_range[0]:traj_range[1]] = success
        
    success_seed = set(seed[success_list])
    success_seed = np.array(list(success_seed))
    # for s in success_seed:
    #     print("success seed: ", s)
    # print(success_seed.shape, success_seed )
    
    fail_seed = set(seed[~success_list])
    fail_seed = np.array(list(fail_seed))
    # for s in fail_seed:
    #     print("fail seed: ", s)
    # print(fail_seed.shape, fail_seed )
    
    return success_list, success_seed, fail_seed

def evaluate(data, d_min_env, d_min_self):
    eval = {}

    arm_goal_tracking = arm_goal_tracking_score(data)
    if arm_goal_tracking is not None:
        eval['arm_goal_tracking'] = arm_goal_tracking

    base_goal_tracking = base_goal_tracking_score(data)
    if base_goal_tracking is not None:
        eval['base_goal_tracking'] = base_goal_tracking

    safety_score_self = evaluate_safety_self(data, d_min=d_min_self)
    if safety_score_self is not None:
        eval['safety_score_self'] = safety_score_self
    
    safety_score_env = evaluate_safety_env(data, d_min=d_min_env)
    if safety_score_env is not None:
        eval['safety_score_env'] = safety_score_env

    # critical_safety_score = evaluate_critical_safety(data)
    # if critical_safety_score is not None:
    #     eval['critical_safety_score'] = critical_safety_score

    constraint_value_score_self = evaluate_constraint_satisfaction_by_value_self(data)
    if constraint_value_score_self is not None:
        eval['constraint_value_score_self'] = constraint_value_score_self
    
    constraint_value_score_env = evaluate_constraint_satisfaction_by_value_env(data)
    if constraint_value_score_env is not None:
        eval['constraint_value_score_env'] = constraint_value_score_env

    feasibility_rate = evaluate_feasibility_rate(data)
    if feasibility_rate is not None:
        eval['feasibility_rate'] = feasibility_rate

    return eval

    