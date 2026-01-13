from spark_robot import RobotKinematics
import numpy as np
from spark_utils import compute_pairwise_distance

from spark_policy.safe.safe_algo.value_based.base.basic_collision_safety_index import BasicCollisionSafetyIndex

class SecondOrderCollisionSafetyIndexApprox(BasicCollisionSafetyIndex):
    
    def __init__(self,
                 robot_kinematics : RobotKinematics,
                 **kwargs
                 ):
        
        super().__init__(robot_kinematics, **kwargs)
        
        assert "Dynamic2" in self.robot_cfg.__class__.__name__, "SecondOrderCollisionSafetyIndexApprox is only supported for robot configuration with Dynamic2 (Acceleration Control)"
        
        self.n = kwargs["phi_n"]
        self.k = kwargs["phi_k"]
        self.eps_phi = 1e-6 # Add small value to avoid division by zero
        self.eps_x = 1e-4 # Small value for numerical differentiation
        self.eps_dof = 1e-4 # Small value for numerical differentiation of state
        
        # Safety specification in the Cartesian space
        self._phi0 = lambda dist: self.dmin ** self.n - dist ** self.n

        # Time derivative of the safety specification in the Cartesian space
        self._phi0dot = lambda phi0_forward_dof, phi0_backward_dof: (phi0_forward_dof - phi0_backward_dof) / (2 * self.eps_x)

        # General implementation of second-order safety index
        self._phi = lambda phi0, phi0dot: phi0 +  self.k * phi0dot
    
    def construct_vol_list(self,
                            x        : np.ndarray,
                            task_info: dict,
                            ):
        
        trans_world2base          = task_info["robot_base_frame"]
        obstacle_vol_frames_world = task_info["obstacle"]["frames_world"]
        obstacle_vol_geom         = task_info["obstacle"]["geom"]

        vol_frame_list_robot = []
        vol_geom_list_robot = []
        x_list = []
        x_list.append(x)
        for i in range(self.num_state):
            x_forward = x.copy()
            x_forward[i] += self.eps_x
            x_backward = x.copy()
            x_backward[i] -= self.eps_x
            x_list.append(x_forward)
            x_list.append(x_backward)
        
        for _x in x_list:
            # compute all robot frames
            # frames = self.robot_kinematics.forward_kinematics(self.robot_cfg.decompose_state_to_dof_pos(x))
            dof_pos = self.robot_cfg.decompose_state_to_dof_pos(_x)
            dof_vel = self.robot_cfg.decompose_state_to_dof_vel(_x)
            dof_pos_forward = dof_pos.copy()
            dof_pos_forward += dof_vel * self.eps_dof
            
            dof_pos_backward = dof_pos.copy()
            dof_pos_backward -= dof_vel * self.eps_dof
            for _dof_pos in [dof_pos, dof_pos_forward, dof_pos_backward]:
                self.robot_kinematics.pre_computation(_dof_pos, dof_vel)
                frames = self.robot_kinematics.forward_kinematics(_dof_pos)
                # trans_world2base = self.robot_kinematics.get_base_frame(_dof_pos)
                trans_world2base = self.robot_kinematics.update_base_frame(trans_world2base, _dof_pos)               
                
                # # get collision vol frames
                collision_vol_frames_base = np.stack([frames[fid] for fid in self.collision_vol_frame_ids], axis=0) # [num_collision_vol, 4, 4]
                # # transform to world frame
                collision_vol_frames_world = trans_world2base @ collision_vol_frames_base
                # import ipdb;ipdb.set_trace()
                vol_frame_list_robot.append(collision_vol_frames_world)
                vol_geom_list_robot.append(self.collision_vol_geom) 
        B = len(vol_frame_list_robot)
        
        return np.array(vol_frame_list_robot), np.array(self.collision_vol_geom), np.tile(obstacle_vol_frames_world, (B, 1, 1, 1)), np.array(obstacle_vol_geom)
   
        
    def phi(self,
                 x        : np.ndarray,
                 task_info: dict,
                 ):
        '''
            Args:
                x (np.ndarray): [num_state,] Robot state values.
            
            Returns:
                grad (np.ndarray): [num_constraint, num_state] gradient of safety index function w.r.t. x.
        '''
        self.update_obstacle_info(task_info)
        
        vol_frame_list_robot, vol_geom_list_robot, obstacle_vol_frames_world, obstacle_vol_geom = self.construct_vol_list(x, task_info)
        dist_env = compute_pairwise_distance(
            frame_list_1 = vol_frame_list_robot,
            geom_list_1  = vol_geom_list_robot,
            frame_list_2 = obstacle_vol_frames_world,
            geom_list_2  = obstacle_vol_geom
        )
        # compute pairwise distances for self collision
        dist_self = compute_pairwise_distance(
            frame_list_1 = vol_frame_list_robot,
            geom_list_1  = vol_geom_list_robot,
            frame_list_2 = vol_frame_list_robot,
            geom_list_2  = vol_geom_list_robot
        )
        
        dist_env_curr_dof = dist_env[0::3]
        dist_env_forward_dof = dist_env[1::3]
        dist_env_backward_dof = dist_env[2::3]
        dist_self_curr_dof = dist_self[0::3]
        dist_self_forward_dof = dist_self[1::3]
        dist_self_backward_dof = dist_self[2::3]
        
        # compute phi 
        B = dist_self_curr_dof.shape[0]
        dist = np.zeros((B, self.num_constraint))
        dist_forward_dof = np.zeros((B, self.num_constraint))
        dist_backward_dof = np.zeros((B, self.num_constraint))
        dist[:, :self.num_constraint_env] = dist_env_curr_dof.reshape(B, -1)
        dist_forward_dof[:, :self.num_constraint_env] = dist_env_forward_dof.reshape(B, -1)
        dist_backward_dof[:, :self.num_constraint_env] = dist_env_backward_dof.reshape(B, -1)
        if self.num_constraint_self > 0:
            triu_indices = np.triu_indices(dist_self_curr_dof.shape[1], k=1)
            dist[:, -self.num_constraint_self:] = dist_self_curr_dof[:, triu_indices[0], triu_indices[1]]
            dist_forward_dof[:, -self.num_constraint_self:] = dist_self_forward_dof[:, triu_indices[0], triu_indices[1]]
            dist_backward_dof[:, -self.num_constraint_self:] = dist_self_backward_dof[:, triu_indices[0], triu_indices[1]]
        
        phi0_dof = self._phi0(dist)
        phi0_forward_dof = self._phi0(dist_forward_dof)
        phi0_backward_dof = self._phi0(dist_backward_dof)
        
        phi0dot_dof = self._phi0dot(phi0_forward_dof, phi0_backward_dof)
        phi_dof = self._phi(phi0_dof, phi0dot_dof)

        phi = phi_dof[0]
        phi0 = phi0_dof[0]
        phi0dot = phi0dot_dof[0]
        # compute phi gradient
        phi_forward_x = phi_dof[1::2]
        phi_backward_x = phi_dof[2::2]
        grad_phi = (phi_forward_x - phi_backward_x) / (2 * self.eps_dof)
        
        grad_phi = grad_phi.T
        
        Lf = grad_phi @ self.robot_cfg.dynamics_f(x)
        Lg = grad_phi @ self.robot_cfg.dynamics_g(x)

        return phi, Lg, Lf, phi0, phi0dot
        
    