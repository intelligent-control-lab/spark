from spark_robot import RobotKinematics
from spark_utils import Geometry, compute_pairwise_dist
import numpy as np

from spark_safe.safe_algo.value_based.base.base_safety_index import BaseSafetyIndex

class BasicCollisionSafetyIndex(BaseSafetyIndex):
    
    def __init__(self,
                 robot_kinematics : RobotKinematics,
                 **kwargs
                 ):
        
        super().__init__(robot_kinematics)
        
        self.min_distance  = kwargs["min_distance"]
        
        # process robot collision info
        self.collision_vol_frame_ids  = list(self.robot_cfg.CollisionVol.keys()) # frame ids, to index the frames computed by FK
        self.collision_vol_geom       = [self.robot_cfg.CollisionVol[fid] for fid in self.collision_vol_frame_ids] # collision volume info
        self.num_collision_vol        = len(self.collision_vol_frame_ids)
        self.enable_self_collision    = kwargs["enable_self_collision"]
        self.env_collision_vol_ignore = [getattr(self.robot_cfg.Frames, name) for name in kwargs["env_collision_vol_ignore"] if hasattr(self.robot_cfg.Frames, name)] # str to frame id
        
        self.num_constraint_self = self.num_collision_vol * (self.num_collision_vol - 1)  // 2
        
        # create mask for self collision check
        self.self_collision_mask = self.enable_self_collision * np.ones((self.num_collision_vol, self.num_collision_vol), dtype=bool)
        self.self_collision_mask[np.tril_indices(self.self_collision_mask.shape[0], k=0)] = False
        for frame_i, frame_j in self.robot_cfg.AdjacentCollisionVolPairs:
            i = self.collision_vol_frame_ids.index(frame_i)
            j = self.collision_vol_frame_ids.index(frame_j)
            self.self_collision_mask[i, j] = False
            self.self_collision_mask[j, i] = False
        for frame_i in self.robot_cfg.SelfCollisionVolIgnored:
            i = self.collision_vol_frame_ids.index(frame_i)
            self.self_collision_mask[i, :] = False
            self.self_collision_mask[:, i] = False
            
        self.min_dist_self = self.min_distance["self"] * np.ones((self.num_collision_vol, self.num_collision_vol))
    
        self.update_obstacle_info()
    
    def update_obstacle_info(self, task_info=None):
        
        if task_info is not None:
            self.num_obstacles = task_info["obstacle"]["num"]
        else:
            self.num_obstacles = 0
        
        # compute number of constraints
        self.num_constraint_env  = self.num_collision_vol * self.num_obstacles
        self.num_constraint      = self.num_constraint_env + self.num_constraint_self
        
        # create mask for environment collision check
        self.env_collision_mask = np.ones((self.num_collision_vol, self.num_obstacles), dtype=bool)
        for fid in self.env_collision_vol_ignore:
            i = self.collision_vol_frame_ids.index(fid)
            self.env_collision_mask[i, :] = False
        
        # generate constraint mask
        self.phi_mask = np.zeros(self.num_constraint, dtype=bool)
        self.phi_mask[:self.num_constraint_env] = self.env_collision_mask.flatten()
        if self.num_constraint_self > 0:
            self.phi_mask[-self.num_constraint_self:] = self.self_collision_mask[np.triu_indices(self.self_collision_mask.shape[0], k=1)]

        # min dist matrix
        self.min_dist_env = self.min_distance["environment"] * np.ones((self.num_collision_vol, self.num_obstacles))
    
    def phi(self,
            x        : np.ndarray,
            task_info: dict):
        '''
            [num_constraint,] Safety index function.

            Args:
                x (np.ndarray): [num_state,] robot state.
        '''
        
        # assert "robot_base_frame" in task_info, "robot_base_frame not found in task_info"
        assert "obstacle" in task_info, "obstacle not found in task_info"
        assert "frames_world" in task_info["obstacle"], "frames_world not found in task_info['obstacle']"
        assert "geom" in task_info["obstacle"], "geom not found in task_info['obstacle']"
        
        self.update_obstacle_info(task_info)
        
        trans_world2base          = task_info["robot_base_frame"]
        obstacle_vol_frames_world = task_info["obstacle"]["frames_world"]
        obstacle_vol_geom         = task_info["obstacle"]["geom"]
        # compute all robot frames
        frames = self.robot_kinematics.forward_kinematics(self.robot_cfg.decompose_state_to_dof(x))
        trans_world2base = self.robot_kinematics.update_base_frame(trans_world2base, self.robot_cfg.decompose_state_to_dof(x))
        # get collision vol frames
        collision_vol_frames_base = np.stack([frames[fid] for fid in self.collision_vol_frame_ids], axis=0) # [num_collision_vol, 4, 4]
        # transform to world frame
        collision_vol_frames_world = np.zeros_like(collision_vol_frames_base)
        for i in range(self.num_collision_vol):
            collision_vol_frames_world[i, :, :] = trans_world2base @ collision_vol_frames_base[i, :, :]
        
        # compute pairwise distances between collision volumes and obstacles
        dist_env = compute_pairwise_dist(
            frame_list_1 = collision_vol_frames_world,
            geom_list_1  = self.collision_vol_geom,
            frame_list_2 = obstacle_vol_frames_world,
            geom_list_2  = obstacle_vol_geom
        )
        # compute pairwise distances for self collision
        dist_self = compute_pairwise_dist(
            frame_list_1 = collision_vol_frames_world,
            geom_list_1  = self.collision_vol_geom,
            frame_list_2 = collision_vol_frames_world,
            geom_list_2  = self.collision_vol_geom
        )
        # compute phi 
        phi = np.zeros(self.num_constraint)
        phi[:self.num_constraint_env] = (self.min_dist_env - dist_env).flatten()
        if self.num_constraint_self > 0:
            phi[-self.num_constraint_self:] = (self.min_dist_self - dist_self)[np.triu_indices(dist_self.shape[0], k=1)]

        return phi
        
    def grad_phi(self,
                 x        : np.ndarray,
                 task_info: dict,
                 eps      : float = 1e-4,
                 ):
        '''
            Args:
                x (np.ndarray): [num_state,] Robot state values.
            
            Returns:
                grad (np.ndarray): [num_constraint, num_state] gradient of safety index function w.r.t. x.
        '''
        
        grad = np.zeros((self.num_constraint, self.num_state))
        for i in range(self.num_state):
            x_forward = x.copy()
            x_forward[i] += eps
            x_backward = x.copy()
            x_backward[i] -= eps
            phi_forward = self.phi(x_forward, task_info)
            phi_backward = self.phi(x_backward, task_info)
            grad[:, i] = (phi_forward - phi_backward) / (2 * eps)
        
        return grad

    def phi_cartesian(self,
                        robot_frame_closest : np.ndarray,
                        task_info: dict,
                        trans_world2base    : np.ndarray):
        '''
            [num_constraint,] Safety index function.

            Args:
                robot_frame_closest (np.ndarray): [num_state,] closest point on robot.
        '''
        
        # assert "robot_base_frame" in task_info, "robot_base_frame not found in task_info"
        assert "obstacle" in task_info, "obstacle not found in task_info"
        assert "frames_world" in task_info["obstacle"], "frames_world not found in task_info['obstacle']"
        assert "geom" in task_info["obstacle"], "geom not found in task_info['obstacle']"
        
        self.update_obstacle_info(task_info)
        
        # trans_world2base          = task_info["robot_base_frame"]
        obstacle_vol_frames_world = task_info["obstacle"]["frames_world"]
        obstacle_vol_geom         = task_info["obstacle"]["geom"]
        # transform to world frame
        
        robot_frame_closest = [trans_world2base @ robot_frame_closest]
        cloest_robot_geom = [Geometry(type="sphere", radius=0.01)]
        # compute pairwise distances between collision volumes and obstacles
        dist_env = compute_pairwise_dist(
            frame_list_1 = robot_frame_closest,
            geom_list_1  = cloest_robot_geom,
            frame_list_2 = obstacle_vol_frames_world,
            geom_list_2  = obstacle_vol_geom
        )
        self.min_dist_env = self.min_distance["environment"] * np.ones(self.num_obstacles)
        # compute phi 
        phi_cartesian = (self.min_dist_env - dist_env).flatten()

        return phi_cartesian
    
    def grad_phi_cartesian(self,
                            robot_frame_closest        : np.ndarray,
                            task_info                  : dict,
                            trans_world2base           : np.ndarray,
                            eps                        : float = 1e-4,
                            ):
        '''
            Args:
                x (np.ndarray): [num_state,] Robot state values.
            
            Returns:
                grad (np.ndarray): [num_constraint, num_state] gradient of safety index function w.r.t. x.
        '''
        
        grad = np.zeros((self.num_obstacles, 3))
        for i in range(3):
            x_forward = robot_frame_closest.copy()
            x_forward[i,3] += eps
            x_backward = robot_frame_closest.copy()
            x_backward[i,3] -= eps
            phi_forward = self.phi_cartesian(x_forward, task_info,trans_world2base)
            phi_backward = self.phi_cartesian(x_backward, task_info,trans_world2base)
            grad[:, i] = (phi_forward - phi_backward) / (2 * eps)
        
        return grad
    
    def decode_constraint_info(self, vec, name):
        '''
                
        '''
            
        assert vec.shape[0] == self.num_constraint, f"vec shape {vec.shape} does not match num_constraint {self.num_constraint}"
        
        mat_env = vec[:self.num_constraint_env].reshape(self.num_collision_vol, self.num_obstacles)
        mat_self = np.zeros((self.num_collision_vol, self.num_collision_vol))
        if self.num_constraint_self > 0:
            mat_self[np.triu_indices(self.num_collision_vol, k=1)] = vec[-self.num_constraint_self:]
        
        ret = {
            f"{name}_mat_env": mat_env,
            f"{name}_mat_self": mat_self,
        }
        
        return ret
        