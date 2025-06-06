from spark_robot import RobotKinematics
from spark_utils import Geometry, Volume, compute_pairwise_distance, compute_pairwise_distance_info
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
        self.num_robot_vol        = len(self.collision_vol_frame_ids)
        self.enable_self_collision    = kwargs["enable_self_collision"]
        self.env_collision_vol_ignore = self.robot_cfg.EnvCollisionVolIgnored
        
        self.num_constraint_self = self.num_robot_vol * (self.num_robot_vol - 1)  // 2
        
        # create mask for self collision check
        self.self_collision_mask = self.enable_self_collision * np.ones((self.num_robot_vol, self.num_robot_vol), dtype=bool)
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
            
        self.min_dist_self = self.min_distance["self"] * np.ones((self.num_robot_vol, self.num_robot_vol))
        
        self.update_obstacle_info()
    
    def update_obstacle_info(self, task_info=None):
        
        if task_info is not None:
            self.num_obstacle_vol = task_info["obstacle"]["num"]
        else:
            self.num_obstacle_vol = 0
        
        # compute number of constraints
        self.num_constraint_env  = self.num_robot_vol * self.num_obstacle_vol
        self.num_constraint      = self.num_constraint_env + self.num_constraint_self
        
        # create mask for environment collision check
        self.env_collision_mask = np.ones((self.num_robot_vol, self.num_obstacle_vol), dtype=bool)
        for fid in self.env_collision_vol_ignore:
            i = self.collision_vol_frame_ids.index(fid)
            self.env_collision_mask[i, :] = False
        
        # generate constraint mask
        self.phi_mask = np.zeros(self.num_constraint, dtype=bool)
        self.phi_mask[:self.num_constraint_env] = self.env_collision_mask.flatten()
        if self.num_constraint_self > 0:
            self.phi_mask[-self.num_constraint_self:] = self.self_collision_mask[np.triu_indices(self.self_collision_mask.shape[0], k=1)]

        # min dist matrix
        self.min_dist_env = self.min_distance["environment"] * np.ones((self.num_robot_vol, self.num_obstacle_vol))

        self.dmin = np.zeros(self.num_constraint)
        self.dmin[:self.num_constraint_env] = self.min_distance["environment"]
        if self.num_constraint_self > 0:
            self.dmin[-self.num_constraint_self:] = self.min_distance["self"]

    def get_vol_info(self, x, task_info):
        assert "obstacle" in task_info, "obstacle not found in task_info"
        assert "frames_world" in task_info["obstacle"], "frames_world not found in task_info['obstacle']"
        assert "geom" in task_info["obstacle"], "geom not found in task_info['obstacle']"
        
        self.update_obstacle_info(task_info)
        
        trans_world2base          = task_info["robot_base_frame"]
        obstacle_vol_frames       = task_info["obstacle"]["frames_world"]
        obstacle_vol_geom         = task_info["obstacle"]["geom"]
        obstacle_vol_vel          = task_info["obstacle"]["velocity"]
        
        # compute all robot frames
        robot_frames = self.robot_kinematics.forward_kinematics(self.robot_cfg.decompose_state_to_dof_pos(x))
        trans_world2base = self.robot_kinematics.update_base_frame(trans_world2base, self.robot_cfg.decompose_state_to_dof_pos(x))
        # get collision vol frames
        robot_vol_frames = np.stack([robot_frames[fid] for fid in self.collision_vol_frame_ids], axis=0) # [num_robot_vol, 4, 4]
        # transform to world frame
           
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        dof_vel = self.robot_cfg.decompose_state_to_dof_vel(x)
        self.robot_kinematics.pre_computation(dof_pos, dof_vel)
        robot_collision_vol = []
        obstacle_collision_vol = []
        for i in range(self.num_obstacle_vol):
            obstacle_collision_vol.append(Volume(
                frame = obstacle_vol_frames[i],
                geometry  = obstacle_vol_geom[i],
                velocity = obstacle_vol_vel[i]
            ))
        for frame in self.robot_cfg.Frames:
            jacobian = self.robot_kinematics.get_jacobian(frame.name)
            robot_collision_vol.append(Volume(
                frame = trans_world2base @ robot_vol_frames[frame, :, :],
                geometry  = self.collision_vol_geom[frame],
                velocity = jacobian @ dof_vel
            ))
        return robot_collision_vol, obstacle_collision_vol

    def compute_pairwise_info(self, volume_list_1, volume_list_2):
        """
        Compute pairwise information between two sets of volumes.
        
        Args:
            volume_list_1 (List[Volume]): List of volumes in set 1.
            volume_list_2 (List[Volume]): List of volumes in set 2.
        
        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: Tuple containing the distances, velocities, normals, and curvatures.
        """
        return compute_pairwise_distance_info(
            np.array([volume.frame for volume in volume_list_1]),
            np.array([volume.velocity for volume in volume_list_1]),
            np.array([volume.geometry.type_code for volume in volume_list_1]),
            np.array([volume.geometry.size for volume in volume_list_1]),
            np.array([volume.frame for volume in volume_list_2]),
            np.array([volume.velocity for volume in volume_list_2]),
            np.array([volume.geometry.type_code for volume in volume_list_2]),
            np.array([volume.geometry.size for volume in volume_list_2]),
        )

    def compute_pairwise_vol_info(self, robot_collision_vol, obstacle_collision_vol):
        d_env, v_env, normal_env, curv_env = self.compute_pairwise_info(robot_collision_vol, obstacle_collision_vol)
        d_self, v_self, normal_self, curv_self = self.compute_pairwise_info(robot_collision_vol, robot_collision_vol)
        
        d = np.zeros((self.num_constraint, 3))
        v = np.zeros((self.num_constraint, 3))
        normal = np.zeros((self.num_constraint, 3))
        curv = np.zeros((self.num_constraint, 3, 3))
        
        d[:self.num_constraint_env] = d_env.reshape(-1, 3)  
        v[:self.num_constraint_env] = v_env.reshape(-1, 3)  
        normal[:self.num_constraint_env] = normal_env.reshape(-1, 3)  
        curv[:self.num_constraint_env] = curv_env.reshape(-1, 3, 3)
        if self.num_constraint_self > 0:
            d[-self.num_constraint_self:] = d_self[np.triu_indices(d_self.shape[0], k=1)]
            v[-self.num_constraint_self:] = v_self[np.triu_indices(v_self.shape[0], k=1)]
            normal[-self.num_constraint_self:] = normal_self[np.triu_indices(normal_self.shape[0], k=1)]
            curv[-self.num_constraint_self:] = curv_self[np.triu_indices(curv_self.shape[0], k=1)]
        
        return d, v, normal, curv

    def phi_cartesian(self,
                        robot_frame_closest : np.ndarray,
                        task_info: dict,
                        trans_world2base    : np.ndarray):
        '''
            [num_constraint,] Safety index function.

            Args:
                robot_frame_closest (np.ndarray): [num_state,] closest point on robot.
        '''
        
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
        dist_env = compute_pairwise_distance(
            frame_list_1 = robot_frame_closest,
            geom_list_1  = cloest_robot_geom,
            frame_list_2 = obstacle_vol_frames_world,
            geom_list_2  = obstacle_vol_geom
        )
        self.min_dist_env = self.min_distance["environment"] * np.ones(self.num_obstacle_vol)
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
        
        grad = np.zeros((self.num_obstacle_vol, 3))
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
            Decode the constraint vector into matrices.
        '''
            
        assert vec.shape[0] == self.num_constraint, f"vec shape {vec.shape} does not match num_constraint {self.num_constraint}"
        
        mat_env = vec[:self.num_constraint_env].reshape(self.num_robot_vol, self.num_obstacle_vol)
        mat_self = np.zeros((self.num_robot_vol, self.num_robot_vol))
        if self.num_constraint_self > 0:
            mat_self[np.triu_indices(self.num_robot_vol, k=1)] = vec[-self.num_constraint_self:]
        
        ret = {
            f"{name}_mat_env": mat_env,
            f"{name}_mat_self": mat_self,
        }
        
        return ret
        
    def jacobian_full(self, dof_pos):
        self.robot_kinematics.pre_computation(dof_pos)
        jacobian = np.array([self.robot_kinematics.get_jacobian(frame.name)[:3, :] for frame in self.robot_cfg.Frames])
        # for frame in self.robot_cfg.Frames:
        #     # jacobian[frame_id] = self.robot_kinematics.compute_frame_jacobian(q, frame_id)[:3, :]
        #     jacobian.append(self.robot_kinematics.get_jacobian(frame.name)[:3, :])
        
        jacobian_full_env = np.zeros((self.num_robot_vol, self.num_obstacle_vol, 3, jacobian.shape[2]))
        jacobian_full_env[:, :, :, :] = jacobian[:, np.newaxis, :, :]
        jacobian_full = np.zeros((self.num_constraint, 3, jacobian.shape[2]))
        jacobian_full[:self.num_constraint_env, :, :] = jacobian_full_env.reshape(-1, 3, jacobian.shape[2])
        
        if self.num_constraint_self > 0:
            jacobian_full_self = np.zeros((self.num_robot_vol, self.num_robot_vol, 3, jacobian.shape[2]))
            jacobian_full_self[:, :, :, :] = jacobian[:, np.newaxis, :, :]
            jacobian_full[self.num_constraint_env:, :, :] = jacobian_full_self[np.triu_indices(self.num_robot_vol, k=1)].reshape(-1, 3, jacobian.shape[2])
        
        return jacobian_full
        
    def jacobian_dot_full(self, dof_pos, dof_vel):
        self.robot_kinematics.pre_computation(dof_pos, dof_vel)
        jacobian_dot = np.array([self.robot_kinematics.get_jacobian_dot(frame.name)[:3, :] for frame in self.robot_cfg.Frames])
        # for frame in self.robot_cfg.Frames:
        #     # jacobian_dot[frame_id] = self.robot_kinematics.compute_frame_jacobian_dot(q, dq, frame_id)[:3, :]
        #     jacobian_dot[frame] = self.robot_kinematics.get_jacobian_dot(frame.name)[:3, :]
            
        jacobian_dot_full_env = np.zeros((self.num_robot_vol, self.num_obstacle_vol, 3, jacobian_dot.shape[2]))
        jacobian_dot_full_env[:, :, :, :] = jacobian_dot[:, np.newaxis, :, :]
        jacobian_dot_full = np.zeros((self.num_constraint, 3, jacobian_dot.shape[2]))
        jacobian_dot_full[:self.num_constraint_env, :, :] = jacobian_dot_full_env.reshape(-1, 3, jacobian_dot.shape[2])
        
        if self.num_constraint_self > 0:
            jacobian_dot_full_self = np.zeros((self.num_robot_vol, self.num_robot_vol, 3, jacobian_dot.shape[2]))
            jacobian_dot_full_self[:, :, :, :] = jacobian_dot[:, np.newaxis, :, :]
            jacobian_dot_full[self.num_constraint_env:, :, :] = jacobian_dot_full_self[np.triu_indices(self.num_robot_vol, k=1)].reshape(-1, 3, jacobian_dot.shape[2])
        
        return jacobian_dot_full