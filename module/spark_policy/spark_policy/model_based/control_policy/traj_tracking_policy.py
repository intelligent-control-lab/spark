from spark_policy.base.base_policy import BasePolicy
from spark_robot import RobotKinematics, RobotConfig
import numpy as np

class TrajTrackingPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        self.target_traj = None
        self.traj_cnt = 0
        self.dof_pos_target_traj = []
        
    def tracking_pos_with_vel(self, 
                              dof_pos_target,
                              dof_vel_target,
                              dof_pos_current,
                              dof_vel_current):
        #NOTE: Parameters for real robot open loop control with cmd pos and vel
        # K_p = 120.0 * np.ones(len(self.robot_cfg.Control))
        # K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        # NOTE: Parameters for real robot closed loop control with fbk pos and vel
        # K_p = 50.0 * np.ones(len(self.robot_cfg.Control))
        # K_d = 5.0 * np.ones(len(self.robot_cfg.Control))
        # NOTE: Parameters for viz robot open loop control with fbk pos and vel
        # K_p = 10.0 * np.ones(len(self.robot_cfg.Control))
        # K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        # NOTE: Parameters for sim robot open loop control with fbk pos and vel
        K_p = 1.0 * np.ones(len(self.robot_cfg.Control))
        K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        dof_vel_nominal = K_p * (dof_pos_target - dof_pos_current) + K_d * (dof_vel_target - dof_vel_current)
        nominal_control = np.linalg.pinv(self.robot_cfg.dynamics_g(dof_pos_current)) @ dof_vel_nominal
        return nominal_control

    def tracking_pos_with_acc(self, 
                            dof_pos_target,
                            dof_vel_target,
                            dof_pos_current,
                            dof_vel_current):
        K_p = 1.0 * np.ones(len(self.robot_cfg.Control))
        K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        nominal_dof_vel = np.zeros_like(dof_vel_current)
        
        nominal_dof_vel = K_p * (dof_pos_target - dof_pos_current) + K_d * dof_vel_current
        nominal_dof_acc = nominal_dof_vel - dof_vel_current
        nominal_control = np.linalg.pinv(self.robot_cfg.dynamics_g(np.concatenate((dof_pos_current, dof_vel_current)))) @ np.concatenate((nominal_dof_vel, nominal_dof_acc))

        return nominal_control

    def plan(self, target_traj_cur, agent_feedback: dict, task_info: dict):
        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        dof_pos_fbk = agent_feedback["dof_pos_fbk"]
        dof_vel_fbk = agent_feedback["dof_vel_fbk"]
        goal_teleop = task_info["goal_teleop"]
        traj_len = len(goal_teleop["right"])
        if self.target_traj_changed(target_traj_cur):
            dof_pos_tmp = dof_pos_fbk.copy()
            for t in range(traj_len):
                if "left" not in goal_teleop:
                    dof_pos_target, _ = self.robot_kinematics.inverse_kinematics([target_traj_cur["right"][t]], dof_pos_tmp)
                else:
                    dof_pos_target, _ = self.robot_kinematics.inverse_kinematics([target_traj_cur["right"][t], 
                                                                                target_traj_cur["left"][t]], dof_pos_tmp)
                dof_pos_tmp = dof_pos_target
                self.dof_pos_target_traj.append(dof_pos_target)
            self.traj_cnt = 0
        else:
            # check if the dof_pos_fbk is close to the current target, if yes, move to next target
            if np.linalg.norm(dof_pos_fbk - self.dof_pos_target_traj[self.traj_cnt]) < 0.01:
                self.traj_cnt += 1
            if self.traj_cnt >= traj_len:
                self.traj_cnt = 0
        return self.dof_pos_target_traj[self.traj_cnt]
    
    def target_traj_changed(self, target_traj_cur):
        if self.target_traj is None:
            self.target_traj = target_traj_cur
            return True
        if len(self.target_traj["right"]) != len(target_traj_cur["right"]):
            self.target_traj = target_traj_cur
            return True
        for t in range(len(target_traj_cur["right"])):
            if not np.allclose(self.target_traj["right"][t], target_traj_cur["right"][t]):
                self.target_traj = target_traj_cur
                return True
            if "left" in target_traj_cur:
                if not np.allclose(self.target_traj["left"][t], target_traj_cur["left"][t]):
                    self.target_traj = target_traj_cur
                    return True
        return False

    def act(self, agent_feedback: dict, task_info: dict):
        info = {}
        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        dof_pos_fbk = agent_feedback["dof_pos_fbk"]
        dof_vel_fbk = agent_feedback["dof_vel_fbk"]

        robot_base_frame = agent_feedback["robot_base_frame"]
        target_traj_cur = {}
        target_traj_cur["right"] = np.linalg.inv(robot_base_frame) @ task_info["goal_teleop"]["right"]
        if "left" in task_info["goal_teleop"]:
            target_traj_cur["left"] = np.linalg.inv(robot_base_frame) @ task_info["goal_teleop"]["left"]


        try:
            dof_pos_target = self.plan(target_traj_cur, agent_feedback, task_info)

            if "Dynamic1" in self.robot_cfg.__class__.__name__:
                dof_vel_target = np.zeros_like(dof_vel_fbk)
                dof_pos_current = dof_pos_fbk
                dof_vel_current = dof_vel_fbk
                control = self.tracking_pos_with_vel(dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current)

            if "Dynamic2" in self.robot_cfg.__class__.__name__:
                dof_vel_target = np.zeros_like(dof_vel_fbk)
                dof_pos_current = dof_pos_cmd
                dof_vel_current = dof_vel_cmd
                control = self.tracking_pos_with_acc(dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current)
            
            info["ik_success"] = True
        
        except Exception as e:
            print("inverse_kinematics error", e)
        
            control = np.zeros_like(dof_pos_fbk)
            info["ik_success"] = False
            # raise e
            control = np.zeros_like(dof_pos_fbk)
        
        for control_id in self.robot_cfg.Control:
            control[control_id] = np.clip(control[control_id], -self.robot_cfg.ControlLimit[control_id], self.robot_cfg.ControlLimit[control_id])

        if "left_gripper_goal" in task_info["goal_teleop"]:
            info["left_gripper_goal"] = task_info["goal_teleop"]["left_gripper_goal"]
        if "right_gripper_goal" in task_info["goal_teleop"]:
            info["right_gripper_goal"] = task_info["goal_teleop"]["right_gripper_goal"]

        return control, info