from spark_policy.base.base_policy import BasePolicy
from spark_robot import RobotKinematics, RobotConfig
import numpy as np

class TeleopPIDPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        
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

    def act(self, agent_feedback: dict, task_info: dict):
        info = {}
        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        dof_pos_fbk = agent_feedback["dof_pos_fbk"]
        dof_vel_fbk = agent_feedback["dof_vel_fbk"]
        robot_base_frame = agent_feedback["robot_base_frame"]
        goal_teleop = task_info["goal_teleop"]
        try:
            if "left" not in goal_teleop:
                dof_pos_target, _ = self.robot_kinematics.inverse_kinematics([np.linalg.inv(robot_base_frame) @ goal_teleop["right"][0]], dof_pos_fbk)
            else:
                dof_pos_target, _ = self.robot_kinematics.inverse_kinematics([np.linalg.inv(robot_base_frame) @ goal_teleop["right"][0], 
                                                                            np.linalg.inv(robot_base_frame) @ goal_teleop["left"][0]], dof_pos_fbk)
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