import mujoco
import numpy as np

from spark_agent.simulation.mujoco.galaxea_r1lite.galaxea_r1lite_fixed_base_agent import (
    GalaxeaR1LiteFixedBaseAgent,
)
from spark_robot import RobotConfig


class GalaxeaR1LiteRightArmAgent(GalaxeaR1LiteFixedBaseAgent):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

    def _send_control_sim_dynamics(self, command, **kwargs):
        action_info = kwargs.get("action_info")
        x = self._compose_simulator_dynamics_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)
        self.target_pos = self.dof_pos_cmd.copy()
        self.target_vel = self.dof_vel_cmd.copy()

        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(self.target_pos, self.target_vel)
                self._enforce_uncontrolled_actuator_holds()
                self._apply_controlled_joint_bias(ctrl)
            else:
                self._apply_controlled_joint_pd(ctrl, self.target_pos, self.target_vel)
            self._apply_uncontrolled_actuator_holds(ctrl)
            self._apply_r1lite_gripper_control(ctrl, action_info)
            self.data.ctrl[:] = ctrl
            self.counter += 1
            mujoco.mj_step(self.model, self.data)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(self.target_pos, self.target_vel)
            self._enforce_uncontrolled_actuator_holds()
