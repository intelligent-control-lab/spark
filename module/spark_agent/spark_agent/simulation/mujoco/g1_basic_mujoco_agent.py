from .mujoco_agent import MujocoAgent


class G1BasicMujocoAgent(MujocoAgent):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def compose_state(self):
        
        x = self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd)

        return x