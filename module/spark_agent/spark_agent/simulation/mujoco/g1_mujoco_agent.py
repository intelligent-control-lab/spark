from .mujoco_agent import MujocoAgent
import mujoco
import mujoco.viewer
import numpy as np
import os
import torch as torch
import time
from scipy.spatial.transform import Rotation as R
from spark_agent import SPARK_AGENT_ROOT
from spark_robot import RobotConfig, SPARK_ROBOT_RESOURCE_DIR
from spark_utils import Geometry, VizColor

class G1MujocoAgent(MujocoAgent):
    """
    A Mujoco-based agent for the G1 robot, extending MujocoAgent.
    Provides methods to compose command and feedback states.
    """
    
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        """
        Initializes the Mujoco agent with robot configuration and simulation parameters.

        Args:
            robot_cfg (RobotConfig): The configuration for the robot.
            **kwargs: Additional keyword arguments such as simulation dynamics, model path, viewer settings, etc.
        """
        super().__init__(robot_cfg)
        
        # Mode flags
        self.sport_mode = False
        self.whole_body_mode = False

        # Whether to use simulation dynamics
        self.use_sim_dynamics = kwargs.get("use_sim_dynamics", True)

        # Robot class name and Mujoco model initialization
        self.class_name = robot_cfg.__class__.__name__
        self.model = mujoco.MjModel.from_xml_path(os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.robot_cfg.mujoco_model_path))
        self.data = mujoco.MjData(self.model)
        
        # Target position initialized to zeros
        self.target_pos = np.zeros(len(self.data.qpos))
        
        # Set dynamic order based on class name
        if "Dynamic1" in self.class_name:
            self.dynamic_order = 1
        elif "Dynamic2" in self.class_name:
            self.dynamic_order = 2
        else:
            raise ValueError("Unknown dynamic order")

        # Handle special modes based on robot class
        if "SportMode" in self.class_name:
            self.sport_mode = True
            self.use_sim_dynamics = True  # Sports mode must use simulation dynamics
            self._init_sport_mode()
        
        if "WholeBody" in self.class_name:
            self.whole_body_mode = True
            self.use_sim_dynamics = True  # Whole-body mode must use simulation dynamics
            self._init_whole_body_mode()
        # Run forward simulation
        mujoco.mj_forward(self.model, self.data)

        # Viewer initialization
        self.enable_viewer = kwargs.get("enable_viewer", True)
        if self.enable_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data, key_callback=self._key_callback)
            self.viewer_setup()
            self.renderer = mujoco.Renderer(self.model)
        else:
            self.viewer = None
            self.renderer = None

        # Time and control settings
        self.counter = 0
        self.dt = kwargs["dt"]
        self.control_decimation = kwargs["control_decimation"]
        self.model.opt.timestep = self.dt

        # Initialize PID controllers for motor control
        self.kps = np.zeros_like(self.data.ctrl)
        self.kds = np.zeros_like(self.data.ctrl)
        for mj_ctrl_idx in self.robot_cfg.MujocoMotorKps:
            self.kps[mj_ctrl_idx] = self.robot_cfg.MujocoMotorKps[mj_ctrl_idx]
            self.kds[mj_ctrl_idx] = self.robot_cfg.MujocoMotorKds[mj_ctrl_idx]

        # Obstacle simulation debug settings
        self.obstacle_debug = kwargs.get("obstacle_debug", dict(num_obstacle=0, manual_movement_step_size=0.1))
        self.obstacle_debug_geom = []
        self.num_obstacle_debug = int(self.obstacle_debug["num_obstacle"])
        self.manual_step_size = self.obstacle_debug["manual_movement_step_size"]
        
        # If there are obstacles, initialize their positions and geometry
        if self.num_obstacle_debug > 0:
            self.obstacle_debug_frame = np.stack([np.eye(4) for _ in range(self.num_obstacle_debug)], axis=0)
            
            # Randomly place obstacles
            for frame in self.obstacle_debug_frame:
                frame[:3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
            
            # Define geometry for obstacles (currently using spheres)
            self.obstacle_debug_geom = [Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug) for _ in range(self.num_obstacle_debug)]
            # self.obstacle_debug_geom = [Geometry(type="box", length=0.4, width=0.01, height=0.4, color=VizColor.obstacle_debug) for _ in range(self.num_obstacle_debug)]
            self.obstacle_debug_selected = 0
            self.num_obstacle_debug_change_buf = 0
        else:
            self.obstacle_debug_frame = np.zeros((0, 4, 4))
            self.obstacle_debug_selected = None
            self.num_obstacle_debug_change_buf = 0
        
        self.left_goal_debug_frame = np.eye(4)
        self.right_goal_debug_frame = np.eye(4)
        
        # Store last frame for calculating obstacle velocity
        self.obstacle_debug_frame_last = self.obstacle_debug_frame.copy()
        
        # Calculate the velocity of obstacles based on position change
        self.obstacle_debug_velocity = [
            np.hstack(((self.obstacle_debug_frame[i][:3, 3] - self.obstacle_debug_frame_last[i][:3, 3]), np.zeros(3)))
            for i in range(self.num_obstacle_debug)
        ]
        
        self.debug_object = None  # Placeholder for the currently selected obstacle
        
    def _init_sport_mode(self):
        """
        Initializes the sport mode configuration for the Mujoco agent.
        This includes setting up action arrays, loading the policy, and configuring action scales.
        """
        if self.sport_mode:
            # Initialize variables related to sport mode
            self.sport_cmd = np.zeros(3)  # Command vector for sports actions
            self.target_dof_pos_lower_body = np.zeros(12, dtype=np.float32)  # Target position for lower body DOFs
            self.action = np.zeros(12, dtype=np.float32)  # Action vector
            self.obs = np.zeros(47, dtype=np.float32)  # Observation vector

            # Load policy for action control
            self.policy_path = os.path.join(SPARK_AGENT_ROOT, "spark_agent/simulation/mujoco/motion.pt")
            self.policy = torch.jit.load(self.policy_path)
            self.counter = 0  # Reset the action counter

            # Number of actions and action scaling parameters
            self.num_actions = 12
            self.cmd_scale = np.array([2.0, 2.0, 0.25], dtype=np.float32)  # Scaling for action commands
            self.default_pos_lower_body = [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, 
                                        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]  # Default position for the lower body
            self.dof_pos_scale = 1.0  # DOF position scaling
            self.dof_vel_scale = 0.05  # DOF velocity scaling
            self.ang_vel_scale = 0.25  # Angular velocity scaling
            self.action_scale = 0.25  # Action scale factor

    def _init_whole_body_mode(self):
        """
        Initializes the whole-body mode configuration for the Mujoco agent.
        This includes setting up target positions and velocities.
        """
        if self.whole_body_mode:
            # Initialize target positions and velocities for whole-body control
            self.target_pos = np.zeros_like(self.dof_pos_cmd)  # Target position array
            self.target_vel = np.zeros_like(self.dof_vel_cmd)  # Target velocity array

            # Default lower body position (same as in sport mode)
            self.default_pos_lower_body = [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, 
                                        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]

        # -------------------------------- Simulation Helpers -------------------------------- #

    def reset(self) -> None:
        """Resets the robot to its default position and configuration."""
        
        # Call the parent reset method (if any)
        super().reset()

        # Set the default position for the robot's DoFs
        self.default_dof_pos = np.array([self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs])
        
        # Adjust default position for sport mode if applicable
        if self.sport_mode:
            self.default_dof_pos = np.concatenate([np.array([0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0]), 
                                                  self.default_pos_lower_body, 
                                                  self.default_dof_pos[:-3]])
        
        # Set the robot's DoF positions to the default ones
        self._set_dof_pos(self.default_dof_pos)

        # Step through the simulation to update the robot's configuration
        self._mujoco_step() 

    def get_feedback(self) -> None:
        ret = {}

        # Feedback: Robot frame in world frame
        # Extract robot's global position and orientation
        global_position = self.data.body("robot").xpos.copy()  # [x, y, z]
        global_orientation = self.data.body("robot").xmat.copy().reshape(3, 3)  # 3x3 rotation matrix
        
        # Construct the 4x4 transformation matrix for the robot's base frame
        robot_base_frame = np.eye(4)  # Start with identity matrix
        robot_base_frame[:3, :3] = global_orientation  # Top-left 3x3 is the rotation matrix
        robot_base_frame[:3, 3] = global_position  # Translation vector
        
        ret["robot_base_frame"] = robot_base_frame

        # Feedback: Degrees of Freedom (DoF) positions
        rot = R.from_matrix(robot_base_frame[:3, :3])  # Rotation matrix to Euler angles
        euler = rot.as_euler("xyz")  # Euler angles in XYZ order
        
        # Initialize or reset feedback arrays for position and velocity
        self.dof_pos_fbk = np.zeros(self.num_dof) if self.dof_pos_fbk is None else self.dof_pos_fbk
        self.dof_vel_fbk = np.zeros(self.num_dof) if self.dof_vel_fbk is None else self.dof_vel_fbk

        # Update DoF position feedback based on the robot configuration
        for dof in self.robot_cfg.DoFs:
            if dof.name == "LinearX":
                self.dof_pos_fbk[dof] = robot_base_frame[0, 3]
            elif dof.name == "LinearY":
                self.dof_pos_fbk[dof] = robot_base_frame[1, 3]
            elif dof.name == "RotYaw":
                self.dof_pos_fbk[dof] = euler[2]
            else:
                mj_dof = self.robot_cfg.DoF_to_MujocoDoF[dof]
                self.dof_pos_fbk[dof] = self.data.qpos[mj_dof]
        
        # Update DoF velocity feedback based on mode (sport or whole body)
        if self.sport_mode:
            for dof in self.robot_cfg.DoFs:
                if dof.name == "LinearX":
                    self.dof_vel_fbk[dof] = self.data.qvel[0]
                elif dof.name == "LinearY":
                    self.dof_vel_fbk[dof] = self.data.qvel[1]
                elif dof.name == "RotYaw":
                    self.dof_vel_fbk[dof] = self.data.qvel[5]
        elif self.whole_body_mode:
        # Feedback for whole body mode: position and velocity
            ret["qpos_fbk"] = self.data.qpos
            ret["qvel_fbk"] = self.data.qvel
            self.dof_pos_fbk = self.data.qpos.copy()  # Set commanded position to feedback position
            self.dof_vel_fbk = self.data.qvel.copy()  # Set commanded velocity to feedback velocity
        else:
            # Default: Update DoF velocities from MuJoCo data
            for dof in self.robot_cfg.DoFs:
                mj_dof = self.robot_cfg.DoF_to_MujocoDoF[dof]
                self.dof_vel_fbk[dof] = self.data.qvel[mj_dof]

        # Initialize commanded DoF values if they are None
        self.dof_pos_cmd = self.dof_pos_fbk.copy() if self.dof_pos_cmd is None else self.dof_pos_cmd
        self.dof_vel_cmd = self.dof_vel_fbk.copy() if self.dof_vel_cmd is None else self.dof_vel_cmd
        self.dof_acc_cmd = np.zeros(self.num_dof) if self.dof_acc_cmd is None else self.dof_acc_cmd
        
        # If not using simulated dynamics, set feedback to commanded values
        if not self.use_sim_dynamics:
            self.dof_pos_fbk = self.dof_pos_cmd.copy()
            self.dof_vel_fbk = self.dof_vel_cmd.copy()

        # Collect and return feedback
        ret["dof_pos_fbk"] = self.dof_pos_fbk.copy()
        ret["dof_vel_fbk"] = self.dof_vel_fbk.copy()
        ret["dof_pos_cmd"] = self.dof_pos_cmd.copy()
        ret["dof_vel_cmd"] = self.dof_vel_cmd.copy()
        ret["dof_acc_cmd"] = self.dof_acc_cmd.copy()

        # Dynamics state feedback
        ret["state"] = self.compose_fbk_state()

        # Virtual obstacle feedback
        ret["obstacle_debug_frame"] = self.obstacle_debug_frame
        ret["obstacle_debug_geom"] = self.obstacle_debug_geom
        ret["obstacle_debug_velocity"] = np.array(self.obstacle_debug_velocity) if len(self.obstacle_debug_velocity) > 0 else np.empty((0, 6))
        
        # Update goal position
        ret["robot_goal_left_offset"] = self.left_goal_debug_frame
        ret["robot_goal_right_offset"] = self.right_goal_debug_frame
        
        return ret
    
    def _pd_control(self, target_q, q, kp, target_dq, dq, kd):
        """Calculates torques from position commands using PD control."""
        
        # PD control formula to calculate torques
        # (target position - current position) * proportional gain
        # + (target velocity - current velocity) * derivative gain
        return (target_q - q) * kp + (target_dq - dq) * kd
        
    def _set_dof_pos(self, dof_pos: np.ndarray) -> None:
        """Sets the Degrees of Freedom (DoF) positions for the robot."""
        
        # If sport_mode or whole_body_mode is enabled, directly set qpos
        if self.sport_mode or self.whole_body_mode:
            qpos = dof_pos
        else:
            qpos = np.zeros(self.model.nq)  # Initialize qpos with zeros
            # Map the DoF positions to Mujoco DoFs based on the robot configuration
            for mj_dof_idx in self.robot_cfg.MujocoDoFs:
                dof_idx = self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx]
                qpos[mj_dof_idx] = dof_pos[dof_idx]
              
        # Set the robot's configuration in the model data
        self.data.qpos = qpos
        # Reset velocities, accelerations, and applied forces
        self.data.qvel[:] = 0              # Clear velocities
        self.data.qacc[:] = 0              # Clear accelerations
        self.data.qfrc_applied[:] = 0      # Clear applied forces
        self.data.xfrc_applied[:, :] = 0   # Clear external forces

    def _mujoco_step(self):
        """Performs a simulation step to update the model and data."""
        
        # Perform forward kinematics (update the model based on current state)
        mujoco.mj_forward(self.model, self.data)
        
        # If viewer is available, reset the number of geometries in the scene
        if self.viewer:
            self.viewer.user_scn.ngeom = 0
        
        # If renderer is available, reset the number of geometries in the scene
        if self.renderer:
            self.renderer._scene.ngeom = 0
        
        # Sleep for the timestep duration to simulate the time progression
        time.sleep(self.model.opt.timestep)

    def _send_control_sport_mode(self, command):
        """
        This method controls the robot in sport mode by adjusting its joint positions and applying control signals.
        """
        # Initialize target positions for joints
        self.target_pos = np.zeros(len(self.data.qpos))

        # Determine the sport mode command based on the dynamic order
        if self.dynamic_order == 1:
            self.sport_cmd = command[-3:]
        elif self.dynamic_order == 2:
            self.sport_cmd += command[-3:]
        
        # Clip sport command to avoid extreme values
        self.sport_cmd = np.clip(self.sport_cmd, -0.3, 0.3)

        # Set target positions for the lower body and the rest of the joints
        self.target_pos[7:19] = self.target_dof_pos_lower_body
        self.target_pos[19:] = self.dof_pos_cmd[:-3]

        # Generate observations from the current robot state
        qj = self.data.qpos[7:19]  # Joint positions (lower body)
        dqj = self.data.qvel[6:18]  # Joint velocities
        quat = self.data.qpos[3:7]  # Quaternion for orientation
        omega = self.data.qvel[3:6]  # Angular velocity
        qw, qx, qy, qz = quat

        # Calculate gravity orientation from the robot's orientation
        gravity_orientation = np.zeros(3)
        gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
        gravity_orientation[1] = -2 * (qz * qy + qw * qx)
        gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

        # Scale joint positions, velocities, and angular velocities for control
        qj = (qj - self.default_pos_lower_body) * self.dof_pos_scale
        dqj = dqj * self.dof_vel_scale
        omega = omega * self.ang_vel_scale

        # Generate a periodic signal (sine and cosine) based on time for rhythmic control
        period = 0.8
        count = self.counter * self.dt
        phase = count % period / period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        # Populate observation array with robot states
        self.obs[:3] = omega
        self.obs[3:6] = gravity_orientation
        self.obs[6:9] = self.sport_cmd * self.cmd_scale
        self.obs[9 : 9 + self.num_actions] = qj
        self.obs[9 + self.num_actions : 9 + 2 * self.num_actions] = dqj
        self.obs[9 + 2 * self.num_actions : 9 + 3 * self.num_actions] = self.action
        self.obs[9 + 3 * self.num_actions : 9 + 3 * self.num_actions + 2] = np.array([sin_phase, cos_phase])

        # Perform policy inference using the current observation
        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
        self.action = self.policy(obs_tensor).detach().numpy().squeeze()

        # Transform the inferred action to target lower body joint positions
        self.target_dof_pos_lower_body = self.action * self.action_scale + self.default_pos_lower_body

        # Apply control decimation for smoother control (multiple steps per control cycle)
        for _ in range(self.control_decimation):
            tau = self._pd_control(self.target_pos[7:], 
                                self.data.qpos[7:], 
                                self.kps, 
                                np.zeros_like(self.kds), 
                                self.data.qvel[6:], 
                                self.kds)
            self.data.ctrl[:] = tau
            mujoco.mj_step(self.model, self.data)
            self.counter += 1

    def _send_control_whole_body_mode(self, command, action_info):
        """
        This method applies control signals directly to the whole body of the robot.
        """
        # Apply the given command to all actuators (joints)
        target_dof_acc = action_info["sol_acc"][6:]
        target_torque = action_info["sol_torque"]
        target_contact_force = action_info["sol_contact_force"]
        
        
        # for _ in range(self.control_decimation):
        dof_pos_fbk = self.data.qpos.copy()[7:]
        dof_vel_fbk = self.data.qvel.copy()[6:]
        target_dof_vel = dof_vel_fbk + target_dof_acc * self.dt 
        target_dof_pos = dof_pos_fbk + 0.5 * target_dof_acc * (self.dt ) ** 2
        ctrl = target_torque + self.kps * (target_dof_pos - dof_pos_fbk) + self.kds * (target_dof_vel - dof_vel_fbk)
        self.data.ctrl = ctrl
        
        self.counter += 1
        mujoco.mj_step(self.model, self.data)   

    def _send_control_sim_dynamics(self, command, **kwargs):
        """
        This method integrates the robot's dynamics and updates its state based on modeled dynamics.
        """
        if self.whole_body_mode:
            self._send_control_whole_body_mode(command, kwargs.get("action_info", {}))
        else:
            # Get the current state of the robot and apply the commanded dynamics
            x = self.compose_cmd_state()
            x_dot = self.robot_cfg.dynamics_xdot(x, command)

            # Integrate the state over the control timestep
            x += x_dot * self.dt * self.control_decimation

            # Decompose the state into degree-of-freedom (DoF) positions and velocities
            self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
            self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

            # Apply control in either sport mode or whole-body mode
            if self.sport_mode:
                self._send_control_sport_mode(command)
            else:
                # Set target positions and velocities for other modes
                self.target_pos = np.zeros_like(self.dof_pos_cmd)
                self.target_vel = np.zeros_like(self.dof_vel_cmd)
                
                # Apply control for each degree-of-freedom (DoF)
                for dof in self.robot_cfg.MujocoDoFs:
                    dof_idx = self.robot_cfg.MujocoDoF_to_DoF[dof]
                    if dof.name == "LinearX":
                        self.target_vel[dof] = command[self.robot_cfg.Control.vLinearX] if self.dynamic_order == 1 else self.dof_vel_cmd[dof_idx]
                    elif dof.name == "LinearY":
                        self.target_vel[dof] = command[self.robot_cfg.Control.vLinearY] if self.dynamic_order == 1 else self.dof_vel_cmd[dof_idx]
                    elif dof.name == "RotYaw":
                        self.target_vel[dof] = command[self.robot_cfg.Control.vRotYaw] if self.dynamic_order == 1 else self.dof_vel_cmd[dof_idx]
                    else:
                        self.target_pos[dof] = self.dof_pos_cmd[dof_idx]
                        self.target_vel[dof] = self.dof_vel_cmd[dof_idx]

                # Apply PD control to the robot's actuators
                for _ in range(self.control_decimation):
                    tau = self._pd_control(self.target_pos, 
                                        self.data.qpos, 
                                        self.kps, 
                                        self.target_vel, 
                                        self.data.qvel, 
                                        self.kds)
                    self.data.ctrl[:] = tau
                    self.counter += 1
                    mujoco.mj_step(self.model, self.data)

    def _send_control_modeled_dynamics(self, command, **kwargs):
        """
        This method invokes modeled dynamics to simulate the robot's state changes and applies control.
        """
        # Get the current robot state and apply the dynamics model
        x = self.compose_cmd_state()
        x_dot = self.robot_cfg.dynamics_xdot(x, command)

        # Integrate the state over the control timestep
        x += x_dot * self.dt * self.control_decimation

        # Decompose the state to update degree-of-freedom (DoF) positions
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)
        
        # Set the degree-of-freedom (DoF) positions based on the updated state
        self._set_dof_pos(self.dof_pos_cmd)

    def _post_control_processing(self, **kwargs):
        """
        This method processes control feedback, updates obstacles, and steps the simulation forward.
        """
        num_obstacle_debug_change = self.num_obstacle_debug_change_buf
        self.num_obstacle_debug_change_buf -= num_obstacle_debug_change  # Prevent missing keyboard input

        # Add or remove obstacles based on debug feedback
        while num_obstacle_debug_change > 0:
            self._add_obstacle()
            num_obstacle_debug_change -= 1
        while num_obstacle_debug_change < 0:
            self._remove_obstacle()
            num_obstacle_debug_change += 1

        # Update obstacle velocity based on the change in their frames

        self.obstacle_debug_velocity = [np.hstack(((self.obstacle_debug_frame[i][:3, 3] - self.obstacle_debug_frame_last[i][:3, 3]), np.zeros(3))) 
                                        for i in range(self.num_obstacle_debug)]
        self.obstacle_debug_frame_last = self.obstacle_debug_frame.copy()

        # Step the simulation forward
        self._mujoco_step()
        

    def compose_cmd_state(self):
        """
        Compose the command state representation based on DoF position and velocity commands.
        
        Returns:
            np.ndarray: Composed command state.
        """
        return self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd, self.dof_vel_cmd)
    
    def compose_fbk_state(self):
        """
        Compose the feedback state representation based on DoF position and velocity feedback.
        
        Returns:
            np.ndarray: Composed feedback state.
        """
        return self.robot_cfg.compose_state_from_dof(self.dof_pos_fbk, self.dof_vel_fbk)