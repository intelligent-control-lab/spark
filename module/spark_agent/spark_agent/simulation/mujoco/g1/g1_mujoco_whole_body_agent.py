from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
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

class G1MujocoWholeBodyAgent(MujocoAgent):
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
        
    def _init_whole_body_mode(self):
        """
        Initializes the whole-body mode configuration for the Mujoco agent.
        This includes setting up target positions and velocities.
        """
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

        # Feedback for whole body mode: position and velocity
        ret["qpos_fbk"] = self.data.qpos
        ret["qvel_fbk"] = self.data.qvel
        self.dof_pos_fbk = self.data.qpos.copy()  # Set commanded position to feedback position
        self.dof_vel_fbk = self.data.qvel.copy()  # Set commanded velocity to feedback velocity

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
        
        qpos = dof_pos

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

    def _send_control_whole_body_mode(self, command, action_info):
        """
        This method applies control signals directly to the whole body of the robot.
        """
        # Apply the given command to all actuators (joints)
        target_dof_acc = action_info["sol_acc"][6:]
        target_torque = action_info["sol_torque"]
        target_contact_force = action_info["sol_contact_force"]
        
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
        self._send_control_whole_body_mode(command, kwargs.get("action_info", {}))
        
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