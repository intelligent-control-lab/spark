from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
import mujoco
import mujoco.viewer
import numpy as np
import os
import time
from scipy.spatial.transform import Rotation as R
from spark_agent import SPARK_AGENT_ROOT
from spark_robot import RobotConfig, SPARK_ROBOT_RESOURCE_DIR
from spark_utils import Geometry, VizColor
import cv2
import imageio

class R1LiteMujocoFixedBaseAgent(MujocoAgent):
    """
    A Mujoco-based agent for the IIWA 14 robot, extending MujocoAgent.
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

        # Run forward simulation
        mujoco.mj_forward(self.model, self.data)
        
        self.width, self.height = 640, 480
        self.enable_viewer = kwargs.get("enable_viewer", True)
        self.enable_camera = kwargs.get("enable_camera", False)
        if self.enable_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data, key_callback=self._key_callback)
            self.viewer_setup()
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            self.depth_renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)  
            self.depth_renderer.enable_depth_rendering()
        else:
            self.viewer = None
            self.renderer = None

        if self.enable_camera:
            self.cam_step = 0

            # Define two virtual cameras
            self.fixed_cam = mujoco.MjvCamera()
            self.head_cam = mujoco.MjvCamera()
            self.right_cam = mujoco.MjvCamera()
            self.left_cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(self.fixed_cam)
            mujoco.mjv_defaultCamera(self.head_cam)
            mujoco.mjv_defaultCamera(self.fixed_cam)
            mujoco.mjv_defaultCamera(self.left_cam)

            # Position each camera
            self.fixed_cam.lookat[:] = [0.4, 0.3, 1.0]
            self.fixed_cam.distance = 0.4
            self.fixed_cam.elevation = -60
            self.fixed_cam.azimuth = 0

            self.head_cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.head_cam.fixedcamid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_head_left")

            self.right_cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.right_cam.fixedcamid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "right_d405_cam")

            self.left_cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.left_cam.fixedcamid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "left_d405_cam")

        # Option settings
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.opt)


        # Time and control settings
        self.counter = 0
        self.dt = kwargs["dt"]
        self.control_decimation = kwargs["control_decimation"]
        self.model.opt.timestep = self.dt

        # Initialize PID controllers for motor control
        self.kps = np.zeros(self.num_dof+4)
        self.kds = np.zeros(self.num_dof+4)
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
        
        self.right_gripper_picking = None
        self.left_gripper_picking = None
    
    # ---------------------------------- Setup Helpers --------------------------------- #

    def close_viewer(self):
        """
        Closes the Mujoco viewer if it is open.
        """
        if self.viewer:
            self.viewer.close()  # Close the viewer window
            self.viewer = None  # Set viewer to None

    def viewer_setup(self):
        """
        Configures the viewer camera settings for optimal visualization.
        """
        # Camera distance and positioning
        self.viewer.cam.distance = 3.5  # Set zoom level (distance from the object)
        self.viewer.cam.lookat[0] = 0  # X offset for the camera focus point
        self.viewer.cam.lookat[1] = 0  # Y offset for the camera focus point
        self.viewer.cam.lookat[2] = 1.0  # Z offset for the camera focus point

        # Camera rotations and angles
        self.viewer.cam.elevation = -20  # Camera rotation around the X axis (up/down)
        self.viewer.cam.azimuth = 180  # Camera rotation around the Y axis (left/right)

        # Set geometry group for visualization
        self.viewer.opt.geomgroup = 1  # Set which geometry group to visualize
    
    def render_camera(self, cam):     
        self.cam_step += 1
        self.renderer.update_scene(self.data, camera=cam)
        self.depth_renderer.update_scene(self.data, camera=cam)
        rgb_buffer = self.renderer.render()
        depth_buffer = self.depth_renderer.render()
        # if self.cam_step == 100:
        #     imageio.imwrite("rgb.png", rgb_buffer)

        #     # Convert depth (in meters) to millimeters and scale for visualization
        #     depth_vis = (depth_buffer * 1000).astype(np.uint16)  # depth in mm as 16-bit PNG
        #     imageio.imwrite("depth.png", depth_vis)
        return rgb_buffer.copy(), depth_buffer.copy()
    
    def stream_image(self, rgb_image, win_size=(320, 240)):
        bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        bgr_resized = cv2.resize(bgr, win_size, interpolation=cv2.INTER_AREA)
        cv2.imshow("MuJoCo Camera", bgr_resized)
        cv2.waitKey(1)

    def stream_depth(self, depth_image, win_size=(320, 240)):
        depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = depth_norm.astype(np.uint8)
        depth_resized = cv2.resize(depth_display, win_size, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("MuJoCo Depth", depth_resized)
        cv2.waitKey(1)

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
        
        ret["object_pos"] = {}
        for b in range(1, self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or f"body_{b}"
            p = self.data.xpos[b].copy()          # (3,)
            q = self.data.xquat[b].copy()         # (4,) quaternion (w, x, y, z)
            ret["object_pos"][name] = np.hstack((p, q))  # Combine position and orientation

        # Feedback: Robot frame in world frame
        # Extract robot's global position and orientation
        global_position = self.data.body("robot").xpos.copy()  # [x, y, z]
        global_orientation = self.data.body("robot").xmat.copy().reshape(3, 3)  # 3x3 rotation matrix
        # Construct the 4x4 transformation matrix for the robot's base frame
        robot_base_frame = np.eye(4)  # Start with identity matrix
        robot_base_frame[:3, :3] = global_orientation  # Top-left 3x3 is the rotation matrix
        robot_base_frame[:3, 3] = global_position  # Translation vector
        
        ret["robot_base_frame"] = robot_base_frame
        
        # Initialize or reset feedback arrays for position and velocity
        self.dof_pos_fbk = np.zeros(self.num_dof) if self.dof_pos_fbk is None else self.dof_pos_fbk
        self.dof_vel_fbk = np.zeros(self.num_dof) if self.dof_vel_fbk is None else self.dof_vel_fbk

        # Update DoF position feedback based on the robot configuration
        for dof in self.robot_cfg.DoFs:
            mj_dof = self.robot_cfg.DoF_to_MujocoDoF[dof]
            self.dof_pos_fbk[dof] = self.data.qpos[mj_dof]
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
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            dof_idx = self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx]
            self.data.qpos[mj_dof_idx] = dof_pos[dof_idx]
              
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
            
        if self.enable_camera:
            img_fixed, depth_fixed = self.render_camera(self.fixed_cam)
            # img_head, depth_head = self.render_camera(self.head_cam)
            # img_right, depth_right = self.render_camera(self.right_cam)
            # img_left, depth_left = self.render_camera(self.left_cam)


            self.stream_image(img_fixed)
            self.stream_depth(depth_fixed)


        # Sleep for the timestep duration to simulate the time progression
        time.sleep(self.model.opt.timestep)

    def _send_control_sim_dynamics(self, command, **kwargs):
        """
        This method integrates the robot's dynamics and updates its state based on modeled dynamics.
        """
        # Get the current state of the robot and apply the commanded dynamics
        x = self.compose_cmd_state()
        x_dot = self.robot_cfg.dynamics_xdot(x, command)

        # Integrate the state over the control timestep
        x += x_dot * self.dt * self.control_decimation

        # Decompose the state into degree-of-freedom (DoF) positions and velocities
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)
        
        # Set target positions and velocities for other modes
        self.target_pos = np.zeros_like(self.dof_pos_cmd)
        self.target_vel = np.zeros_like(self.dof_vel_cmd)

        self.target_pos = self.dof_pos_cmd.copy()
        self.target_vel = self.dof_vel_cmd.copy()
        
        action_info = kwargs.get("action_info", dict())
        left_gripper_goal = action_info.get("left_gripper_goal", False)
        right_gripper_goal = action_info.get("right_gripper_goal", False)
        # Apply PD control to the robot's actuators
        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl))
            ctrl[:9] = self.target_pos[:9]  # Apply control to the first 7 DoFs (right arm)
            ctrl[11:17] = self.target_pos[9:]  # Apply control to the next
            
            if left_gripper_goal:
                ctrl[9] = 0
                ctrl[10] = 0
            else:
                ctrl[9] = 0.05
                ctrl[10] = -0.05
            if right_gripper_goal:
                ctrl[17] = 0
                ctrl[18] = 0
            else:
                ctrl[17] = 0.05
                ctrl[18] = -0.05
                        
            self.data.ctrl[:] = ctrl
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