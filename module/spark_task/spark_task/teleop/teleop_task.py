from spark_task.base.base_goal_task import BaseGoalTask
from spark_task.utils.task_objects import TaskObject3D
from spark_utils import Geometry, VizColor
from spark_utils import pos_quat_to_transformation
import numpy as np
import json
from pathlib import Path


class TeleopTask(BaseGoalTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent, **kwargs)

        # Initialize task configuration
        self.task_name = kwargs.get("task_name", "TeleopTask")  # Name of the task
        self.max_episode_length = kwargs.get(
            "max_episode_length", -1
        )  # Maximum number of steps per episode
        self.gripper_debug_print = bool(kwargs.get("gripper_debug_print", False))
        self._last_gripper_debug_time = 0.0
        self.upper_body_joint_target_callback = None
        self.upper_body_joint_target_mask_callback = None
        self.target_dof_pos_callback = None
        self.teleop_upper_body_mode = str(kwargs.get("teleop_upper_body_mode", "cartesian")).lower()
        self.environment_representation = kwargs.get("environment_representation", "sphere")
        self.points_per_obstacle = max(4, int(kwargs.get("points_per_obstacle", 64)))
        self.max_visualized_points = max(1, int(kwargs.get("max_visualized_points", 200)))
        self._environment_surface_points = None
        self._environment_point_geometries = None
        self.object_mesh_directory = kwargs.get("object_mesh_directory")
        self.object_mesh_scale = float(kwargs.get("object_mesh_scale", 1.0))
        self._environment_mesh_names = []

        if self.enable_ros:
            self.ros_args = kwargs["ros_params"]
            self.ros_version = kwargs.get("ros_version", "ros1")
            self._init_ros()

        self.cart_traj = kwargs.get("cart_traj", [])  # Cartesian trajectory from teleop device
        self.traj_idx = 0
        self.robot_goal_right_frame = None
        self.task_debug_frames = np.empty((0, 4, 4))
        self.active_arm_goal = None

    # ------------------------------------------------------------------ common msg buffers
    def _init_ros_common_msgs(self):
        """
        Create message buffers shared by ROS1 and ROS2.
        Call this exactly once per instance after ROS is enabled.
        """
        from std_msgs.msg import Float64MultiArray, String

        # outbound messages
        self.Float64MultiArray = Float64MultiArray
        self.robot_command = Float64MultiArray()
        self.robot_state = Float64MultiArray()
        self.object_name_msg = String()
        self.object_position_msg = Float64MultiArray()
        self.robot_frame_msg = Float64MultiArray()
        self.robot_location_msg = Float64MultiArray()
        self.robot_hand_state_msg = Float64MultiArray()
        self.env_reset_msg = Float64MultiArray()
        self.robot_location_pub = None
        self.robot_hand_state_pub = None
        self.camera_image_pubs = {}
        self.camera_info_pubs = {}
        self.ImageMsg = None
        self.CameraInfoMsg = None

        # (optional) inbound caches if you want symmetry
        # self._last_teleop = Float64MultiArray()
        # self._last_gripper = Float64MultiArray()

    # ------------------------------------------------------------------ top-level init switch
    def _init_ros(self):
        if self.ros_version == "ros1":
            self._init_ros1()
        elif self.ros_version == "ros2":
            self._init_ros2()
        else:
            raise ValueError(f"Unsupported ROS version: {self.ros_version}")

    # ------------------------------------------------------------------ ROS1 init
    def _init_ros1(self):
        import rospy
        from std_msgs.msg import Float64MultiArray, String

        self.rospy = rospy

        # IMPORTANT: don't let rospy hijack SIGINT if your env manages signals
        self.rospy.init_node(self.task_name, anonymous=True, disable_signals=True)

        # shared message buffers
        self._init_ros_common_msgs()

        # ------------------------- publishers ------------------------- #
        self.robot_command_pub = self.rospy.Publisher(
            self.ros_args["robot_command_topic"], Float64MultiArray, queue_size=17
        )
        self.robot_state_pub = self.rospy.Publisher(
            self.ros_args["robot_state_topic"], Float64MultiArray, queue_size=17
        )
        self.object_name_pub = self.rospy.Publisher(
            self.ros_args["object_name_topic"], String, queue_size=17
        )
        self.object_position_pub = self.rospy.Publisher(
            self.ros_args["object_position_topic"], Float64MultiArray, queue_size=17
        )
        self.robot_frame_pub = self.rospy.Publisher(
            self.ros_args["robot_frame_topic"], Float64MultiArray, queue_size=17
        )
        if self.ros_args.get("robot_location_topic", None):
            self.robot_location_pub = self.rospy.Publisher(
                self.ros_args["robot_location_topic"], Float64MultiArray, queue_size=17
            )
        if self.ros_args.get("robot_hand_state_topic", None):
            self.robot_hand_state_pub = self.rospy.Publisher(
                self.ros_args["robot_hand_state_topic"], Float64MultiArray, queue_size=17
            )
        self.env_reset_pub = self.rospy.Publisher(
            self.ros_args["env_reset_topic"], Float64MultiArray, queue_size=17
        )
        self._init_ros1_camera_publishers()

        # ------------------------- subscribers ------------------------ #
        self.robot_teleop_sub = self.rospy.Subscriber(
            self.ros_args["robot_teleop_topic"], Float64MultiArray, self._teleop_goal_callback
        )
        self.obstacle_sub = self.rospy.Subscriber(
            self.ros_args["obstacle_topic"], Float64MultiArray, self._obstacle_state_callback
        )
        self.robot_teleop_joint_sub = self.rospy.Subscriber(
            self.ros_args["robot_teleop_joint_topic"],
            Float64MultiArray,
            self._teleop_goal_joint_callback,
        )
        self.robot_teleop_mode_sub = self.rospy.Subscriber(
            self.ros_args.get("robot_teleop_mode_topic", "robot_teleop_mode"),
            String,
            self._teleop_mode_callback,
        )
        self.debug_frame_sub = self.rospy.Subscriber(
            self.ros_args["debug_frames_topic"], Float64MultiArray, self._debug_frame_callback
        )
        self.robot_gripper_sub = self.rospy.Subscriber(
            self.ros_args["robot_gripper_topic"], Float64MultiArray, self._gripper_callback
        )
        self._attach_agent_ros1_real_robot_location(Float64MultiArray)

    # ------------------------------------------------------------------ ROS2 init
    def _init_ros2(self):
        # source /opt/ros/humble/setup.bash
        import threading
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import Float64MultiArray, String

        self.rclpy = rclpy

        # init rclpy once per process
        if not rclpy.ok():
            rclpy.init(args=None)

        # create node
        self.node = Node(self.task_name)

        # shared message buffers
        self._init_ros_common_msgs()

        # QoS: ROS1 queue_size ~= KEEP_LAST depth
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=17,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------------------- publishers ------------------------- #
        self.robot_command_pub = self.node.create_publisher(
            Float64MultiArray, self.ros_args["robot_command_topic"], qos
        )
        self.robot_state_pub = self.node.create_publisher(
            Float64MultiArray, self.ros_args["robot_state_topic"], qos
        )
        self.object_name_pub = self.node.create_publisher(
            String, self.ros_args["object_name_topic"], qos
        )
        self.object_position_pub = self.node.create_publisher(
            Float64MultiArray, self.ros_args["object_position_topic"], qos
        )
        self.robot_frame_pub = self.node.create_publisher(
            Float64MultiArray, self.ros_args["robot_frame_topic"], qos
        )
        if self.ros_args.get("robot_location_topic", None):
            self.robot_location_pub = self.node.create_publisher(
                Float64MultiArray, self.ros_args["robot_location_topic"], qos
            )
        if self.ros_args.get("robot_hand_state_topic", None):
            self.robot_hand_state_pub = self.node.create_publisher(
                Float64MultiArray, self.ros_args["robot_hand_state_topic"], qos
            )
        self.env_reset_pub = self.node.create_publisher(
            Float64MultiArray, self.ros_args["env_reset_topic"], qos
        )
        self._init_ros2_camera_publishers(qos)

        # ------------------------- subscribers ------------------------ #
        self.robot_teleop_sub = self.node.create_subscription(
            Float64MultiArray, self.ros_args["robot_teleop_topic"], self._teleop_goal_callback, qos
        )
        self.obstacle_sub = self.node.create_subscription(
            Float64MultiArray, self.ros_args["obstacle_topic"], self._obstacle_state_callback, qos
        )
        self.robot_teleop_joint_sub = self.node.create_subscription(
            Float64MultiArray,
            self.ros_args["robot_teleop_joint_topic"],
            self._teleop_goal_joint_callback,
            qos,
        )
        self.robot_teleop_mode_sub = self.node.create_subscription(
            String,
            self.ros_args.get("robot_teleop_mode_topic", "robot_teleop_mode"),
            self._teleop_mode_callback,
            qos,
        )
        self.debug_frame_sub = self.node.create_subscription(
            Float64MultiArray, self.ros_args["debug_frames_topic"], self._debug_frame_callback, qos
        )
        self.robot_gripper_sub = self.node.create_subscription(
            Float64MultiArray, self.ros_args["robot_gripper_topic"], self._gripper_callback, qos
        )
        self._attach_agent_ros2_real_robot_location(Float64MultiArray, qos)

        # ------------------- executor thread (callbacks) -------------- #
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._ros2_spin_running = True

        def _spin():
            while self._ros2_spin_running and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)

        self._ros2_spin_thread = threading.Thread(target=_spin, daemon=True)
        self._ros2_spin_thread.start()

    def _attach_agent_ros1_real_robot_location(self, msg_type):
        if not hasattr(self.agent, "attach_ros1_real_robot_location"):
            return

        topic = self.ros_args.get("real_robot_location_topic", None)
        if topic:
            self.agent.real_robot_location_topic = topic

        self.agent.attach_ros1_real_robot_location(self.rospy, msg_type)

    def _attach_agent_ros2_real_robot_location(self, msg_type, qos):
        if not hasattr(self.agent, "attach_ros2_real_robot_location"):
            return

        topic = self.ros_args.get("real_robot_location_topic", None)
        if topic:
            self.agent.real_robot_location_topic = topic

        self.agent.attach_ros2_real_robot_location(self.node, msg_type, qos)

    # Optional: call this when your env/task is being destroyed
    def close(self):
        if not getattr(self, "enable_ros", False):
            return

        if getattr(self, "ros_version", None) == "ros2":
            try:
                self._ros2_spin_running = False
            except Exception:
                pass
            try:
                self._executor.remove_node(self.node)
            except Exception:
                pass
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                if self.rclpy.ok():
                    self.rclpy.shutdown()
            except Exception:
                pass

    def _camera_topic_config(self):
        topics = self.ros_args.get("camera_topics", {})
        if topics:
            return topics

        if "camera_rgb_topic" not in self.ros_args and "camera_depth_topic" not in self.ros_args:
            return {}

        return {
            "head": {
                "rgb": self.ros_args.get("camera_rgb_topic", "g1/head_camera/color/image_raw"),
                "depth": self.ros_args.get("camera_depth_topic", "g1/head_camera/depth/image_raw"),
                "rgb_info": self.ros_args.get(
                    "camera_rgb_info_topic", "g1/head_camera/color/camera_info"
                ),
                "depth_info": self.ros_args.get(
                    "camera_depth_info_topic", "g1/head_camera/depth/camera_info"
                ),
            }
        }

    def _init_ros1_camera_publishers(self):
        topics = self._camera_topic_config()
        if not topics:
            return

        from sensor_msgs.msg import CameraInfo, Image

        self.ImageMsg = Image
        self.CameraInfoMsg = CameraInfo
        for camera_name, cfg in topics.items():
            self.camera_image_pubs[camera_name] = {}
            self.camera_info_pubs[camera_name] = {}
            if cfg.get("rgb"):
                self.camera_image_pubs[camera_name]["rgb"] = self.rospy.Publisher(
                    cfg["rgb"], Image, queue_size=3
                )
            if cfg.get("depth"):
                self.camera_image_pubs[camera_name]["depth"] = self.rospy.Publisher(
                    cfg["depth"], Image, queue_size=3
                )
            if cfg.get("rgb_info"):
                self.camera_info_pubs[camera_name]["rgb"] = self.rospy.Publisher(
                    cfg["rgb_info"], CameraInfo, queue_size=3
                )
            if cfg.get("depth_info"):
                self.camera_info_pubs[camera_name]["depth"] = self.rospy.Publisher(
                    cfg["depth_info"], CameraInfo, queue_size=3
                )

    def _init_ros2_camera_publishers(self, qos):
        topics = self._camera_topic_config()
        if not topics:
            return

        from sensor_msgs.msg import CameraInfo, Image

        self.ImageMsg = Image
        self.CameraInfoMsg = CameraInfo
        for camera_name, cfg in topics.items():
            self.camera_image_pubs[camera_name] = {}
            self.camera_info_pubs[camera_name] = {}
            if cfg.get("rgb"):
                self.camera_image_pubs[camera_name]["rgb"] = self.node.create_publisher(
                    Image, cfg["rgb"], qos
                )
            if cfg.get("depth"):
                self.camera_image_pubs[camera_name]["depth"] = self.node.create_publisher(
                    Image, cfg["depth"], qos
                )
            if cfg.get("rgb_info"):
                self.camera_info_pubs[camera_name]["rgb"] = self.node.create_publisher(
                    CameraInfo, cfg["rgb_info"], qos
                )
            if cfg.get("depth_info"):
                self.camera_info_pubs[camera_name]["depth"] = self.node.create_publisher(
                    CameraInfo, cfg["depth_info"], qos
                )

    def _init_obstacle(self):
        # ------------------------------- init obstacle ------------------------------ #
        self.obstacle_task = []
        self.obstacle_task_geom = []
        for _ in range(self.num_obstacle_task):
            obstacle = TaskObject3D(
                velocity=self.obstacle_velocity,
                keep_direction_step=self.obstacle_keep_direction_step,
                bound=self.obstacle_range,
                direction=self.obstacle_direction,
                smooth_weight=self.obstacle_smooth_weight,
                _seed=self._seed + len(self.obstacle_task),
                dt=self.dt,
            )
            if self.object_mesh_directory:
                obstacle.frame[:3, 3] = np.array(
                    [self.rs.uniform(low, high) for low, high in obstacle.bound]
                )
            elif self.mode == "Velocity":
                obstacle.frame[:3, 3] = self.obstacle_init
            else:
                obstacle.frame[:3, 3] = np.array(
                    [np.random.uniform(low, high) for low, high in obstacle.bound]
                )
            obstacle.frame_callback = obstacle.frame.copy()

            self.obstacle_task.append(obstacle)
            self.obstacle_task_geom.append(
                Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_task)
            )

        if self.environment_representation == "point_cloud" and self.object_mesh_directory:
            self._initialize_mesh_point_clouds()
        elif self.environment_representation == "point_cloud":
            index = np.arange(self.points_per_obstacle, dtype=float)
            z = 1.0 - 2.0 * (index + 0.5) / self.points_per_obstacle
            radius = np.sqrt(np.maximum(0.0, 1.0 - z * z))
            angle = index * np.pi * (3.0 - np.sqrt(5.0))
            self._environment_surface_points = self.obstacle_size * np.stack(
                (radius * np.cos(angle), radius * np.sin(angle), z), axis=1
            )
            self._environment_point_geometries = [
                Geometry(type="sphere", radius=0.004, color=VizColor.obstacle_task)
                for _ in range(self.num_obstacle_task * self.points_per_obstacle)
            ]
        elif self.environment_representation != "sphere":
            raise ValueError("TeleopTask supports only sphere and point_cloud environments")

        return

    def _initialize_mesh_point_clouds(self):
        """Sample collision points from distinct WBCD-style object meshes."""
        import trimesh

        directory = Path(self.object_mesh_directory).expanduser().resolve()
        mesh_paths = sorted(
            path for path in directory.iterdir() if path.suffix.lower() in {".stl", ".obj", ".ply"}
        )
        if not mesh_paths:
            raise ValueError(f"No supported object meshes found in {directory}")

        order = self.rs.permutation(len(mesh_paths))
        point_batches = []
        self._environment_mesh_names = []
        for obstacle_index in range(self.num_obstacle_task):
            path = mesh_paths[int(order[obstacle_index % len(order)])]
            mesh = trimesh.load(str(path), force="mesh", process=True)
            vertices = np.asarray(mesh.vertices, dtype=float)
            faces = np.asarray(mesh.faces, dtype=int)
            if not len(vertices) or not len(faces):
                raise ValueError(f"Object mesh has no triangles: {path}")

            center = 0.5 * (vertices.min(axis=0) + vertices.max(axis=0))
            vertices = vertices - center
            extent = float(np.max(np.ptp(vertices, axis=0)))
            if extent <= 1.0e-12:
                raise ValueError(f"Object mesh has zero extent: {path}")
            target_extent = 2.0 * float(self.obstacle_size) * self.object_mesh_scale
            vertices *= target_extent / extent
            triangles = vertices[faces]
            areas = 0.5 * np.linalg.norm(
                np.cross(
                    triangles[:, 1] - triangles[:, 0],
                    triangles[:, 2] - triangles[:, 0],
                ),
                axis=1,
            )
            valid_faces = areas > 1.0e-12
            triangles = triangles[valid_faces]
            areas = areas[valid_faces]
            if not len(triangles):
                raise ValueError(f"Object mesh has no nondegenerate faces: {path}")
            face_ids = self.rs.choice(
                len(triangles), self.points_per_obstacle, p=areas / areas.sum()
            )
            uv = self.rs.random((self.points_per_obstacle, 2))
            reflected = uv.sum(axis=1) > 1.0
            uv[reflected] = 1.0 - uv[reflected]
            selected = triangles[face_ids]
            points = (
                selected[:, 0]
                + uv[:, :1] * (selected[:, 1] - selected[:, 0])
                + uv[:, 1:] * (selected[:, 2] - selected[:, 0])
            )

            point_batches.append(points)
            self._environment_mesh_names.append(path.name)

        self._environment_surface_points = point_batches
        self._environment_point_geometries = [
            Geometry(type="sphere", radius=0.004, color=VizColor.obstacle_task)
            for _ in range(self.num_obstacle_task * self.points_per_obstacle)
        ]

    def _init_goal(self):
        # --------------------------------- init goal -------------------------------- #
        if self.arm_goal_enable:
            self.robot_goal_right = TaskObject3D(
                velocity=self.goal_right_velocity,
                direction=self.goal_right_direction,
                keep_direction_step=self.arm_goal_keep_direction_step,
                bound=self.right_arm_goal_range,
                smooth_weight=self.arm_goal_smooth_weight,
                _seed=self._seed,
                dt=self.dt,
            )
            if self.arm_goal_init_from_current_ee and self.robot_goal_right_home_frame is not None:
                self.robot_goal_right.frame = self.robot_goal_right_home_frame.copy()
            elif self.mode == "Velocity":
                self.robot_goal_right.frame[:3, 3] = self.goal_right_init
            else:
                self.robot_goal_right.frame[:3, 3] = np.array(
                    [self.rs.uniform(low, high) for low, high in self.robot_goal_right.bound]
                )
            self.robot_goal_right.frame_callback = self.robot_goal_right.frame.copy()

            if self.use_dual_arm:
                self.robot_goal_left = TaskObject3D(
                    velocity=self.goal_left_velocity,
                    direction=self.goal_left_direction,
                    keep_direction_step=self.arm_goal_keep_direction_step,
                    bound=self.left_arm_goal_range,
                    smooth_weight=self.arm_goal_smooth_weight,
                    _seed=self._seed,
                    dt=self.dt,
                )
                if (
                    self.arm_goal_init_from_current_ee
                    and self.robot_goal_left_home_frame is not None
                ):
                    self.robot_goal_left.frame = self.robot_goal_left_home_frame.copy()
                elif self.mode == "Velocity":
                    self.robot_goal_left.frame[:3, 3] = self.goal_left_init
                else:
                    self.robot_goal_left.frame[:3, 3] = np.array(
                        [self.rs.uniform(low, high) for low, high in self.robot_goal_left.bound]
                    )
                self.robot_goal_left.frame_callback = self.robot_goal_left.frame.copy()

        if self.base_goal_enable:
            self.robot_goal_base = TaskObject3D(
                velocity=self.base_goal_velocity,
                direction=self.base_goal_direction,
                keep_direction_step=self.base_goal_keep_direction_step,
                bound=self.base_goal_range,
                smooth_weight=self.base_goal_smooth_weight,
                _seed=self._seed,
                dt=self.dt,
            )

            if self.mode == "Velocity":
                self.robot_goal_base.frame[:3, 3] = self.base_goal_init
            else:
                self.robot_goal_base.frame[:3, 3] = np.array(
                    [self.rs.uniform(low, high) for low, high in self.robot_goal_base.bound]
                )

            self.robot_goal_base.frame_callback = self.robot_goal_base.frame.copy()

        return

    def _compose_keyboard_offset_goal(self, home_frame, offset_frame):
        home_frame = np.asarray(home_frame, dtype=float).reshape(4, 4)
        offset_frame = np.asarray(offset_frame, dtype=float).reshape(4, 4)
        goal_frame = home_frame.copy()
        goal_frame[:3, :3] = home_frame[:3, :3] @ offset_frame[:3, :3]
        goal_frame[:3, 3] = home_frame[:3, 3] + offset_frame[:3, 3]
        return goal_frame

    def _update_goal(self):
        use_keyboard_goal = (not self.enable_ros) or bool(
            getattr(self.agent, "enable_keyboard_control", False)
        )
        if use_keyboard_goal:
            selected_goal_side = getattr(self.agent, "_selected_goal_debug_side", None)
            self.active_arm_goal = selected_goal_side() if callable(selected_goal_side) else None
            if self.arm_goal_enable:
                if (
                    self.arm_goal_init_from_current_ee
                    and self.robot_goal_right_home_frame is not None
                ):
                    robot_goal_right = self.robot_goal_right_home_frame.copy()
                else:
                    robot_goal_right = np.eye(4)
                    robot_goal_right[:3, 3] = self.goal_right_init.copy()
                self.robot_goal_right.frame = self._compose_keyboard_offset_goal(
                    robot_goal_right,
                    self.agent_feedback["robot_goal_right_offset"],
                )
                if self.use_dual_arm:
                    if (
                        self.arm_goal_init_from_current_ee
                        and self.robot_goal_left_home_frame is not None
                    ):
                        robot_goal_left = self.robot_goal_left_home_frame.copy()
                    else:
                        robot_goal_left = np.eye(4)
                        robot_goal_left[:3, 3] = self.goal_left_init.copy()
                    self.robot_goal_left.frame = self._compose_keyboard_offset_goal(
                        robot_goal_left,
                        self.agent_feedback["robot_goal_left_offset"],
                    )
            if self.base_goal_enable:
                robot_goal_base = np.eye(4)
                robot_goal_base[:3, 3] = self.base_goal_init.copy()
                self.robot_goal_base.frame = (
                    robot_goal_base @ self.agent_feedback["robot_goal_base_offset"]
                )
            if "left_gripper_debug_state" in self.agent_feedback:
                self.left_gripper_goal = self.agent_feedback["left_gripper_debug_state"]
            if "right_gripper_debug_state" in self.agent_feedback:
                self.right_gripper_goal = self.agent_feedback["right_gripper_debug_state"]
        else:
            self.active_arm_goal = None
            if self.arm_goal_enable:
                self.robot_goal_right.frame = self.robot_goal_right.frame_callback
                if self.use_dual_arm:
                    self.robot_goal_left.frame = self.robot_goal_left.frame_callback
            if self.base_goal_enable:
                self.robot_goal_base.frame = self.robot_goal_base.frame_callback
        return

    def _update_obstacle(self):
        if self.enable_ros == False:
            for obstacle in self.obstacle_task:
                obstacle.move(self.mode)
        else:
            for i in range(self.num_obstacle_task):
                # update the obstacle position based on the ros callback
                self.obstacle_task[i].frame[:3, 3] = self.obstacle_task[i].frame_callback[:3, 3]

    def _update_done(self):
        self._done = False

        if self.enable_ros and self.rospy.is_shutdown():
            self._done = True

    def step(self, feedback):
        super().step(feedback)
        self._pub_object_pos()
        self._pub_robot_frame()
        self._pub_robot_state()
        self._pub_robot_location()
        self._pub_robot_hand_state()
        self._pub_camera_images()
        self._pub_env_reset()

    def _upper_body_slice_for_dof_pos(self, dof_pos: np.ndarray):
        n = np.asarray(dof_pos).reshape(-1).shape[0]
        if n >= 36:
            return slice(19, 36)
        if n >= 29:
            return slice(12, 29)
        if n >= 17:
            return slice(n - 17, n)
        return None

    def _joint_goal_visual_dof_pos(self):
        target = getattr(self, "upper_body_joint_target_callback", None)
        if target is None:
            return None

        try:
            dof_pos = self.robot_cfg.decompose_state_to_dof_pos(self.agent_feedback["state"])
        except Exception:
            dof_pos = self.agent_feedback.get(
                "body_qpos_fbk",
                self.agent_feedback.get("dof_pos_fbk", None),
            )
        if dof_pos is None:
            return None

        dof_pos = np.asarray(dof_pos, dtype=float).reshape(-1).copy()
        upper_slice = self._upper_body_slice_for_dof_pos(dof_pos)
        if upper_slice is None or upper_slice.stop - upper_slice.start != 17:
            return None

        target = np.asarray(target, dtype=float).reshape(-1)
        if target.shape[0] != 17:
            return None

        mask = getattr(self, "upper_body_joint_target_mask_callback", None)
        if mask is None:
            mask = np.ones(17, dtype=bool)
        else:
            mask = np.asarray(mask, dtype=bool).reshape(-1)
            if mask.shape[0] != 17:
                return None

        upper = dof_pos[upper_slice].copy()
        upper[mask] = target[mask]
        dof_pos[upper_slice] = upper
        return dof_pos

    def _update_joint_goal_visualization(self) -> None:
        if str(getattr(self, "teleop_upper_body_mode", "cartesian")).lower() != "joint":
            return

        dof_pos = self._joint_goal_visual_dof_pos()
        if dof_pos is None:
            return

        try:
            robot_frames = self.robot_kinematics.forward_kinematics(dof_pos)
        except Exception as exc:
            if not getattr(self, "_warned_joint_goal_visual_fk", False):
                print(
                    f"[TeleopTask] Could not compute joint-mode arm goal FK for visualization: {exc}"
                )
                self._warned_joint_goal_visual_fk = True
            return

        if "R_ee" in self.robot_cfg.Frames.__members__:
            self.info["goal_teleop"]["right"] = (
                self.robot_base_frame @ robot_frames[self.robot_cfg.Frames.R_ee]
            ).reshape(1, 4, 4)
        if self.use_dual_arm and "L_ee" in self.robot_cfg.Frames.__members__:
            self.info["goal_teleop"]["left"] = (
                self.robot_base_frame @ robot_frames[self.robot_cfg.Frames.L_ee]
            ).reshape(1, 4, 4)

    def get_info(self) -> dict:
        super().get_info()
        # Keyboard teleoperation selects one Cartesian target at a time.  The
        # policy may use this hint to keep an inactive manipulator stationary;
        # autonomous/ROS dual-goal tasks intentionally leave it unset.
        self.info["active_arm_goal"] = self.active_arm_goal
        if (
            self.environment_representation == "point_cloud"
            and self._environment_surface_points is not None
        ):
            frame_batches = []
            velocity_batches = []
            for obstacle_index, obstacle in enumerate(self.obstacle_task):
                local_points = (
                    self._environment_surface_points[obstacle_index]
                    if isinstance(self._environment_surface_points, list)
                    else self._environment_surface_points
                )
                frames = np.repeat(np.eye(4)[None], self.points_per_obstacle, axis=0)
                frames[:, :3, 3] = (obstacle.frame[:3, :3] @ local_points.T).T + obstacle.frame[
                    :3, 3
                ]
                velocity = obstacle.velocity * np.concatenate((obstacle.direction, np.zeros(3)))
                frame_batches.append(frames)
                velocity_batches.append(np.repeat(velocity[None], self.points_per_obstacle, axis=0))
            point_frames = (
                np.concatenate(frame_batches, axis=0) if frame_batches else np.empty((0, 4, 4))
            )
            point_velocities = (
                np.concatenate(velocity_batches, axis=0) if velocity_batches else np.empty((0, 6))
            )
            point_geometries = self._environment_point_geometries[: len(point_frames)]
            debug_frames = self.info["obstacle_debug"]["frames_world"]
            debug_velocities = self.info["obstacle_debug"]["velocity"]
            debug_geometries = self.info["obstacle_debug"]["geom"]
            self.info["obstacle_task"].update(
                frames_world=point_frames,
                velocity=point_velocities,
                geom=point_geometries,
            )
            self.info["obstacle"].update(
                frames_world=np.concatenate((point_frames, debug_frames), axis=0),
                velocity=np.concatenate((point_velocities, debug_velocities), axis=0),
                geom=np.concatenate((point_geometries, debug_geometries), axis=0),
                num=len(point_frames) + len(debug_frames),
            )
            self.info["environment_representation"] = "point_cloud"
            self.info["max_visualized_points"] = self.max_visualized_points
            self.info["environment_mesh_names"] = list(self._environment_mesh_names)
        self._update_joint_goal_visualization()

        self.info["task_debug_frames"] = self.task_debug_frames
        return self.info

    # ---------------------------------- ROS helpers --------------------------------- #
    def _teleop_mode_callback(self, data):
        mode = str(data.data).strip().lower()
        aliases = {
            "cart": "cartesian",
            "ee": "cartesian",
            "ik": "cartesian",
            "pos": "joint",
            "position": "joint",
            "joint_pos": "joint",
            "joint_position": "joint",
        }
        mode = aliases.get(mode, mode)
        if mode not in ("cartesian", "joint", "auto"):
            print(
                "[TeleopTask] Ignoring robot_teleop_mode="
                f"{data.data!r}; expected cartesian, joint, or auto."
            )
            return
        self.teleop_upper_body_mode = mode

    def _gripper_callback(self, data):
        values = np.asarray(data.data, dtype=float).reshape(-1)
        if values.size < 1:
            return
        if not np.isnan(values[0]):
            self.right_gripper_goal = bool(values[0])
        if self.use_dual_arm:
            if values.size < 2:
                return
            if not np.isnan(values[1]):
                self.left_gripper_goal = bool(values[1])

        if self.gripper_debug_print:
            now = (
                self.rospy.Time.now().to_sec()
                if self.ros_version == "ros1"
                else self.node.get_clock().now().nanoseconds * 1.0e-9
            )
            if now - self._last_gripper_debug_time > 0.5:
                self._last_gripper_debug_time = now
                print(
                    "[TeleopTask gripper] "
                    f"right={self.right_gripper_goal} left={self.left_gripper_goal} raw={values.tolist()}"
                )

    def _teleop_goal_callback(self, data):
        values = np.asarray(data.data, dtype=float).reshape(-1)
        if self.arm_goal_enable:
            if values.size < 16:
                if not getattr(self, "_warned_short_teleop_goal", False):
                    print(
                        f"[TeleopTask] Ignoring robot_teleop with {values.size} floats; right arm goal needs at least 16."
                    )
                    self._warned_short_teleop_goal = True
                return
            self.robot_goal_right.frame_callback = values[:16].reshape(1, 4, 4)
            if self.use_dual_arm:
                if values.size < 32:
                    if not getattr(self, "_warned_short_teleop_goal", False):
                        print(
                            f"[TeleopTask] Ignoring left arm robot_teleop goal with {values.size} floats; dual-arm goal needs at least 32."
                        )
                        self._warned_short_teleop_goal = True
                    return
                self.robot_goal_left.frame_callback = values[16:32].reshape(1, 4, 4)
        if self.base_goal_enable:
            if values.size < 48:
                if not getattr(self, "_warned_short_base_goal", False):
                    print(
                        f"[TeleopTask] robot_teleop has {values.size} floats; base goal needs 48. Keeping previous base goal."
                    )
                    self._warned_short_base_goal = True
                return
            self.robot_goal_base.frame_callback = values[32:48].reshape(1, 4, 4)

    def _obstacle_state_callback(self, data):
        for i in range(self.num_obstacle_task):
            self.obstacle_task[i].frame_callback = self.info[
                "robot_base_frame"
            ] @ pos_quat_to_transformation(
                np.array(data.data[i * 7 + 1 : i * 7 + 4]), np.array([0.0, 0.0, 0.0, 1.0])
            )

    def _cart_traj_callback(self, msg):
        # msg.data is array.array('d')
        flat = np.asarray(msg.data, dtype=np.float64)  # (N*7,)

        if flat.size % 7 != 0:
            print(f"[cart_traj_callback] Bad length: {flat.size} (not divisible by 7)")
            return

        traj_len = flat.size // 7
        if traj_len == 0:
            return

        self.cart_traj = flat.reshape(traj_len, 7)  # (N,7)

    def _teleop_goal_joint_callback(self, data):
        values = np.asarray(data.data, dtype=float).reshape(-1)

        if values.size == 17:
            mask = np.isfinite(values)
            target = np.nan_to_num(values, nan=0.0)
            self.upper_body_joint_target_callback = target.copy()
            self.upper_body_joint_target_mask_callback = mask.copy()
            if hasattr(self, "info"):
                self.info["teleop_joint_upper_body_target"] = target.copy()
                self.info["teleop_joint_upper_body_target_mask"] = mask.copy()
            return

        if values.size == 29:
            target = values[12:29].copy()
            mask = np.ones(17, dtype=bool)
            self.upper_body_joint_target_callback = target.copy()
            self.upper_body_joint_target_mask_callback = mask
            if hasattr(self, "info"):
                self.info["teleop_joint_upper_body_target"] = target.copy()
                self.info["teleop_joint_upper_body_target_mask"] = mask.copy()
            return

        if values.size == 36:
            mask = np.ones(17, dtype=bool)
            target = values[19:36].copy()
            self.upper_body_joint_target_callback = target.copy()
            self.upper_body_joint_target_mask_callback = mask
            if hasattr(self, "info"):
                self.info["teleop_joint_upper_body_target"] = target.copy()
                self.info["teleop_joint_upper_body_target_mask"] = mask.copy()
            return

        print(
            "[TeleopTask] Ignoring robot_teleop_joint with "
            f"{values.size} floats; expected 17 upper-body, 29 body, or 36 full-qpos values."
        )

    def _debug_frame_callback(self, data):
        self.task_debug_frames = np.array(data.data).reshape(-1, 4, 4)

    def _pub_robot_command(self, command):
        if self.enable_ros:
            self.robot_command.data = command
            self.robot_command_pub.publish(self.robot_command)

    def _extract_g1_body_29_state(self):
        body_qpos = self.agent_feedback.get("body_qpos_fbk", None)
        if body_qpos is not None:
            body_qpos = np.asarray(body_qpos, dtype=float).reshape(-1)
            if body_qpos.size >= 36:
                return body_qpos[7:36]
            if body_qpos.size == 29:
                return body_qpos

        dof_pos = np.asarray(self.agent_feedback["dof_pos_fbk"], dtype=float).reshape(-1)
        if dof_pos.size == 36:
            return dof_pos[7:36]
        if dof_pos.size == 29:
            return dof_pos
        raise ValueError(
            "Cannot publish G1 body-29 robot_state without body_qpos_fbk; "
            f"got dof_pos_fbk length {dof_pos.size}."
        )

    def _robot_state_for_publish(self):
        mode = self.ros_args.get("robot_state_publish_mode", "raw_dof_pos")
        if mode == "raw_dof_pos":
            return np.asarray(self.agent_feedback["dof_pos_fbk"], dtype=float).reshape(-1)
        if mode == "g1_body_29":
            return self._extract_g1_body_29_state()
        raise ValueError(f"Unsupported robot_state_publish_mode: {mode}")

    def _extract_g1_hand_14_state(self):
        qpos = self.agent_feedback.get(
            "mujoco_qpos_fbk",
            self.agent_feedback.get("qpos_fbk", self.agent_feedback.get("dof_pos_fbk", None)),
        )
        if qpos is None:
            return None

        qpos = np.asarray(qpos, dtype=float).reshape(-1)
        left_idx = getattr(self.agent, "left_hand_qpos_index", None)
        right_idx = getattr(self.agent, "right_hand_qpos_index", None)
        if left_idx is not None and right_idx is not None:
            left_idx = np.asarray(left_idx, dtype=int)
            right_idx = np.asarray(right_idx, dtype=int)
            hand_idx = np.concatenate((left_idx, right_idx))
            if hand_idx.size == 14 and np.max(hand_idx) < qpos.size:
                return np.concatenate(
                    (
                        qpos[left_idx],
                        qpos[right_idx],
                    )
                )
            return None

        hand_names = (
            "LeftHandThumb0",
            "LeftHandThumb1",
            "LeftHandThumb2",
            "LeftHandIndex0",
            "LeftHandIndex1",
            "LeftHandMiddle0",
            "LeftHandMiddle1",
            "RightHandThumb0",
            "RightHandThumb1",
            "RightHandThumb2",
            "RightHandIndex0",
            "RightHandIndex1",
            "RightHandMiddle0",
            "RightHandMiddle1",
        )
        if all(hasattr(self.robot_cfg.DoFs, name) for name in hand_names):
            hand_idx = np.asarray(
                [int(getattr(self.robot_cfg.DoFs, name)) for name in hand_names],
                dtype=int,
            )
            if hand_idx.size == len(hand_names) and np.max(hand_idx) < qpos.size:
                return qpos[hand_idx].astype(float)
            return None

        if qpos.size == len(hand_names):
            return qpos
        return None

    def _pub_robot_state(self, action_info=None):
        if self.enable_ros:
            self.robot_state.data = self._robot_state_for_publish().astype(float).tolist()
            self.robot_state_pub.publish(self.robot_state)

    def _pub_robot_location(self):
        if not (self.enable_ros and self.robot_location_pub is not None):
            return

        robot_base_frame = self.agent_feedback.get("robot_base_frame", None)
        if robot_base_frame is None:
            return

        self.robot_location_msg.data = (
            np.asarray(robot_base_frame, dtype=float).reshape(4, 4).reshape(-1).tolist()
        )
        self.robot_location_pub.publish(self.robot_location_msg)

    def _pub_robot_hand_state(self):
        if not (self.enable_ros and self.robot_hand_state_pub is not None):
            return

        hand_state = self._extract_g1_hand_14_state()
        if hand_state is None:
            return

        self.robot_hand_state_msg.data = np.asarray(hand_state, dtype=float).reshape(-1).tolist()
        self.robot_hand_state_pub.publish(self.robot_hand_state_msg)

    def _pub_robot_frame(self):
        if self.enable_ros:
            robot_base_frame = self.agent_feedback.get("robot_base_frame", None)
            kinematics_dof_pos = self.agent_feedback.get(
                "body_qpos_fbk",
                self.agent_feedback["dof_pos_fbk"],
            )
            robot_frames = self.robot_kinematics.forward_kinematics(kinematics_dof_pos)
            left_hand_frame = (
                robot_frames[self.robot_cfg.Frames.L_ee]
                if "L_ee" in self.robot_cfg.Frames.__members__
                else None
            )
            right_hand_frame = (
                robot_frames[self.robot_cfg.Frames.R_ee]
                if "R_ee" in self.robot_cfg.Frames.__members__
                else None
            )

            self.robot_frame_msg.data = (
                np.concatenate(
                    (
                        robot_base_frame.reshape(-1),
                        (
                            right_hand_frame.reshape(-1)
                            if right_hand_frame is not None
                            else np.zeros(16)
                        ),
                        (
                            left_hand_frame.reshape(-1)
                            if left_hand_frame is not None
                            else np.zeros(16)
                        ),
                    )
                )
                .astype(float)
                .tolist()
            )

            self.robot_frame_pub.publish(self.robot_frame_msg)

    def _ros_stamp(self, stamp=None):
        if self.ros_version == "ros1":
            if stamp is not None:
                try:
                    return self.rospy.Time.from_sec(float(stamp))
                except Exception:
                    pass
            return self.rospy.Time.now()
        return self.node.get_clock().now().to_msg()

    def _make_image_msg(self, image, encoding, frame_id, stamp):
        msg = self.ImageMsg()
        array = np.asarray(image)
        if encoding == "rgb8":
            array = np.ascontiguousarray(array.astype(np.uint8, copy=False))
            if array.ndim != 3 or array.shape[2] != 3:
                raise ValueError(f"rgb8 camera image must be HxWx3, got shape {array.shape}")
            step = int(array.shape[1] * 3)
        elif encoding == "32FC1":
            array = np.ascontiguousarray(array.astype(np.float32, copy=False))
            if array.ndim != 2:
                raise ValueError(f"32FC1 depth image must be HxW, got shape {array.shape}")
            step = int(array.shape[1] * 4)
        else:
            raise ValueError(f"Unsupported camera encoding: {encoding}")

        msg.header.stamp = self._ros_stamp(stamp)
        msg.header.frame_id = str(frame_id)
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        payload = array.tobytes()
        try:
            msg.data = payload
        except TypeError:
            msg.data = list(payload)
        return msg

    def _make_camera_info_msg(self, camera_info, header):
        msg = self.CameraInfoMsg()
        msg.header = header
        msg.height = int(camera_info.get("height", 0))
        msg.width = int(camera_info.get("width", 0))
        msg.distortion_model = str(camera_info.get("distortion_model", "plumb_bob"))

        arrays = {
            "D": list(camera_info.get("D", [])),
            "K": list(camera_info.get("K", [0.0] * 9)),
            "R": list(camera_info.get("R", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])),
            "P": list(camera_info.get("P", [0.0] * 12)),
        }
        for upper, lower in (("D", "d"), ("K", "k"), ("R", "r"), ("P", "p")):
            values = [float(item) for item in arrays[upper]]
            if hasattr(msg, upper):
                setattr(msg, upper, values)
            if hasattr(msg, lower):
                setattr(msg, lower, values)
        return msg

    def _pub_camera_images(self):
        if not (self.enable_ros and self.camera_image_pubs):
            return

        camera_feedback = self.agent_feedback.get("camera", {})
        if not camera_feedback:
            return

        for camera_name, data in camera_feedback.items():
            image_pubs = self.camera_image_pubs.get(camera_name, {})
            info_pubs = self.camera_info_pubs.get(camera_name, {})
            if not image_pubs and not info_pubs:
                continue

            frame_id = data.get("frame_id", f"{camera_name}_optical_frame")
            stamp = data.get("stamp", None)
            camera_info = data.get("camera_info", {})

            if "rgb" in image_pubs and data.get("rgb", None) is not None:
                msg = self._make_image_msg(data["rgb"], "rgb8", frame_id, stamp)
                image_pubs["rgb"].publish(msg)
                if "rgb" in info_pubs:
                    info_pubs["rgb"].publish(self._make_camera_info_msg(camera_info, msg.header))

            if "depth" in image_pubs and data.get("depth", None) is not None:
                msg = self._make_image_msg(data["depth"], "32FC1", frame_id, stamp)
                image_pubs["depth"].publish(msg)
                if "depth" in info_pubs:
                    info_pubs["depth"].publish(self._make_camera_info_msg(camera_info, msg.header))

    def _pub_object_pos(self):
        if self.enable_ros:
            object_pos = self.agent_feedback.get("object_pos", None)
            if not object_pos:
                return
            object_names = sorted(object_pos.keys())
            transform_list = []
            for name in object_names:
                obj_pose = object_pos[name]
                obj_quat = np.array([obj_pose[4], obj_pose[5], obj_pose[6], obj_pose[3]])
                obj_T = pos_quat_to_transformation(obj_pose[:3], obj_quat)
                transform_list.append(obj_T.reshape(-1))
            if not transform_list:
                return
            if self.ros_version == "ros1":
                stamp = self.rospy.Time.now().to_sec()
            elif self.ros_version == "ros2":
                stamp = (
                    self.node.get_clock().now().to_msg().sec
                    + self.node.get_clock().now().to_msg().nanosec * 1e-9
                )
            self.object_name_msg.data = json.dumps({"stamp": stamp, "names": object_names})
            self.object_position_msg.data = np.concatenate(transform_list).astype(float).tolist()
            self.object_name_pub.publish(self.object_name_msg)
            self.object_position_pub.publish(self.object_position_msg)

    def _pub_env_reset(self):
        if self.enable_ros:
            self.env_reset_msg.data = [float(self._done)]
            self.env_reset_pub.publish(self.env_reset_msg)


if __name__ == "__main__":
    pass
