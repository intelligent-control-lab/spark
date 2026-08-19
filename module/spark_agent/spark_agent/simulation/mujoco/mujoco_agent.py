import numpy as np
import mujoco
import mujoco.viewer
from mujoco.glfw import glfw
from spark_agent.simulation.simulation_agent import SimulationAgent
from spark_agent.simulation.viewer_config import (
    normalize_sensor_camera_config,
    normalize_viewer_config,
)
from spark_agent.dynamics import DynamicsExecutor
from spark_robot import RobotConfig
from spark_utils import Geometry, VizColor
from scipy.spatial.transform import Rotation as R
from spark_robot import RobotConfig, SPARK_ROBOT_RESOURCE_DIR
import os
from fnmatch import fnmatchcase
import time
import cv2
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, Optional, Sequence, Set, Tuple, Dict, Any, List, Mapping


class MujocoAgent(SimulationAgent):
    """
    A class for controlling a Mujoco-based simulation agent.
    """

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        """
        Initializes the MuJoCo agent with robot configuration and simulation parameters.

        Args:
            robot_cfg (RobotConfig): Robot configuration.
            **kwargs: Simulation/viewer/camera/debug options.
        """
        super().__init__(robot_cfg)

        # ----------------------------- defaults / debug state ----------------------------- #
        self.left_goal_debug_frame = np.eye(4)
        self.right_goal_debug_frame = np.eye(4)
        self.base_goal_debug_frame = np.eye(4)

        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self._keyboard_gripper_goal_override = {}

        self.translate_in_local_frame = False
        self.debug_object = None

        self.obstacle_debug_frame = np.empty((0, 4, 4))
        self.obstacle_debug_geom = []
        self.obstacle_debug_velocity = []

        self.obstacle_debug_selected = 0
        self.num_obstacle_debug_change_buf = 0
        self.obstacle_debug_frame_last = np.empty((0, 4, 4))

        self.left_hand_picked_object = None
        self.right_hand_picked_object = None

        self.render_robot_collision_volumes = bool(
            kwargs.get("render_robot_collision_volumes", True)
        )

        # ----------------------------- sim config (kwargs) ----------------------------- #
        simulator_spec = self.robot_cfg.simulator_dynamics
        dynamics_backend = kwargs.get("dynamics_backend")
        if dynamics_backend is None:
            dynamics_backend = "simulator" if kwargs.get("use_sim_dynamics", True) else "model"
        if dynamics_backend not in ("simulator", "model", "projected"):
            raise ValueError('dynamics_backend must be "simulator", "model", or "projected".')
        self.dynamics_backend = dynamics_backend
        # Backward-compatible alias. New configurations should use dynamics_backend.
        self.use_sim_dynamics = dynamics_backend in ("simulator", "projected")
        self.dt = float(kwargs.get("dt", simulator_spec.physics_dt))
        self.control_decimation = int(
            kwargs.get("control_decimation", simulator_spec.control_decimation)
        )
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.control_decimation < 1:
            raise ValueError("control_decimation must be positive")
        self.model_integrator = kwargs.get("model_integrator")
        self.model_substeps = int(kwargs.get("model_substeps", 1))
        self.real_time = bool(kwargs.get("real_time", True))
        self.enable_keyboard_control = bool(kwargs.get("enable_keyboard_control", True))
        self.enable_viewer = bool(kwargs.get("enable_viewer", True))
        self.viewer_show_left_ui = bool(kwargs.get("viewer_show_left_ui", False))
        self.viewer_show_right_ui = bool(kwargs.get("viewer_show_right_ui", False))
        self.viewer_show_simulation_info = bool(kwargs.get("viewer_show_simulation_info", False))
        self._viewer_simulation_info = {}
        self.viewer_config = normalize_viewer_config(kwargs.get("viewer_config"))
        self.record_video_path = kwargs.get("record_video_path")
        self.record_gif_path = kwargs.get("record_gif_path")
        self.record_fps = float(kwargs.get("record_fps", 30.0))
        self.record_duration = float(kwargs.get("record_duration", 0.0))
        self.record_width = int(
            kwargs.get("record_width", self.width if hasattr(self, "width") else 1280)
        )
        self.record_height = int(
            kwargs.get("record_height", self.height if hasattr(self, "height") else 720)
        )
        self.recording_enabled = bool(self.record_video_path or self.record_gif_path)
        if self.recording_enabled and self.record_fps <= 0.0:
            raise ValueError("record_fps must be positive")
        self.enable_camera = bool(kwargs.get("enable_camera", False))
        self.stabilize_sim_dynamics_joint_positions = bool(
            kwargs.get(
                "stabilize_sim_dynamics_joint_positions",
                dynamics_backend == "projected",
            )
        )
        self.sim_position_error_limit = kwargs.get("sim_position_error_limit", 0.35)
        self.sim_ctrl_abs_limit = kwargs.get("sim_ctrl_abs_limit", 1000.0)
        self.uncontrolled_actuator_hold_kp = float(
            kwargs.get("uncontrolled_actuator_hold_kp", 300.0)
        )
        self.uncontrolled_actuator_hold_kd = float(
            kwargs.get("uncontrolled_actuator_hold_kd", 12.0)
        )
        self.uncontrolled_joint_stiffness = float(
            kwargs.get("uncontrolled_joint_stiffness", 2000.0)
        )
        self.uncontrolled_joint_damping = float(kwargs.get("uncontrolled_joint_damping", 80.0))
        self.uncontrolled_joint_armature = float(kwargs.get("uncontrolled_joint_armature", 0.05))
        self._uncontrolled_actuator_holds = []

        self.width = int(kwargs.get("width", 640))
        self.height = int(kwargs.get("height", 480))
        self.camera_width = int(kwargs.get("camera_width", self.width))
        self.camera_height = int(kwargs.get("camera_height", self.height))
        self.camera_rate_hz = float(kwargs.get("camera_rate_hz", 30.0))
        self.camera_display = bool(kwargs.get("camera_display", False))
        self.camera_config = self._normalize_camera_config(kwargs.get("camera_config", None))
        self._camera_capture_period = (
            0.0 if self.camera_rate_hz <= 0.0 else 1.0 / self.camera_rate_hz
        )
        self._last_camera_capture_time = -np.inf
        self.camera_feedback = {}
        self._viewer_free_camera_state = None
        self._viewer_visual_state = None
        self._restore_viewer_state_after_spark_key = False
        # Pipeline visualization is appended to the viewer/recording scene
        # after the simulation step, whereas robot-mounted cameras are
        # captured on the following step.  Retain one completed frame of
        # simple solids so virtual perception obstacles (notably sampled
        # point clouds) are visible in both RGB and depth camera output.
        self._camera_overlay_geoms = []
        self.counter = 0

        # ----------------------------- model / data ----------------------------- #
        self.class_name = robot_cfg.__class__.__name__

        self.mujoco_model_path = kwargs.get("mujoco_model_path") or self.robot_cfg.mujoco_model_path
        self.mujoco_xml_path = self._resolve_mujoco_model_path(self.mujoco_model_path)
        self.model = self._load_mujoco_model()
        self._apply_configured_visual_palette()
        self._configure_camera_offscreen_buffer()
        self.data = mujoco.MjData(self.model)

        self.model.opt.timestep = self.dt
        self.target_pos = np.zeros(len(self.data.qpos), dtype=np.float64)

        # dynamic order
        self.dynamic_order = int(self.robot_cfg.dynamics_order)
        self.dynamics_model = self.robot_cfg.create_dynamics_model()
        self.dynamics_executor = DynamicsExecutor(
            self.dynamics_model,
            dt=self.dt * self.control_decimation,
            integrator=self.model_integrator,
            substeps=self.model_substeps,
            seed=kwargs.get("dynamics_seed"),
        )

        # controller gains (aligned with MujocoMotors indexing)
        self.kps = np.array(
            [
                self.robot_cfg.MujocoMotorKps[mj_ctrl_idx]
                for mj_ctrl_idx in self.robot_cfg.MujocoMotors
            ],
            dtype=np.float64,
        )
        self.kds = np.array(
            [
                self.robot_cfg.MujocoMotorKds[mj_ctrl_idx]
                for mj_ctrl_idx in self.robot_cfg.MujocoMotors
            ],
            dtype=np.float64,
        )
        self.enable_hand_control = bool(kwargs.get("enable_hand_control", False))
        self._init_embodiment_control(**kwargs)

        # forward once to initialize derived quantities
        mujoco.mj_forward(self.model, self.data)

        # ----------------------------- obstacle debug ----------------------------- #
        self.obstacle_debug = kwargs.get(
            "obstacle_debug",
            dict(num_obstacle=0, manual_movement_step_size=0.1),
        )
        self.num_obstacle_debug = int(self.obstacle_debug.get("num_obstacle", 0))
        self.manual_step_size = float(self.obstacle_debug.get("manual_movement_step_size", 0.1))
        self.manual_step_size_min = float(
            self.obstacle_debug.get("manual_movement_step_size_min", 0.001)
        )
        self.manual_step_size_max = float(
            self.obstacle_debug.get("manual_movement_step_size_max", 0.5)
        )
        self.manual_step_size_delta = float(
            self.obstacle_debug.get("manual_movement_step_size_delta", 0.01)
        )
        self.keyboard_hold_step_rate = float(
            self.obstacle_debug.get("keyboard_hold_step_rate", 10.0)
        )
        self.continuous_keyboard_control = bool(
            self.obstacle_debug.get("continuous_keyboard_control", True)
        )
        self._last_continuous_keyboard_time = time.time()
        self.manual_step_size = float(
            np.clip(self.manual_step_size, self.manual_step_size_min, self.manual_step_size_max)
        )

        if self.num_obstacle_debug > 0:
            self.obstacle_debug_frame = np.stack(
                [np.eye(4) for _ in range(self.num_obstacle_debug)], axis=0
            )

            # random placement (keep your original heuristic)
            for frame in self.obstacle_debug_frame:
                frame[:3, 3] = np.array([0.6, 0.0, 0.793], dtype=np.float64) + np.random.uniform(
                    -0.2, 0.2, 3
                )

            self.obstacle_debug_geom = [
                Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug)
                for _ in range(self.num_obstacle_debug)
            ]
            self.obstacle_debug_selected = 0
            self.num_obstacle_debug_change_buf = 0

        # velocity bookkeeping
        self.obstacle_debug_frame_last = self.obstacle_debug_frame.copy()
        self.obstacle_debug_velocity = [
            np.hstack(
                (
                    (
                        self.obstacle_debug_frame[i][:3, 3]
                        - self.obstacle_debug_frame_last[i][:3, 3]
                    ),
                    np.zeros(3),
                )
            )
            for i in range(self.num_obstacle_debug)
        ]

        # ----------------------------- viewer / renderer ----------------------------- #
        if self.enable_viewer:
            try:
                self.viewer = mujoco.viewer.launch_passive(
                    self.model,
                    self.data,
                    key_callback=self._key_callback,
                    show_left_ui=self.viewer_show_left_ui,
                    show_right_ui=self.viewer_show_right_ui,
                )
            except TypeError:
                self.viewer = mujoco.viewer.launch_passive(
                    self.model,
                    self.data,
                    key_callback=self._key_callback,
                )
            self.viewer_setup()
            self._remember_viewer_visual_state()
            self.renderer = mujoco.Renderer(
                self.model,
                height=self.record_height if self.recording_enabled else self.height,
                width=self.record_width if self.recording_enabled else self.width,
            )
        else:
            self.viewer = None
            self.renderer = (
                mujoco.Renderer(self.model, height=self.record_height, width=self.record_width)
                if self.recording_enabled
                else None
            )

        self._render_scene_prepared = False
        self._record_elapsed = 0.0
        self._record_next_frame_time = 0.0
        self._record_frame_count = 0
        self._record_complete = False
        self._record_writer = None
        self._record_gif_frames = []
        self._recording_closed = False
        self._record_camera = self._make_record_camera()

        # ----------------------------- camera (optional) ----------------------------- #
        self.camera_renderer = None
        self.depth_renderer = None
        self.cam_step = 0
        self.fixed_cam = None
        self.head_cam = None
        self.right_cam = None
        self.left_cam = None
        self.camera_handles = {}
        self.camera_model = None
        self.camera_data = None

        if self.enable_camera:
            self.camera_model = self._load_mujoco_camera_model()
            self._apply_configured_visual_palette(self.camera_model)
            self._configure_camera_offscreen_buffer(self.camera_model)
            self.camera_model.opt.timestep = self.dt
            self.camera_data = (
                self.data if self.camera_model is self.model else mujoco.MjData(self.camera_model)
            )
            self._sync_camera_model_state()
            self.camera_renderer = mujoco.Renderer(
                self.camera_model, height=self.camera_height, width=self.camera_width
            )
            self._init_camera_handles()

        # ----------------------------- viz options ----------------------------- #
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.opt)

    # ---------------------------------- Setup Helpers --------------------------------- #

    def _apply_configured_visual_palette(self, model=None) -> None:
        """Apply an optional robot-config palette without changing MJCF dynamics."""

        model = self.model if model is None else model
        palette = tuple(getattr(self.robot_cfg, "visual_link_colors", ()))
        if not palette:
            return
        records = tuple(
            (str(pattern).lower(), np.asarray(rgba, dtype=float)) for pattern, rgba in palette
        )
        for geom_id in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            normalized = name.rsplit("/", 1)[-1]
            if normalized.lower().endswith("_visual"):
                normalized = normalized[:-7]
            color = next(
                (rgba for pattern, rgba in records if fnmatchcase(normalized.lower(), pattern)),
                None,
            )
            if color is not None:
                model.geom_rgba[geom_id] = color

    def reset(self, **kwargs):
        super().reset(**kwargs)
        self.dynamics_executor.clear()
        self._keyboard_gripper_goal_override.clear()
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False

    def _advance_configured_dynamics(self, state, command, **kwargs):
        """Advance the config-selected model shared with DynamicsModelAgent."""

        action_info = kwargs.get("action_info") or {}
        model_state = self.dynamics_model.project_state(state)
        model_control = self.dynamics_model.project_control(command)
        next_model_state = self.dynamics_executor.step(
            model_control,
            state=model_state,
            parameters=action_info.get("dynamics_parameters"),
            exogenous=action_info.get("dynamics_exogenous"),
        )
        return self.dynamics_model.merge_state(next_model_state, into=state)

    def _joint_acceleration_actuator_forces(self, command) -> dict[int, float]:
        """Map configured joint accelerations to current actuator forces.

        Second-order robot configurations expose acceleration controls, while
        MuJoCo assets may use either raw motors or affine position actuators.
        Compute the common inverse-dynamics force here; embodiment agents then
        express that force through their actual actuator parameterization.
        """

        command = np.asarray(command, dtype=np.float64).reshape(self.num_control)
        desired_qacc = np.zeros(self.model.nv, dtype=np.float64)
        actuator_dofs = {}
        for mj_motor in self.robot_cfg.MujocoMotors:
            actuator_id = int(mj_motor)
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            dof_address = int(self.model.jnt_dofadr[joint_id])
            control = self.robot_cfg.MujocoMotor_to_Control[mj_motor]
            desired_qacc[dof_address] = command[int(control)]
            actuator_dofs[actuator_id] = dof_address

        # ``mj_fullM`` changed Python signatures when MuJoCo stopped exposing
        # ``MjData.qM``.  We only need M @ qacc here, and ``mj_mulM`` has the
        # same model/data/vector contract across supported MuJoCo releases.
        inertial_force = np.empty(self.model.nv, dtype=np.float64)
        mujoco.mj_mulM(self.model, self.data, inertial_force, desired_qacc)
        generalized_force = inertial_force + self.data.qfrc_bias - self.data.qfrc_passive
        result = {}
        for actuator_id, dof_address in actuator_dofs.items():
            moment = float(self.model.actuator_gear[actuator_id, 0])
            if abs(moment) <= 1.0e-9:
                continue
            result[actuator_id] = float(generalized_force[dof_address] / moment)
        return result

    def _init_embodiment_control(self, **kwargs) -> None:
        """Initialize optional embodiment-specific actuators.

        Robot-specific mixins override this hook. The generic backend does not
        import or inspect any particular robot family.
        """
        del kwargs

    def _resolve_mujoco_model_path(self, model_path: str) -> str:
        if model_path is None:
            raise ValueError("mujoco_model_path cannot be None.")

        model_path = os.path.expanduser(os.path.expandvars(str(model_path)))
        if os.path.isabs(model_path):
            return model_path
        return os.path.join(SPARK_ROBOT_RESOURCE_DIR, model_path)

    def _normalize_camera_config(self, camera_config):
        normalized = normalize_sensor_camera_config(camera_config, self.viewer_config)
        return [dict(name=name, **config) for name, config in normalized.items()]

    def _load_mujoco_model(self):
        # Keep dynamically injected sensor cameras out of the interactive
        # physics model. MuJoCo binds '[' and ']' to cycling model cameras,
        # while SPARK reserves those keys for gripper control. The offscreen
        # camera model below retains the exact link-mounted sensor view.
        return mujoco.MjModel.from_xml_path(self.mujoco_xml_path)

    def _load_mujoco_camera_model(self):
        xml_string = self._mujoco_xml_with_agent_cameras()
        if xml_string is not None:
            return mujoco.MjModel.from_xml_string(xml_string)
        return self.model

    def _configure_camera_offscreen_buffer(self, model=None):
        if not (self.enable_camera or self.recording_enabled):
            return

        model = self.model if model is None else model
        model.vis.global_.offwidth = max(
            int(model.vis.global_.offwidth),
            int(self.camera_width if self.enable_camera else 0),
            int(self.record_width if self.recording_enabled else 0),
        )
        model.vis.global_.offheight = max(
            int(model.vis.global_.offheight),
            int(self.camera_height if self.enable_camera else 0),
            int(self.record_height if self.recording_enabled else 0),
        )

    def _mujoco_xml_with_agent_cameras(self):
        if not (self.enable_camera and self.camera_config):
            return None

        tree = ET.parse(self.mujoco_xml_path)
        root = tree.getroot()
        added = False

        compiler = root.find("compiler")
        if compiler is not None:
            xml_dir = os.path.dirname(self.mujoco_xml_path)
            for attr in ("meshdir", "texturedir", "assetdir"):
                path = compiler.get(attr)
                if path and not os.path.isabs(path):
                    compiler.set(attr, os.path.abspath(os.path.join(xml_dir, path)))

        existing_camera_names = {
            camera.get("name") for camera in root.iter("camera") if camera.get("name") is not None
        }
        bodies = {
            body.get("name"): body for body in root.iter("body") if body.get("name") is not None
        }

        for cfg in self.camera_config:
            body_name = cfg.get("body_name", None)
            if not body_name:
                continue

            camera_name = cfg["mujoco_camera_name"]
            if camera_name in existing_camera_names:
                continue

            body = bodies.get(body_name, None)
            if body is None:
                print(
                    f"[mujoco_agent] Camera '{cfg['name']}' requested body '{body_name}', "
                    "but the body is not in the model; skipping camera injection."
                )
                continue

            attrs = {
                "name": camera_name,
                "mode": cfg.get("mode", "fixed"),
                "pos": self._format_xml_vector(cfg.get("pos", [0.0, 0.0, 0.0])),
                "fovy": str(float(cfg.get("fovy", 58.0))),
            }
            for key in ("xyaxes", "quat", "euler"):
                if key in cfg and cfg[key] is not None:
                    attrs[key] = self._format_xml_vector(cfg[key])

            ET.SubElement(body, "camera", attrs)
            existing_camera_names.add(camera_name)
            added = True

        if not added:
            return None
        return ET.tostring(root, encoding="unicode")

    def _format_xml_vector(self, value) -> str:
        return " ".join(f"{float(item):.9g}" for item in np.asarray(value, dtype=float).reshape(-1))

    def _init_camera_handles(self):
        camera_model = self.camera_model if self.camera_model is not None else self.model
        for cfg in self.camera_config:
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)

            camera_name = cfg.get("mujoco_camera_name", None)
            camera_id = -1
            if cfg.get("type", "fixed") == "fixed" and camera_name:
                camera_id = mujoco.mj_name2id(
                    camera_model,
                    mujoco.mjtObj.mjOBJ_CAMERA,
                    camera_name,
                )
                if camera_id < 0:
                    print(
                        f"[mujoco_agent] Camera '{cfg['name']}' references missing MuJoCo camera "
                        f"'{camera_name}'; skipping."
                    )
                    continue
                cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                cam.fixedcamid = camera_id
            else:
                cam.lookat[:] = np.asarray(cfg.get("lookat", [0.1, 0.0, 1.0]), dtype=float).reshape(
                    3
                )
                cam.distance = float(cfg.get("distance", 0.4))
                cam.elevation = float(cfg.get("elevation", -60.0))
                cam.azimuth = float(cfg.get("azimuth", 0.0))

            cfg["_mujoco_camera_id"] = int(camera_id)
            self.camera_handles[cfg["name"]] = cam

        self.fixed_cam = self.camera_handles.get("fixed", self.camera_handles.get("overview", None))
        self.head_cam = self.camera_handles.get("head", None)

    def _sync_camera_model_state(self):
        """Mirror physics state into the camera-only model when one is used."""

        camera_model = getattr(self, "camera_model", None)
        camera_data = getattr(self, "camera_data", None)
        physics_data = getattr(self, "data", None)
        if (
            camera_model is None
            or camera_data is None
            or physics_data is None
            or camera_data is physics_data
        ):
            return

        for attribute in ("qpos", "qvel", "act", "ctrl", "mocap_pos", "mocap_quat", "userdata"):
            source = getattr(physics_data, attribute, None)
            target = getattr(camera_data, attribute, None)
            if source is not None and target is not None and source.shape == target.shape:
                target[...] = source
        camera_data.time = physics_data.time
        mujoco.mj_forward(camera_model, camera_data)

    def close_viewer(self):
        """
        Closes the Mujoco viewer if it is open.
        """
        if self.viewer:
            self.viewer.close()  # Close the viewer window
            self.viewer = None  # Set viewer to None
        self._close_recording()
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        for name in ("camera_renderer", "depth_renderer"):
            renderer = getattr(self, name, None)
            if renderer is not None:
                renderer.close()
                setattr(self, name, None)
        if self.camera_display:
            cv2.destroyAllWindows()

    def is_running(self) -> bool:
        """Stop the pipeline when the interactive MuJoCo window is closed."""
        if self.recording_enabled and self._record_complete:
            return False
        if self.viewer is None:
            return True
        return bool(self.viewer.is_running())

    def _make_record_camera(self):
        if not self.recording_enabled:
            return None
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = np.asarray(self.viewer_config["camera_lookat"], dtype=float)
        camera.distance = float(self.viewer_config["camera_distance"])
        camera.azimuth = float(self.viewer_config["camera_azimuth"])
        camera.elevation = float(self.viewer_config["camera_elevation"])
        return camera

    def begin_render_frame(self):
        """Prepare the offscreen scene before pipeline overlays are appended."""
        if self.renderer is None:
            return
        # Camera capture for this step has already consumed the preceding
        # completed frame. Build a fresh overlay list alongside this frame's
        # viewer visualization.
        self._camera_overlay_geoms.clear()
        camera = self._record_camera if self.recording_enabled else None
        # MuJoCo 3.3+ treats an explicitly supplied ``None`` as a camera id
        # and compares it with integer camera bounds. Omit the argument for
        # interactive rendering so the renderer selects its default free
        # camera; pass the configured camera only for deterministic recording.
        if camera is None:
            self.renderer.update_scene(self.data)
        else:
            self.renderer.update_scene(self.data, camera=camera)
        self._render_scene_prepared = True

    def _record_frame(self):
        if not self.recording_enabled or self._record_complete:
            return
        step_duration = float(self.dt) * int(self.control_decimation)
        self._record_elapsed += step_duration
        frame_period = 1.0 / self.record_fps
        if self._record_elapsed + 1.0e-12 < self._record_next_frame_time:
            return

        rgb = np.asarray(self.renderer.render(), dtype=np.uint8)
        while self._record_next_frame_time <= self._record_elapsed + 1.0e-12:
            if self.record_video_path:
                if self._record_writer is None:
                    output = Path(self.record_video_path).expanduser().resolve()
                    output.parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(
                        str(output),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        self.record_fps,
                        (self.record_width, self.record_height),
                    )
                    if not writer.isOpened():
                        raise RuntimeError(f"Could not open video output {output}")
                    self._record_writer = writer
                self._record_writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            if self.record_gif_path:
                self._record_gif_frames.append(rgb.copy())
            self._record_frame_count += 1
            self._record_next_frame_time += frame_period
            if self.record_duration > 0.0 and self._record_frame_count >= int(
                np.ceil(self.record_duration * self.record_fps)
            ):
                self._record_complete = True
                break

    def _close_recording(self):
        if self._recording_closed:
            return
        self._recording_closed = True
        if self._record_writer is not None:
            self._record_writer.release()
            self._record_writer = None
        if self.record_gif_path and self._record_gif_frames:
            import imageio.v2 as imageio

            output = Path(self.record_gif_path).expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            imageio.mimsave(
                output,
                self._record_gif_frames,
                format="GIF",
                duration=1000.0 / self.record_fps,
                loop=0,
            )
            self._record_gif_frames.clear()
        if self.recording_enabled and self._record_frame_count:
            print(f"Recorded {self._record_frame_count} frames at {self.record_fps:g} FPS")

    def viewer_setup(self):
        """
        Configures the viewer camera settings for optimal visualization.
        """
        # Camera distance and positioning
        # self.viewer.cam.distance = 2  # Set zoom level (distance from the object)
        # self.viewer.cam.lookat[0] = 0  # X offset for the camera focus point
        # self.viewer.cam.lookat[1] = 0  # Y offset for the camera focus point
        # self.viewer.cam.lookat[2] = 0.8  # Z offset for the camera focus point

        # # Camera rotations and angles
        # self.viewer.cam.elevation = -10  # Camera rotation around the X axis (up/down)
        # self.viewer.cam.azimuth = 180  # Camera rotation around the Y axis (left/right)

        # Camera distance and positioning
        # self.viewer.cam.distance = 1  # Set zoom level (distance from the object)
        # self.viewer.cam.lookat[0] = -0.8  # X offset for the camera focus point
        # self.viewer.cam.lookat[1] = -1.5  # Y offset for the camera focus point
        # self.viewer.cam.lookat[2] = 2.0  # Z offset for the camera focus point

        # # Camera rotations and angles
        # self.viewer.cam.elevation = -30  # Camera rotation around the X axis (up/down)
        # self.viewer.cam.azimuth = 60  # Camera rotation around the Y axis (left/right)

        self.viewer.cam.distance = self.viewer_config["camera_distance"]
        self.viewer.cam.lookat[:] = self.viewer_config["camera_lookat"]
        self.viewer.cam.elevation = self.viewer_config["camera_elevation"]
        self.viewer.cam.azimuth = self.viewer_config["camera_azimuth"]
        self._remember_free_viewer_camera()

        # Set geometry group for visualization
        self.viewer.opt.geomgroup = 1  # Set which geometry group to visualize

    def render_camera(self, cam):
        self.cam_step += 1
        MujocoAgent._sync_camera_model_state(self)
        camera_data = getattr(self, "camera_data", None)
        if camera_data is None:
            camera_data = self.data
        self.camera_renderer.update_scene(camera_data, camera=cam)
        self._render_camera_debug_overlays()
        rgb_buffer = self.camera_renderer.render()
        # Render RGB and metric depth from one scene and one projection.  Two
        # independent renderers can submit adjacent MuJoCo scene revisions,
        # which is visible as an RGB/depth offset while the robot is moving.
        self.camera_renderer.enable_depth_rendering()
        try:
            depth_buffer = self.camera_renderer.render()
        finally:
            self.camera_renderer.disable_depth_rendering()
        # if self.cam_step == 100:
        #     imageio.imwrite("rgb.png", rgb_buffer)

        #     # Convert depth (in meters) to millimeters and scale for visualization
        #     depth_vis = (depth_buffer * 1000).astype(np.uint16)  # depth in mm as 16-bit PNG
        #     imageio.imwrite("depth.png", depth_vis)
        return rgb_buffer.copy(), depth_buffer.copy()

    def _render_camera_debug_overlays(self):
        for item in self._camera_overlay_geoms:
            for renderer in (self.camera_renderer,):
                if renderer is None:
                    continue
                self._add_geom_to_scene(
                    renderer._scene,
                    item["type"],
                    item["size"],
                    item["pos"],
                    item["mat"],
                    item["color"],
                )
        for renderer in (self.camera_renderer,):
            if renderer is None:
                continue
            for frame, geom in zip(self.obstacle_debug_frame, self.obstacle_debug_geom):
                if geom.type == "sphere":
                    self._add_geom_to_scene(
                        renderer._scene,
                        mujoco.mjtGeom.mjGEOM_SPHERE,
                        geom.attributes["radius"] * np.ones(3),
                        frame[:3, 3],
                        np.eye(3),
                        geom.color,
                    )
                elif geom.type == "box":
                    self._add_geom_to_scene(
                        renderer._scene,
                        mujoco.mjtGeom.mjGEOM_BOX,
                        np.array(
                            [
                                geom.attributes["length"],
                                geom.attributes["width"],
                                geom.attributes["height"],
                            ]
                        )
                        / 2,
                        frame[:3, 3],
                        frame[:3, :3],
                        geom.color,
                    )

    def _add_geom_to_scene(self, scene, geom_type, size, pos, mat, color):
        maxgeom = getattr(scene, "maxgeom", None)
        if maxgeom is None:
            maxgeom = len(scene.geoms)
        if scene.ngeom >= maxgeom:
            return
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            type=geom_type,
            size=np.asarray(size, dtype=float).reshape(3),
            pos=np.asarray(pos, dtype=float).reshape(3),
            mat=np.asarray(mat, dtype=float).reshape(3, 3).flatten(),
            rgba=np.asarray(color, dtype=float).reshape(4),
        )
        self._mark_visual_overlay_geom(scene.geoms[scene.ngeom])
        scene.ngeom += 1

    @staticmethod
    def _mark_visual_overlay_geom(geom) -> None:
        """Keep debug/collision overlays translucent and out of scene shadows."""

        # ``transparent`` routes the geom through MuJoCo's transparent pass;
        # those geoms do not contribute opaque shadow silhouettes. DECOR also
        # makes the non-model nature of the primitive explicit to the viewer.
        if hasattr(geom, "transparent"):
            geom.transparent = 1
        if hasattr(geom, "category"):
            geom.category = int(mujoco.mjtCatBit.mjCAT_DECOR)
        if hasattr(geom, "specular"):
            geom.specular = 0.0
        if hasattr(geom, "reflectance"):
            geom.reflectance = 0.0

    def _capture_camera_feedback(self):
        if not (self.enable_camera and self.camera_renderer):
            return

        sim_time = float(self.data.time)
        capture_time = time.perf_counter()
        if (
            np.isfinite(self._last_camera_capture_time)
            and self._camera_capture_period > 0.0
            and capture_time - self._last_camera_capture_time < self._camera_capture_period
        ):
            return

        feedback = {}
        for cfg in self.camera_config:
            cam = self.camera_handles.get(cfg["name"], None)
            if cam is None:
                continue

            rgb, depth = self.render_camera(cam)
            frame_id = cfg.get("frame_id", f"{cfg['name']}_optical_frame")
            camera_id = int(cfg.get("_mujoco_camera_id", -1))
            camera_frame = self._camera_world_frame(camera_id)
            camera_info = self._camera_info(cfg, camera_id)

            feedback[cfg["name"]] = {
                "rgb": rgb,
                "depth": depth.astype(np.float32, copy=False),
                "stamp": sim_time,
                "frame_id": frame_id,
                "camera_info": camera_info,
                "camera_frame": camera_frame,
            }

            if self.camera_display:
                self.stream_image(rgb, window_name=f"SPARK RGB Camera - {cfg['name']}")
                self.stream_depth(depth, window_name=f"SPARK Depth Camera - {cfg['name']}")

        self.camera_feedback = feedback
        self._last_camera_capture_time = capture_time

    def _camera_world_frame(self, camera_id: int):
        if camera_id < 0:
            return None
        camera_data = self.camera_data if self.camera_data is not None else self.data
        frame = np.eye(4, dtype=np.float64)
        frame[:3, :3] = camera_data.cam_xmat[camera_id].reshape(3, 3)
        frame[:3, 3] = camera_data.cam_xpos[camera_id]
        return frame

    def _camera_info(self, cfg, camera_id: int):
        width = self.camera_width
        height = self.camera_height
        fovy = float(cfg.get("fovy", 58.0))
        if camera_id >= 0:
            camera_model = self.camera_model if self.camera_model is not None else self.model
            fovy = float(camera_model.cam_fovy[camera_id])

        fy = 0.5 * height / np.tan(0.5 * np.deg2rad(fovy))
        fx = fy
        cx = 0.5 * (width - 1)
        cy = 0.5 * (height - 1)

        if "intrinsics" in cfg:
            intrinsics = cfg["intrinsics"]
            fx = float(intrinsics.get("fx", fx))
            fy = float(intrinsics.get("fy", fy))
            cx = float(intrinsics.get("cx", cx))
            cy = float(intrinsics.get("cy", cy))

        k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return {
            "width": width,
            "height": height,
            "fovy": fovy,
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "K": k,
            "R": r,
            "P": p,
            "D": list(cfg.get("distortion", [])),
            "distortion_model": cfg.get("distortion_model", "plumb_bob"),
        }

    def get_camera_feedback(self):
        if not self.enable_camera:
            return {}
        return {
            name: {
                **data,
                "rgb": data["rgb"].copy(),
                "depth": data["depth"].copy(),
                "camera_frame": None
                if data["camera_frame"] is None
                else data["camera_frame"].copy(),
            }
            for name, data in self.camera_feedback.items()
        }

    def stream_image(
        self,
        rgb_image,
        win_size=(320, 240),
        window_name="SPARK RGB Camera",
    ):
        bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        bgr_resized = cv2.resize(bgr, win_size, interpolation=cv2.INTER_AREA)
        cv2.imshow(window_name, bgr_resized)
        cv2.waitKey(1)

    def stream_depth(
        self,
        depth_image,
        win_size=(320, 240),
        window_name="SPARK Depth Camera",
    ):
        depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = depth_norm.astype(np.uint8)
        depth_resized = cv2.resize(depth_display, win_size, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(window_name, depth_resized)
        cv2.waitKey(1)

    # ---------------------------------- User Interface Helpers--------------------------------- #

    def _add_obstacle(self):
        """
        Adds a new obstacle to the simulation and moves the pointer to the newly added obstacle.
        The obstacle is a sphere with a random position near a defined point.
        """
        self.num_obstacle_debug += 1  # Increment the obstacle count
        # Append a new obstacle's frame (4x4 identity matrix) to the list of obstacle frames
        self.obstacle_debug_frame = np.concatenate(
            [self.obstacle_debug_frame, np.eye(4)[None, :, :]], axis=0
        )

        # Set random position for the new obstacle near the point [0.6, 0.0, 0.793]
        self.obstacle_debug_frame[-1, :3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(
            -0.2, 0.2, 3
        )
        self.obstacle_debug_frame_last = np.concatenate(
            [self.obstacle_debug_frame_last, self.obstacle_debug_frame[-1][None, :, :]], axis=0
        )
        # Add a new sphere obstacle with a radius of 0.05 and a color for visualization
        self.obstacle_debug_geom.append(
            Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug)
        )

        # Set the newly added obstacle as the selected one
        self.obstacle_debug_selected = self.num_obstacle_debug - 1
        self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]

    def _remove_obstacle(self):
        """
        Removes the currently selected obstacle from the simulation.
        """
        if self.num_obstacle_debug > 0:
            self.num_obstacle_debug -= 1  # Decrement the obstacle count

            # Remove the selected obstacle's frame and geometry
            self.obstacle_debug_frame = np.concatenate(
                [
                    self.obstacle_debug_frame[: self.obstacle_debug_selected, :, :],
                    self.obstacle_debug_frame[self.obstacle_debug_selected + 1 :, :, :],
                ],
                axis=0,
            )
            self.obstacle_debug_frame_last = np.concatenate(
                [
                    self.obstacle_debug_frame_last[: self.obstacle_debug_selected, :],
                    self.obstacle_debug_frame_last[self.obstacle_debug_selected + 1 :, :],
                ],
                axis=0,
            )
            self.obstacle_debug_geom = (
                self.obstacle_debug_geom[: self.obstacle_debug_selected]
                + self.obstacle_debug_geom[self.obstacle_debug_selected + 1 :]
            )

            # Update the selected obstacle
            self.obstacle_debug_selected = 0 if self.num_obstacle_debug > 0 else None
            if self.obstacle_debug_selected is None:
                self.debug_object = self.base_goal_debug_frame
            else:
                self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]

    def _cycle_obstacle_debug_object(self):
        if self.num_obstacle_debug <= 0:
            return

        if self.obstacle_debug_selected is None:
            self.obstacle_debug_selected = 0
        else:
            self.obstacle_debug_selected = (
                self.obstacle_debug_selected + 1
            ) % self.num_obstacle_debug
        self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]

    def _selected_goal_debug_side(self):
        if self.debug_object is self.right_goal_debug_frame:
            return "right"
        if self.debug_object is self.left_goal_debug_frame:
            return "left"
        return None

    def _default_keyboard_gripper_sides(self):
        if "LeftArm" in self.class_name and "DualArm" not in self.class_name:
            return ("left",)
        if any(name in self.class_name for name in ("RightArm", "SingleArm")):
            return ("right",)
        return ()

    def _set_gripper_debug_state(self, side: str, closed: bool):
        closed = bool(closed)
        if side == "right":
            self.right_gripper_debug_state = closed
            self.right_gripper_goal = closed
            self._keyboard_gripper_goal_override["right"] = closed
        elif side == "left":
            self.left_gripper_debug_state = closed
            self.left_gripper_goal = closed
            self._keyboard_gripper_goal_override["left"] = closed

    def _resolve_binary_gripper_goal(
        self,
        side: str,
        action_info: Optional[Mapping[str, Any]] = None,
        *,
        coerce=bool,
    ) -> bool:
        """Resolve pipeline and keyboard gripper commands consistently.

        A keyboard command remains authoritative until the pipeline reports the
        same goal. This prevents a held ``False`` value in teleoperation action
        metadata from immediately undoing a ``[`` or ``]`` key press.
        """

        if side not in ("left", "right"):
            raise ValueError(f"Unsupported gripper side: {side!r}")

        action_info = {} if action_info is None else action_info
        goal_key = f"{side}_gripper_goal"
        action_goal = action_info.get(
            goal_key,
            action_info.get("gripper_goal") if side == "right" else None,
        )
        if side in self._keyboard_gripper_goal_override:
            keyboard_goal = bool(self._keyboard_gripper_goal_override[side])
            if action_goal is not None and bool(coerce(action_goal)) == keyboard_goal:
                self._keyboard_gripper_goal_override.pop(side, None)
            else:
                action_goal = keyboard_goal

        current_goal = bool(getattr(self, goal_key, False))
        goal = current_goal if action_goal is None else bool(coerce(action_goal))
        setattr(self, goal_key, goal)
        setattr(self, f"{side}_gripper_debug_state", goal)
        return goal

    def _set_resolved_gripper_goal(self, side: str, closed: bool) -> bool:
        """Record an explicit actuator-level gripper target."""

        if side not in ("left", "right"):
            raise ValueError(f"Unsupported gripper side: {side!r}")
        closed = bool(closed)
        self._keyboard_gripper_goal_override.pop(side, None)
        setattr(self, f"{side}_gripper_goal", closed)
        setattr(self, f"{side}_gripper_debug_state", closed)
        return closed

    def _set_robotiq_2f85_qpos(self, side: str, closed: bool) -> None:
        """Apply the coupled 2F-85 linkage in model-owned dynamics mode.

        Model-owned arm execution does not call ``mj_step``, so actuator force
        alone cannot move passive gripper joints. Set the settled linkage pose
        directly while preserving MuJoCo/Isaac's binary open/closed contract.
        """

        if side not in ("left", "right"):
            raise ValueError(f"Unsupported gripper side: {side!r}")
        closed_targets = {
            "driver_joint": 0.79,
            "coupler_joint": 0.0,
            "spring_link_joint": 0.79,
            "follower_joint": -0.775,
        }
        for finger in ("right", "left"):
            for suffix, closed_target in closed_targets.items():
                joint_name = f"{side}/{finger}_{suffix}"
                joint_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    joint_name,
                )
                if joint_id < 0:
                    continue
                qpos_index = int(self.model.jnt_qposadr[joint_id])
                dof_index = int(self.model.jnt_dofadr[joint_id])
                self.data.qpos[qpos_index] = closed_target if closed else 0.0
                if dof_index >= 0:
                    self.data.qvel[dof_index] = 0.0

    def _set_selected_gripper_debug_state(self, closed: bool):
        side = self._selected_goal_debug_side()
        if side is None:
            default_sides = self._default_keyboard_gripper_sides()
            if not default_sides:
                print("[gripper] Select the right hand with O or the left hand with P first")
                return
            for default_side in default_sides:
                self._set_gripper_debug_state(default_side, closed)
            return
        self._set_gripper_debug_state(side, closed)

    def _remember_free_viewer_camera(self):
        """Retain the user's interactive camera before MuJoCo cycles cameras."""

        if self.viewer is None or self.viewer.cam.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
        self._viewer_free_camera_state = {
            "lookat": np.asarray(self.viewer.cam.lookat, dtype=float).copy(),
            "distance": float(self.viewer.cam.distance),
            "elevation": float(self.viewer.cam.elevation),
            "azimuth": float(self.viewer.cam.azimuth),
        }

    def _remember_viewer_visual_state(self):
        """Retain viewer settings that overlap SPARK keyboard bindings."""

        if self.viewer is None:
            return
        self._remember_free_viewer_camera()
        state = {}
        option = getattr(self.viewer, "opt", None)
        if option is not None:
            for attribute in ("geomgroup", "sitegroup", "frame", "label"):
                value = getattr(option, attribute, None)
                if value is not None:
                    state[attribute] = np.asarray(value).copy()
        self._viewer_visual_state = state

    def _request_viewer_state_restore(self):
        # MuJoCo reserves several SPARK goal-control keys for camera and geom
        # display shortcuts.  The native shortcut can run before the Python
        # callback, so do not sample the viewer here: that would preserve the
        # already-toggled state.  ``render`` continuously records the last
        # synchronized state while no SPARK key restore is pending.
        self._restore_viewer_state_after_spark_key = True

    def _restore_or_remember_viewer_state(self):
        if self.viewer is None:
            return
        if not self._restore_viewer_state_after_spark_key:
            self._remember_viewer_visual_state()
            return
        state = self._viewer_free_camera_state
        if state is None:
            state = {
                "lookat": np.asarray(self.viewer_config["camera_lookat"], dtype=float),
                "distance": float(self.viewer_config["camera_distance"]),
                "elevation": float(self.viewer_config["camera_elevation"]),
                "azimuth": float(self.viewer_config["camera_azimuth"]),
            }
        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.viewer.cam.fixedcamid = -1
        self.viewer.cam.lookat[:] = state["lookat"]
        self.viewer.cam.distance = state["distance"]
        self.viewer.cam.elevation = state["elevation"]
        self.viewer.cam.azimuth = state["azimuth"]
        option = getattr(self.viewer, "opt", None)
        if option is not None:
            for attribute, value in (self._viewer_visual_state or {}).items():
                target = getattr(option, attribute, None)
                if target is None:
                    continue
                if np.asarray(target).shape == np.asarray(value).shape:
                    if np.asarray(value).ndim == 0:
                        setattr(option, attribute, np.asarray(value).item())
                    else:
                        target[...] = value
        self._restore_viewer_state_after_spark_key = False

    def _restore_or_remember_free_viewer_camera(self):
        """Compatibility wrapper for the former gripper-only restoration."""

        if not hasattr(self, "_restore_viewer_state_after_spark_key"):
            self._restore_viewer_state_after_spark_key = bool(
                getattr(self, "_restore_free_camera_after_gripper_key", False)
            )
        if not hasattr(self, "_viewer_visual_state"):
            self._viewer_visual_state = {}
        MujocoAgent._restore_or_remember_viewer_state(self)
        self._restore_free_camera_after_gripper_key = self._restore_viewer_state_after_spark_key

    def _key_callback(self, key):
        """
        Handles key events for obstacle movement and selection.
        """
        if self.enable_keyboard_control is False:
            return

        self._ensure_debug_object()
        spark_keys = {
            glfw.KEY_RIGHT,
            glfw.KEY_LEFT,
            glfw.KEY_UP,
            glfw.KEY_DOWN,
            glfw.KEY_E,
            glfw.KEY_Q,
            glfw.KEY_2,
            glfw.KEY_3,
            glfw.KEY_4,
            glfw.KEY_5,
            glfw.KEY_6,
            glfw.KEY_7,
            glfw.KEY_8,
            glfw.KEY_9,
            glfw.KEY_LEFT_BRACKET,
            glfw.KEY_RIGHT_BRACKET,
            glfw.KEY_MINUS,
            glfw.KEY_KP_SUBTRACT,
            glfw.KEY_EQUAL,
            glfw.KEY_KP_ADD,
            glfw.KEY_SPACE,
            glfw.KEY_PAGE_UP,
            glfw.KEY_PAGE_DOWN,
            glfw.KEY_O,
            glfw.KEY_P,
            glfw.KEY_B,
            glfw.KEY_V,
            glfw.KEY_N,
        }
        if key in spark_keys:
            self._request_viewer_state_restore()
        # Handle movement keys for the selected obstacle
        s = self.manual_step_size

        # Move keys (example mapping consistent with your existing comments)
        if key == glfw.KEY_RIGHT:  # +Y
            self._translate_debug_object([0, +s, 0])
        elif key == glfw.KEY_LEFT:  # -Y
            self._translate_debug_object([0, -s, 0])
        elif key == glfw.KEY_UP:  # -X
            self._translate_debug_object([-s, 0, 0])
        elif key == glfw.KEY_DOWN:  # +X
            self._translate_debug_object([+s, 0, 0])
        elif key == glfw.KEY_E:  # +Z
            self._translate_debug_object([0, 0, +s])
        elif key == glfw.KEY_Q:  # -Z
            self._translate_debug_object([0, 0, -s])
        elif key == glfw.KEY_2:  # Rotate +Yaw
            rotation = R.from_euler("xyz", [0.0, 0.0, 0.1], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
        elif key == glfw.KEY_3:  # Rotate -Yaw
            rotation = R.from_euler("xyz", [0.0, 0.0, -0.1], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
        elif key == glfw.KEY_4:  # Rotate +Pitch
            rotation = R.from_euler("xyz", [0.0, 0.1, 0.0], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
        elif key == glfw.KEY_5:  # Rotate -Pitch
            rotation = R.from_euler("xyz", [0.0, -0.1, 0.0], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
        elif key == glfw.KEY_6:  # Rotate +Roll
            rotation = R.from_euler("xyz", [0.1, 0.0, 0.0], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
        elif key == glfw.KEY_7:  # Rotate -Roll
            rotation = R.from_euler("xyz", [-0.1, 0.0, 0.0], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
        elif key == glfw.KEY_8:  # Close Gripper
            self._set_selected_gripper_debug_state(True)
        elif key == glfw.KEY_9:  # Open Gripper
            self._set_selected_gripper_debug_state(False)
        elif key == glfw.KEY_LEFT_BRACKET:  # Open selected gripper
            self._set_selected_gripper_debug_state(False)
        elif key == glfw.KEY_RIGHT_BRACKET:  # Close selected gripper
            self._set_selected_gripper_debug_state(True)

        elif key in (glfw.KEY_MINUS, glfw.KEY_KP_SUBTRACT):
            self._adjust_manual_step_size(-self.manual_step_size_delta)
        elif key in (glfw.KEY_EQUAL, glfw.KEY_KP_ADD):
            self._adjust_manual_step_size(self.manual_step_size_delta)

        # Switch to the next obstacle when SPACE is pressed
        elif key == glfw.KEY_SPACE:
            self._cycle_obstacle_debug_object()

        # Adjust the number of obstacles based on PAGE_UP/PAGE_DOWN keys
        elif key == glfw.KEY_PAGE_UP:
            self.num_obstacle_debug_change_buf += 1
        elif key == glfw.KEY_PAGE_DOWN:
            self.num_obstacle_debug_change_buf -= 1

        elif key == glfw.KEY_O:
            self.debug_object = self.right_goal_debug_frame
        elif key == glfw.KEY_P:
            self.debug_object = self.left_goal_debug_frame
        elif key == glfw.KEY_B:
            self.debug_object = self.base_goal_debug_frame

        elif key == glfw.KEY_V:
            self.render_robot_collision_volumes = not self.render_robot_collision_volumes
            print(
                "[visualization] robot collision volumes = "
                f"{'ON' if self.render_robot_collision_volumes else 'OFF'}"
            )

        elif key == glfw.KEY_N:
            self.translate_in_local_frame = not self.translate_in_local_frame
            print(
                f"[debug] translate frame = {'LOCAL' if self.translate_in_local_frame else 'WORLD'}"
            )
        return

    def _ensure_debug_object(self):
        if self.debug_object is not None:
            return
        if self.obstacle_debug_selected is not None and self.num_obstacle_debug > 0:
            self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]
        else:
            self.debug_object = self.base_goal_debug_frame

    def _viewer_window(self):
        if self.viewer is None:
            return None
        for name in ("_window", "window"):
            window = getattr(self.viewer, name, None)
            if window is not None:
                return window
        return None

    def _is_key_pressed(self, window, key) -> bool:
        try:
            state = glfw.get_key(window, key)
        except Exception:
            return False
        repeat = getattr(glfw, "REPEAT", glfw.PRESS)
        return state in (glfw.PRESS, repeat)

    def _apply_continuous_keyboard_control(self):
        if not (self.enable_keyboard_control and self.continuous_keyboard_control):
            self._last_continuous_keyboard_time = time.time()
            return

        window = self._viewer_window()
        if window is None:
            self._last_continuous_keyboard_time = time.time()
            return

        now = time.time()
        dt = max(0.0, min(now - self._last_continuous_keyboard_time, 0.1))
        self._last_continuous_keyboard_time = now
        step_scale = dt * max(self.keyboard_hold_step_rate, 0.0)
        if step_scale <= 0.0:
            return

        self._ensure_debug_object()
        s = self.manual_step_size * step_scale
        translation_keys = (
            (glfw.KEY_RIGHT, [0, +s, 0]),
            (glfw.KEY_LEFT, [0, -s, 0]),
            (glfw.KEY_UP, [-s, 0, 0]),
            (glfw.KEY_DOWN, [+s, 0, 0]),
            (glfw.KEY_E, [0, 0, +s]),
            (glfw.KEY_Q, [0, 0, -s]),
        )
        for key, delta in translation_keys:
            if self._is_key_pressed(window, key):
                self._translate_debug_object(delta)

        rotation_step = 0.1 * step_scale
        rotation_keys = (
            (glfw.KEY_2, [0.0, 0.0, +rotation_step]),
            (glfw.KEY_3, [0.0, 0.0, -rotation_step]),
            (glfw.KEY_4, [0.0, +rotation_step, 0.0]),
            (glfw.KEY_5, [0.0, -rotation_step, 0.0]),
            (glfw.KEY_6, [+rotation_step, 0.0, 0.0]),
            (glfw.KEY_7, [-rotation_step, 0.0, 0.0]),
        )
        for key, rpy in rotation_keys:
            if self._is_key_pressed(window, key):
                rotation = R.from_euler("xyz", rpy, degrees=False)
                self.debug_object[:3, :3] @= rotation.as_matrix()

    def _adjust_manual_step_size(self, delta: float):
        self.manual_step_size = float(
            np.clip(
                self.manual_step_size + float(delta),
                self.manual_step_size_min,
                self.manual_step_size_max,
            )
        )
        print(f"[debug] manual movement step size = {self.manual_step_size:.4f} m")

    def _translate_debug_object(self, d_world: np.ndarray):
        """
        d_world: (3,) translation expressed in WORLD frame axes (your key intention).
        If local mode is enabled, the step is applied along the object's LOCAL axes instead.
        """
        d_world = np.asarray(d_world, dtype=float).reshape(3)

        if self.translate_in_local_frame:
            # local axes expressed in world coordinates are the columns of R
            R_w_o = self.debug_object[:3, :3]
            d = R_w_o @ d_world
        else:
            d = d_world

        self.debug_object[:3, 3] += d

    # ---------------------------------- Render Helpers --------------------------------- #

    def get_simulation_info(self) -> Dict[str, str]:
        """Return the backend-neutral metadata shown in the MuJoCo viewer."""
        dynamics_model = self.dynamics_model
        if self.dynamics_backend == "simulator":
            propagation = "MuJoCo physics"
        else:
            propagation = f"SPARK model ({self.dynamics_executor.integrator})"

        simulation_info = {
            "Robot config": self.class_name,
            "Dynamics model": dynamics_model.variant,
            "Dynamics order": str(dynamics_model.order),
            "Dimensions": (
                f"state={dynamics_model.state_dim}, control={dynamics_model.control_dim}"
            ),
            "Propagation": propagation,
            "Control period": f"{self.dt * self.control_decimation:g} s",
            "Simulation time": f"{float(self.data.time):.3f} s",
        }
        simulation_info.update(self._viewer_simulation_info)
        return simulation_info

    def set_viewer_simulation_info(
        self,
        information: Mapping[str, Any] | None = None,
        *,
        replace: bool = False,
    ) -> None:
        """Add or override viewer rows for externally propagated simulations.

        A MuJoCo agent may be used only as a visualizer while another SPARK
        environment generates the trajectory. Callers can then report that
        external model without changing the agent's actual dynamics objects.
        """
        if information is None:
            information = {}
        if not isinstance(information, Mapping):
            raise TypeError("information must be a mapping of viewer labels to values")
        if replace:
            self._viewer_simulation_info.clear()
        self._viewer_simulation_info.update(
            {str(label): str(value) for label, value in information.items()}
        )

    def _update_simulation_info_overlay(self) -> bool:
        """Refresh the optional text overlay supported by MuJoCo 3.4+."""
        if not self.viewer_show_simulation_info or self.viewer is None:
            return False

        set_texts = getattr(self.viewer, "set_texts", None)
        if not callable(set_texts):
            # Keep construction compatible with an older externally managed
            # MuJoCo installation even though SPARK's ``mujoco`` extra selects
            # a release with the public text-overlay API.
            return False

        simulation_info = self.get_simulation_info()
        labels = "SPARK simulation\n" + "\n".join(simulation_info)
        values = "\n" + "\n".join(simulation_info.values())
        set_texts(
            (
                mujoco.mjtFontScale.mjFONTSCALE_100,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                labels,
                values,
            )
        )
        return True

    def render(self):
        """Renders the simulation environment, including obstacles and scene updates."""
        self._apply_continuous_keyboard_control()
        self._restore_or_remember_viewer_state()

        # Render the virtual obstacle for debugging
        self._render_obstacle_debug()

        # If renderer is available, update the scene with current model data
        if self.renderer and not self._render_scene_prepared:
            self.renderer.update_scene(self.data)
        if self.renderer and self.recording_enabled:
            self._record_frame()
        self._render_scene_prepared = False

        # If viewer is available, synchronize the viewer with the current scene state
        if self.viewer:
            self._update_simulation_info_overlay()
            self.viewer.sync()

    def render_sphere(self, pos, mat, size, color):
        """Render a radial area (sphere) in the environment."""

        if self.renderer is None:
            return

        pos = np.asarray(pos)
        if pos.shape == (2,):
            pos = np.r_[pos, 0]  # Add Z-coordinate if only 2D position is given

        if self.enable_camera:
            self._camera_overlay_geoms.append(
                {
                    "type": mujoco.mjtGeom.mjGEOM_SPHERE,
                    "size": np.asarray(size, dtype=float).reshape(3).copy(),
                    "pos": np.asarray(pos, dtype=float).reshape(3).copy(),
                    "mat": np.eye(3, dtype=float),
                    "color": np.asarray(color, dtype=float).reshape(4).copy(),
                }
            )

        # Render the sphere in the main renderer scene
        renderer_geom = self.renderer._scene.geoms[self.renderer._scene.ngeom]
        mujoco.mjv_initGeom(
            renderer_geom,
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=size,
            pos=pos.flatten(),
            mat=np.eye(3).flatten(),
            rgba=color,
        )
        self._mark_visual_overlay_geom(renderer_geom)
        self.renderer._scene.ngeom += 1

        # Render the sphere in the viewer scene if a viewer is available
        if self.viewer:
            viewer_geom = self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom]
            mujoco.mjv_initGeom(
                viewer_geom,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=size * np.ones(3),
                pos=pos.flatten(),
                mat=np.eye(3).flatten(),
                rgba=color,
            )
            self._mark_visual_overlay_geom(viewer_geom)
            self.viewer.user_scn.ngeom += 1

    def render_box(self, pos, mat, size, color):
        """Render a rectangular area (box) in the environment."""

        if self.renderer is None:
            return

        pos = np.asarray(pos)
        if pos.shape == (2,):
            pos = np.r_[pos, 0]  # Add Z-coordinate if only 2D position is given

        if self.enable_camera:
            self._camera_overlay_geoms.append(
                {
                    "type": mujoco.mjtGeom.mjGEOM_BOX,
                    "size": (np.asarray(size, dtype=float).reshape(3) / 2.0).copy(),
                    "pos": np.asarray(pos, dtype=float).reshape(3).copy(),
                    "mat": np.asarray(mat, dtype=float).reshape(3, 3).copy(),
                    "color": np.asarray(color, dtype=float).reshape(4).copy(),
                }
            )

        # Render the box in the main renderer scene
        mujoco.mjv_initGeom(
            self.renderer._scene.geoms[self.renderer._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=size / 2,  # Half the size for box rendering
            pos=pos.flatten(),
            mat=mat.flatten(),
            rgba=color,
        )
        self.renderer._scene.ngeom += 1

        # Render the box in the viewer scene if a viewer is available
        if self.viewer:
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=size / 2,  # Half the size for box rendering
                pos=pos.flatten(),
                mat=mat.flatten(),
                rgba=color,
            )
            self.viewer.user_scn.ngeom += 1

    def render_line_segment(self, pos1, pos2, radius, color):
        """Render a line segment as a capsule between two positions."""

        if self.renderer is None:
            return

        pos1 = np.asarray(pos1, dtype=float).reshape(3)
        pos2 = np.asarray(pos2, dtype=float).reshape(3)

        # Compute midpoint and direction of the line segment
        midpoint = (pos1 + pos2) / 2
        length = np.linalg.norm(pos2 - pos1)
        if not np.isfinite(length) or length <= 1e-9:
            return
        direction = (pos2 - pos1) / length

        # Compute the rotation matrix to align the capsule with the line
        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, direction)
        axis_len = np.linalg.norm(axis)

        # If axis is not aligned, compute the quaternion rotation
        if axis_len > 1e-6:
            axis = axis / axis_len
            angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
            quat = np.zeros(4)
            mujoco.mju_axisAngle2Quat(quat, axis, angle)  # Compute quaternion
            rot_matrix = np.zeros((3, 3)).flatten()
            mujoco.mju_quat2Mat(rot_matrix, quat)
        else:
            rot_matrix = np.eye(3)

        # Render the capsule in the main renderer scene
        renderer_scene = self.renderer._scene
        renderer_maxgeom = getattr(renderer_scene, "maxgeom", None)
        if renderer_maxgeom is None:
            renderer_maxgeom = len(renderer_scene.geoms)
        if renderer_scene.ngeom < renderer_maxgeom:
            mujoco.mjv_initGeom(
                renderer_scene.geoms[renderer_scene.ngeom],
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[radius, length / 2, 0.0],  # Capsule size: [radius, half-length]
                pos=midpoint.flatten(),
                mat=rot_matrix.flatten(),
                rgba=color,
            )
            renderer_scene.ngeom += 1

        # Render the capsule in the viewer scene if a viewer is available
        if self.viewer:
            viewer_scene = self.viewer.user_scn
            viewer_maxgeom = getattr(viewer_scene, "maxgeom", None)
            if viewer_maxgeom is None:
                viewer_maxgeom = len(viewer_scene.geoms)
            if viewer_scene.ngeom < viewer_maxgeom:
                mujoco.mjv_initGeom(
                    viewer_scene.geoms[viewer_scene.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                    size=[radius, length / 2, 0.0],  # Capsule size: [radius, half-length]
                    pos=midpoint.flatten(),
                    mat=rot_matrix.flatten(),
                    rgba=color,
                )
                viewer_scene.ngeom += 1

    def render_pixel_line_segment(self, pos1, pos2, width=1.5, color=(1.0, 1.0, 1.0, 1.0)):
        """Render a lightweight screen-space line using MuJoCo's visual-only line geom."""

        if self.renderer is None:
            return

        pos1 = np.asarray(pos1, dtype=np.float64).reshape(3)
        pos2 = np.asarray(pos2, dtype=np.float64).reshape(3)
        if not (np.all(np.isfinite(pos1)) and np.all(np.isfinite(pos2))):
            return
        if np.linalg.norm(pos2 - pos1) <= 1e-9:
            return

        color = np.clip(np.asarray(color, dtype=np.float32).reshape(4), 0.0, 1.0)
        width = float(np.clip(width, 0.5, 10.0))

        def _render_to_scene(scene):
            maxgeom = getattr(scene, "maxgeom", None)
            if maxgeom is None:
                maxgeom = len(scene.geoms)
            if scene.ngeom >= maxgeom:
                return
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_LINE,
                np.zeros(3, dtype=np.float64),
                np.zeros(3, dtype=np.float64),
                np.eye(3, dtype=np.float64).reshape(-1),
                color,
            )
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, pos1, pos2)
            scene.ngeom += 1

        _render_to_scene(self.renderer._scene)
        if self.viewer:
            _render_to_scene(self.viewer.user_scn)

    def render_surface(self, points, color=(0.8, 0.3, 0.3, 1)):
        """Render a two-sided triangle surface without overrunning scene capacity."""

        def _append(scene, v1, dir_x, dir_y, dir_z):
            maxgeom = getattr(scene, "maxgeom", len(scene.geoms))
            if scene.ngeom + 2 > maxgeom:
                return False
            for mat in (
                np.vstack([dir_x, dir_y, dir_z]).T,
                np.vstack([dir_y, dir_x, -dir_z]).T,
            ):
                mujoco.mjv_initGeom(
                    scene.geoms[scene.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_TRIANGLE,
                    size=[1, 1, 1],
                    pos=v1,
                    mat=mat.reshape(-1),
                    rgba=color,
                )
                scene.ngeom += 1
            return True

        scenes = []
        if self.renderer is not None:
            scenes.append(self.renderer._scene)
        if self.viewer is not None:
            scenes.append(self.viewer.user_scn)
        for v1, v2, v3 in np.asarray(points, dtype=float).reshape(-1, 3, 3):
            dir_x = v2 - v1
            dir_y = v3 - v1
            normal = np.cross(dir_x, dir_y)
            norm = np.linalg.norm(normal)
            if norm <= 1.0e-12:
                continue
            dir_z = normal / norm
            for scene in scenes:
                _append(scene, v1, dir_x, dir_y, dir_z)

    def render_arrow(self, start, end, color):
        """Render an arrow from the start point to the end point."""

        if self.renderer is None:
            return

        start = np.asarray(start)
        end = np.asarray(end)

        # Compute arrow direction and length
        arrow_dir = end - start
        arrow_length = np.linalg.norm(arrow_dir)

        # If arrow length is too small, don't render
        if arrow_length < 1e-10:
            return

        # Normalize the arrow direction
        z = arrow_dir / arrow_length

        # Create an orthonormal basis for rotation matrix
        if abs(z[0]) < 0.9:
            tmp = np.array([1.0, 0.0, 0.0])
        else:
            tmp = np.array([0.0, 1.0, 0.0])

        y = np.cross(z, tmp)
        y /= np.linalg.norm(y)
        x = np.cross(y, z)
        mat = np.column_stack((x, y, z)).flatten()

        # Define arrow size (shaft radius, head radius, length)
        shaft_radius = 0.05 * arrow_length
        head_radius = 0.1 * arrow_length
        size = np.array([shaft_radius, head_radius, arrow_length])

        # Render the arrow in the main renderer scene
        mujoco.mjv_initGeom(
            self.renderer._scene.geoms[self.renderer._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_ARROW,
            size=size,
            pos=start,
            mat=mat,
            rgba=np.array(color),
        )
        self.renderer._scene.ngeom += 1

        # Render the arrow in the viewer scene if a viewer is available
        if self.viewer:
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_ARROW,
                size=size,
                pos=start,
                mat=mat,
                rgba=np.array(color),
            )
            self.viewer.user_scn.ngeom += 1

    def render_coordinate_frame(self, frame, size=0.1):
        """Render a coordinate frame at the given position and orientation."""

        origin = frame[:3, 3]
        x_axis = frame[:3, 0] * size
        y_axis = frame[:3, 1] * size
        z_axis = frame[:3, 2] * size

        # Render X axis (red)
        self.render_line_segment(origin, origin + x_axis, radius=0.001, color=VizColor.x_axis)
        # Render Y axis (green)
        self.render_line_segment(origin, origin + y_axis, radius=0.001, color=VizColor.y_axis)
        # Render Z axis (blue)
        self.render_line_segment(origin, origin + z_axis, radius=0.001, color=VizColor.z_axis)

    def _render_obstacle_debug(self):
        """Render obstacles for debugging purposes."""
        for i, (frame, geom) in enumerate(zip(self.obstacle_debug_frame, self.obstacle_debug_geom)):
            if geom.type == "sphere":
                self.render_sphere(
                    pos=frame[:3, 3],
                    mat=frame[:3, :3],
                    size=geom.attributes["radius"] * np.ones(3),
                    color=geom.color,
                )
                if i == self.obstacle_debug_selected:
                    self.render_arrow(
                        frame[:3, 3] + [0, 0, 0.15],
                        frame[:3, 3] + [0, 0, 0.01],
                        VizColor.obstacle_debug,
                    )
            elif geom.type == "box":
                self.render_box(
                    pos=frame[:3, 3],
                    mat=frame[:3, :3],
                    size=np.array(
                        [
                            geom.attributes["length"],
                            geom.attributes["width"],
                            geom.attributes["height"],
                        ]
                    ),
                    color=geom.color,
                )
                if i == self.obstacle_debug_selected:
                    self.render_arrow(
                        frame[:3, 3] + [0, 0, 0.15],
                        frame[:3, 3] + [0, 0, 0.01],
                        VizColor.obstacle_debug,
                    )
            else:
                raise ValueError(f"Unknown geometry type: {geom.type}")

    def _set_object_pos(self, object_name: str, object_T: np.ndarray) -> None:
        """
        Reset a free-joint object pose in the world.

        Args:
            object_name (str): name of the body
            object_T (np.ndarray): 4x4 homogeneous transform in world frame
                                (rotation + translation)
        """
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, object_name)
        if body_id == -1:
            raise ValueError(f"Object '{object_name}' not found in the model.")

        # ----------------------------------------
        # Find the free joint associated with body
        # ----------------------------------------
        jnt_adr = self.model.body_jntadr[body_id]
        if jnt_adr < 0:
            raise RuntimeError(
                f"Body '{object_name}' has no joint. Did you forget to add <freejoint/>?"
            )

        # qpos address for this joint
        qpos_adr = self.model.jnt_qposadr[jnt_adr]
        dof_adr = self.model.jnt_dofadr[jnt_adr]

        # ----------------------------------------
        # Position
        # ----------------------------------------
        self.data.qpos[qpos_adr : qpos_adr + 3] = object_T[:3, 3]

        # ----------------------------------------
        # Orientation (MuJoCo expects w,x,y,z)
        # ----------------------------------------
        rot_matrix = object_T[:3, :3]
        quat_xyzw = R.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        self.data.qpos[qpos_adr + 3 : qpos_adr + 7] = quat_wxyz

        # ----------------------------------------
        # Zero velocities (important!)
        # ----------------------------------------
        self.data.qvel[dof_adr : dof_adr + 6] = 0.0

        # ----------------------------------------
        # Recompute derived quantities
        # ----------------------------------------
        mujoco.mj_forward(self.model, self.data)

    def _has_collision(
        self,
        ignored_bodies: Set[int] = set(),
        *,
        dist_thresh: float = 0.0,  # count contacts with dist <= dist_thresh (0 => touching/penetration)
        require_penetration: bool = False,  # True => only dist < 0
        robot_root_body: Optional[int] = None,  # if set, classify by robot subtree
        detect_self_collision: bool = True,  # NEW: include robot self-collisions
        detect_robot_env_collision: bool = True,  # NEW: include robot-vs-env collisions
        include_positions: bool = False,
        include_forces: bool = False,
    ) -> Tuple[bool, List[Dict[str, Any]]]:
        """
        Returns:
        (has_extra_collision, pairs)

        `pairs` is a list of dicts for each "non-ignored" contact, containing:
        - type: "self" | "robot_env" | "unknown"
        - body1_id, body2_id, body1_name, body2_name
        - geom1_id, geom2_id, geom1_name, geom2_name
        - dist
        - (optional) pos_world
        - (optional) force6_contact_frame

        A contact is ignored if either contacting geom belongs to a body in `ignored_bodies`
        (based on model.geom_bodyid[geom]).

        If `robot_root_body` is provided, contacts are classified using robot subtree membership:
        - self: both geoms in robot subtree
        - robot_env: exactly one geom in robot subtree
        - env_env: neither in robot subtree (not returned)
        """
        mujoco.mj_forward(self.model, self.data)

        def in_robot_subtree(body_id: int) -> bool:
            if robot_root_body is None:
                return False
            # walk up parents until worldbody (0)
            while body_id != 0:
                if body_id == robot_root_body:
                    return True
                body_id = int(self.model.body_parentid[body_id])
            return body_id == robot_root_body

        pairs: List[Dict[str, Any]] = []

        for i in range(int(self.data.ncon)):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1, b2 = int(self.model.geom_bodyid[g1]), int(self.model.geom_bodyid[g2])

            # ignore if either side is in ignored bodies
            if (b1 in ignored_bodies) or (b2 in ignored_bodies):
                continue

            # contact distance filtering
            d = float(c.dist)
            if require_penetration:
                if d >= 0.0:
                    continue
            else:
                if d > dist_thresh:
                    continue

            # classification / filtering
            ctype = "unknown"
            if robot_root_body is not None:
                r1 = in_robot_subtree(b1)
                r2 = in_robot_subtree(b2)

                if r1 and r2:
                    ctype = "self"
                    if not detect_self_collision:
                        continue
                elif r1 ^ r2:
                    ctype = "robot_env"
                    if not detect_robot_env_collision:
                        continue
                else:
                    # env-env contact (usually irrelevant)
                    continue

            entry: Dict[str, Any] = {
                "contact_id": i,
                "dist": d,
                "type": ctype,
                "geom1_id": g1,
                "geom2_id": g2,
                "body1_id": b1,
                "body2_id": b2,
                "geom1_name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g1) or "",
                "geom2_name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g2) or "",
                "body1_name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b1) or "",
                "body2_name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b2) or "",
            }

            if include_positions:
                entry["pos_world"] = np.array(c.pos, dtype=np.float64)

            if include_forces:
                f6 = np.zeros(6, dtype=np.float64)
                mujoco.mj_contactForce(self.model, self.data, i, f6)
                entry["force6_contact_frame"] = f6

            pairs.append(entry)

        return (len(pairs) > 0), pairs

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

        # render camera views if enabled
        if self.enable_camera:
            self._capture_camera_feedback()

        # Sleep for the timestep duration when wall-clock simulation is requested.
        if self.real_time:
            time.sleep(self.model.opt.timestep)

    def _post_control_processing(self, **kwargs):
        """
        This method processes control feedback, updates obstacles, and steps the simulation forward.
        """
        num_obstacle_debug_change = self.num_obstacle_debug_change_buf
        self.num_obstacle_debug_change_buf -= (
            num_obstacle_debug_change  # Prevent missing keyboard input
        )

        # Add or remove obstacles based on debug feedback
        while num_obstacle_debug_change > 0:
            self._add_obstacle()
            num_obstacle_debug_change -= 1
        while num_obstacle_debug_change < 0:
            self._remove_obstacle()
            num_obstacle_debug_change += 1

        # Update obstacle velocity based on the change in their frames

        self.obstacle_debug_velocity = [
            np.hstack(
                (
                    (
                        self.obstacle_debug_frame[i][:3, 3]
                        - self.obstacle_debug_frame_last[i][:3, 3]
                    ),
                    np.zeros(3),
                )
            )
            for i in range(self.num_obstacle_debug)
        ]
        self.obstacle_debug_frame_last = self.obstacle_debug_frame.copy()

        # Step the simulation forward
        self._mujoco_step()

    def _pd_control(self, target_q, q, kp, target_dq, dq, kd):
        """Calculates torques from position commands using PD control."""

        # PD control formula to calculate torques
        # (target position - current position) * proportional gain
        # + (target velocity - current velocity) * derivative gain

        return (target_q - q) * kp + (target_dq - dq) * kd

    def _actuator_joint_id(self, actuator_id: int) -> Optional[int]:
        if actuator_id < 0 or actuator_id >= self.model.nu:
            return None
        if self.model.actuator_trntype[actuator_id] != mujoco.mjtTrn.mjTRN_JOINT:
            return None
        joint_id = int(self.model.actuator_trnid[actuator_id, 0])
        return joint_id if joint_id >= 0 else None

    def _configure_uncontrolled_actuator_holds(
        self,
        *,
        controlled_actuator_ids=(),
        excluded_actuator_ids=(),
        joint_names=None,
    ) -> None:
        controlled = {
            int(item) for item in controlled_actuator_ids if item is not None and int(item) >= 0
        }
        excluded = {
            int(item) for item in excluded_actuator_ids if item is not None and int(item) >= 0
        }
        joint_name_set = None if joint_names is None else {str(name) for name in joint_names}
        holds = []
        for actuator_id in range(self.model.nu):
            if actuator_id in controlled or actuator_id in excluded:
                continue
            joint_id = self._actuator_joint_id(actuator_id)
            if joint_id is None:
                continue
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if joint_name_set is not None and joint_name not in joint_name_set:
                continue
            qpos_adr = int(self.model.jnt_qposadr[joint_id])
            dof_adr = int(self.model.jnt_dofadr[joint_id])
            if qpos_adr < 0 or dof_adr < 0:
                continue
            holds.append(
                {
                    "actuator_id": actuator_id,
                    "joint_id": joint_id,
                    "qpos_adr": qpos_adr,
                    "dof_adr": dof_adr,
                    "target": float(self.data.qpos[qpos_adr]),
                }
            )
        self._uncontrolled_actuator_holds = holds

    def _refresh_uncontrolled_actuator_hold_targets(self) -> None:
        mujoco.mj_forward(self.model, self.data)
        for entry in self._uncontrolled_actuator_holds:
            entry["target"] = float(self.data.qpos[entry["qpos_adr"]])
            entry["bias_control"] = self._actuator_bias_control(entry["actuator_id"])
            self.model.qpos_spring[entry["qpos_adr"]] = entry["target"]
            self.model.jnt_stiffness[entry["joint_id"]] = max(
                float(self.model.jnt_stiffness[entry["joint_id"]]),
                self.uncontrolled_joint_stiffness,
            )
            self.model.dof_damping[entry["dof_adr"]] = max(
                float(self.model.dof_damping[entry["dof_adr"]]),
                self.uncontrolled_joint_damping,
            )
            self.model.dof_armature[entry["dof_adr"]] = max(
                float(self.model.dof_armature[entry["dof_adr"]]),
                self.uncontrolled_joint_armature,
            )

    def _apply_uncontrolled_actuator_holds(self, ctrl: np.ndarray) -> None:
        for entry in self._uncontrolled_actuator_holds:
            actuator_id = entry["actuator_id"]
            if actuator_id >= len(ctrl):
                continue
            q = float(self.data.qpos[entry["qpos_adr"]])
            dq = float(self.data.qvel[entry["dof_adr"]])
            ctrl[actuator_id] = self._clip_actuator_control(
                actuator_id,
                entry.get("bias_control", self._actuator_bias_control(actuator_id))
                + self.uncontrolled_actuator_hold_kp * (entry["target"] - q)
                - self.uncontrolled_actuator_hold_kd * dq,
            )

    def _actuator_bias_control(self, actuator_id: int) -> float:
        joint_id = self._actuator_joint_id(actuator_id)
        if joint_id is None:
            return 0.0
        dof_adr = int(self.model.jnt_dofadr[joint_id])
        if dof_adr < 0 or dof_adr >= self.model.nv:
            return 0.0
        gear = float(self.model.actuator_gear[actuator_id, 0])
        if abs(gear) <= 1e-9:
            return 0.0
        return float(self.data.qfrc_bias[dof_adr]) / gear

    def _clip_actuator_control(self, actuator_id: int, value: float) -> float:
        value = float(value)
        if not np.isfinite(value):
            return 0.0
        if 0 <= actuator_id < self.model.nu and bool(self.model.actuator_ctrllimited[actuator_id]):
            low, high = self.model.actuator_ctrlrange[actuator_id]
            return float(np.clip(value, low, high))
        limit = self.sim_ctrl_abs_limit
        if limit is None:
            return value
        limit = float(limit)
        if limit <= 0.0:
            return value
        return float(np.clip(value, -limit, limit))

    def _enforce_uncontrolled_actuator_holds(self) -> None:
        for entry in self._uncontrolled_actuator_holds:
            self.data.qpos[entry["qpos_adr"]] = entry["target"]
            self.data.qvel[entry["dof_adr"]] = 0.0
            self.data.qacc[entry["dof_adr"]] = 0.0

    def _clip_target_pos_to_mujoco_limits(self, target_pos: np.ndarray) -> np.ndarray:
        clipped = np.asarray(target_pos, dtype=np.float64).copy()
        if not hasattr(self.robot_cfg, "DoF_to_MujocoDoF"):
            return clipped
        for dof in self.robot_cfg.DoFs:
            if dof not in self.robot_cfg.DoF_to_MujocoDoF:
                continue
            mj_dof = int(self.robot_cfg.DoF_to_MujocoDoF[dof])
            if mj_dof < 0 or mj_dof >= self.model.nv:
                continue
            joint_id = int(self.model.dof_jntid[mj_dof])
            if joint_id < 0 or not bool(self.model.jnt_limited[joint_id]):
                continue
            low, high = self.model.jnt_range[joint_id]
            clipped[int(dof)] = np.clip(clipped[int(dof)], low, high)
        return clipped

    def _limit_target_pos_error(
        self, target_pos: np.ndarray, current_pos: np.ndarray
    ) -> np.ndarray:
        limit = self.sim_position_error_limit
        if limit is None:
            return np.asarray(target_pos, dtype=np.float64).copy()
        limit = float(limit)
        if limit <= 0.0:
            return np.asarray(target_pos, dtype=np.float64).copy()
        target = np.asarray(target_pos, dtype=np.float64)
        current = np.asarray(current_pos, dtype=np.float64)
        return current + np.clip(target - current, -limit, limit)

    def _compose_cmd_state(self):
        """
        Compose the command state representation based on DoF position and velocity commands.

        Returns:
            np.ndarray: Composed command state.
        """
        return self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd, self.dof_vel_cmd)

    def _compose_simulator_dynamics_state(self):
        """Return the measured state used by simulator-owned dynamics.

        Velocity or acceleration commands must be integrated from current
        simulator feedback. Integrating repeatedly from the previous target
        causes command-state windup whenever a physical joint servo lags.
        Model-owned execution intentionally continues from command state.
        """

        return self._compose_fbk_state()

    def _compose_fbk_state(self):
        """
        Compose the feedback state representation based on DoF position and velocity feedback.

        Returns:
            np.ndarray: Composed feedback state.
        """
        return self.robot_cfg.compose_state_from_dof(self.dof_pos_fbk, self.dof_vel_fbk)
