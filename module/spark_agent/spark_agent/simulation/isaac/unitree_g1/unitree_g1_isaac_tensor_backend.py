"""Batched Unitree G1 simulator agent built on Isaac Lab articulation views."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from spark_agent.base.base_agent import BaseAgent
from spark_agent.simulation.camera_display import (
    close_camera_display,
    submit_camera_display,
)
from spark_agent.simulation.isaac.isaac_agent import (
    ISAAC_COLLISION_VOLUME_OPACITY_FLOOR,
    ISAAC_COLLISION_VOLUME_OPACITY_SCALE,
    ISAAC_GOAL_OPACITY,
    IsaacAgent,
    _bind_debug_rgba_material,
    _disable_debug_prim_shadows,
    _look_at_quaternion_wxyz,
    _show_viewport_only,
    import_urdf_to_usd,
)
from spark_agent.simulation.isaac.viewer_info import IsaacViewerInfoMixin
from spark_agent.simulation.isaac.unitree_g1.unitree_g1_isaac_agent import (
    G1_29DOF_JOINT_NAMES,
    G1_VIRTUAL_JOINT_NAMES,
)
from spark_agent.simulation.viewer_config import (
    camera_eye_target,
    normalize_sensor_camera_config,
    normalize_viewer_config,
    sensor_camera_eye_target,
)
from spark_robot import SPARK_ROBOT_RESOURCE_DIR, RobotConfig
from spark_robot.unitree_g1.actuation import UNITREE_G1_GRIPPER_SPECS
from spark_utils import Geometry, VizColor


def _motor_to_joint_name() -> dict[str, str]:
    names = {
        "".join(part.capitalize() for part in name.removesuffix("_joint").split("_")): name
        for name in G1_29DOF_JOINT_NAMES
    }
    names.update(G1_VIRTUAL_JOINT_NAMES)
    return names


def _interactive_kit_visualizer_configs(render_enabled: bool, enable_viewer: bool):
    """Return the Kit app-loop owner for an interactive Isaac viewer.

    IsaacLab camera sensors otherwise call ``app.update()`` themselves while
    their output is read. That nested update can dispatch viewport keyboard
    or selection events in the middle of an RTX camera update. A
    ``KitVisualizer`` makes ``SimulationContext.render()`` the sole owner of
    the application update. Headless capture deliberately keeps IsaacLab's
    offscreen path: it has no interactive event source to race, and a
    headless Kit visualizer attempts to re-author the default viewport camera.
    """

    if not (render_enabled and enable_viewer):
        return []

    from isaaclab_visualizers.kit import KitVisualizerCfg

    return [
        KitVisualizerCfg(
            create_viewport=False,
            headless=False,
            enable_markers=False,
            enable_live_plots=False,
        )
    ]


class _UnitreeG1IsaacTensorBackend(IsaacViewerInfoMixin, BaseAgent):
    """Execute one tensorized G1 command across multiple Isaac environments.

    This class deliberately exposes tensor feedback. The existing
    :class:`UnitreeG1IsaacAgent` remains the scalar pipeline adapter, while
    this backend establishes the no-CPU-round-trip contract required for
    parallel Sport and WBT execution.
    """

    def __init__(
        self,
        robot_cfg: RobotConfig,
        *,
        num_envs: int = 1,
        env_spacing: float = 2.5,
        dt: float | None = None,
        control_decimation: int | None = None,
        device: str = "cuda:0",
        urdf_path: str | None = None,
        usd_path: str | None = None,
        fixed_base: bool = False,
        enable_hand_control: bool = False,
        render: bool = False,
        render_on_step: bool = True,
        render_decimation: int = 1,
        sim_stiffness_scale: float | None = None,
        sim_damping_scale: float | None = None,
        hand_kp: float | None = None,
        hand_kd: float | None = None,
        gravity_compensation_scale: float = 0.0,
        viewer_config: dict | None = None,
        joint_passive_damping: float = 0.001,
        joint_armature: float = 0.01,
        joint_friction: float = 0.1,
        solver_position_iteration_count: int = 8,
        solver_velocity_iteration_count: int = 4,
        allow_self_collision: bool = False,
        link_mass_scales: dict[str, float] | None = None,
        scalar_api: bool = False,
        native_implicit_pd: bool = False,
        hybrid_implicit_upper_body: bool = False,
        joint_stiffness: dict[str, float] | None = None,
        joint_damping: dict[str, float] | None = None,
        joint_armature_map: dict[str, float] | None = None,
        default_joint_positions: dict[str, float] | None = None,
        **kwargs,
    ) -> None:
        super().__init__(robot_cfg)
        if num_envs < 1:
            raise ValueError("num_envs must be positive")

        # Isaac Lab and Isaac Sim imports must occur after SimulationApp starts.
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.sim import SimulationContext
        from isaaclab.utils.configclass import configclass

        self._torch = torch
        self.num_envs = int(num_envs)
        self.env_spacing = float(env_spacing)
        timing = robot_cfg.simulator_dynamics
        self.dt = float(timing.physics_dt if dt is None else dt)
        self.control_decimation = max(
            1,
            int(timing.control_decimation if control_decimation is None else control_decimation),
        )
        self.device = str(device)
        self.fixed_base = bool(fixed_base)
        self.enable_hand_control = bool(enable_hand_control)
        self.solver_position_iteration_count = int(solver_position_iteration_count)
        self.solver_velocity_iteration_count = int(solver_velocity_iteration_count)
        if self.solver_position_iteration_count < 1:
            raise ValueError("solver_position_iteration_count must be positive")
        if self.solver_velocity_iteration_count < 0:
            raise ValueError("solver_velocity_iteration_count cannot be negative")
        self.allow_self_collision = bool(allow_self_collision)
        self.link_mass_scales = {
            str(token): float(scale) for token, scale in (link_mass_scales or {}).items()
        }
        if any(not token or scale <= 0.0 for token, scale in self.link_mass_scales.items()):
            raise ValueError("link mass-scale tokens must be nonempty and scales positive")
        self.render_enabled = bool(render)
        self._initialize_viewer_simulation_info(**kwargs)
        self.enable_viewer = bool(kwargs.get("enable_viewer", render))
        self.enable_keyboard_control = bool(kwargs.get("enable_keyboard_control", False))
        self.render_robot_collision_volumes = bool(
            kwargs.get("render_robot_collision_volumes", True)
        )
        self.collision_volume_opacity_scale = float(
            kwargs.get(
                "collision_volume_opacity_scale",
                ISAAC_COLLISION_VOLUME_OPACITY_SCALE,
            )
        )
        if not 0.0 <= self.collision_volume_opacity_scale <= 1.0:
            raise ValueError("collision_volume_opacity_scale must be between 0 and 1")
        self.collision_volume_opacity_floor = float(
            kwargs.get(
                "collision_volume_opacity_floor",
                ISAAC_COLLISION_VOLUME_OPACITY_FLOOR,
            )
        )
        if not 0.0 <= self.collision_volume_opacity_floor <= 1.0:
            raise ValueError("collision_volume_opacity_floor must be between 0 and 1")
        self.render_decimation = max(1, int(render_decimation))
        self.render_on_step = bool(render_on_step) and not bool(kwargs.get("defer_render", False))
        self.defer_render = not self.render_on_step
        self.viewer_config = normalize_viewer_config(viewer_config)
        self.camera_eye, self.camera_target = camera_eye_target(self.viewer_config)
        self.camera_diagnostics = None
        self.enable_camera = bool(kwargs.get("enable_camera", False))
        self.camera_width = int(kwargs.get("camera_width", 640))
        self.camera_height = int(kwargs.get("camera_height", 480))
        self.camera_rate_hz = float(kwargs.get("camera_rate_hz", 30.0))
        self.camera_display = bool(kwargs.get("camera_display", False))
        self.camera_config = normalize_sensor_camera_config(
            kwargs.get("camera_config"), self.viewer_config
        )
        self.camera_feedback = {}
        self._isaaclab_sensor_cameras = {}
        self._camera_display_process = None
        self._camera_ready_reported = False
        self._last_camera_capture_time = -np.inf
        if self.enable_camera:
            # Camera submission is owned by render_frame(), where articulation
            # state is protected from Kit's autonomous RTX app-loop updates.
            self.render_on_step = False
            self.defer_render = True
        self._viewport_layout_updates_remaining = int(
            kwargs.get("viewport_layout_updates", 6 if self.render_enabled else 0)
        )
        self.gravity_compensation_scale = float(gravity_compensation_scale)
        self.scalar_api = bool(scalar_api)
        if self.scalar_api and self.num_envs != 1:
            raise ValueError("scalar_api requires num_envs=1")
        self.native_implicit_pd = bool(native_implicit_pd)
        self.hybrid_implicit_upper_body = bool(hybrid_implicit_upper_body)
        if self.native_implicit_pd and self.hybrid_implicit_upper_body:
            raise ValueError(
                "native_implicit_pd and hybrid_implicit_upper_body are mutually exclusive"
            )
        self._simulation_app = None
        self._renderer_process = None
        self.renderer_process_status = None
        self._debug_primitives = []
        self._debug_prim_count = 0
        self._debug_solid_cache = {"sphere": [], "box": []}
        self._debug_root = "/World/SparkPipelineDebug"
        obstacle_debug = dict(kwargs.get("obstacle_debug") or {})
        self.num_obstacle_debug = int(obstacle_debug.get("num_obstacle", 0))
        self.obstacle_debug_frame = np.repeat(
            np.eye(4, dtype=float)[None], self.num_obstacle_debug, axis=0
        )
        for frame in self.obstacle_debug_frame:
            frame[:3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
        self.obstacle_debug_geom = [
            Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug)
            for _ in range(self.num_obstacle_debug)
        ]
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.obstacle_debug_selected = 0 if self.num_obstacle_debug else None
        self.left_goal_debug_frame = np.eye(4, dtype=float)
        self.right_goal_debug_frame = np.eye(4, dtype=float)
        self.base_goal_debug_frame = np.eye(4, dtype=float)
        self.debug_object = self.base_goal_debug_frame
        self.translate_in_local_frame = False
        self.manual_step_size = float(obstacle_debug.get("manual_movement_step_size", 0.02))
        self.manual_step_size_min = float(
            obstacle_debug.get("manual_movement_step_size_min", 0.001)
        )
        self.manual_step_size_max = float(obstacle_debug.get("manual_movement_step_size_max", 0.5))
        self.manual_step_size_delta = float(
            obstacle_debug.get("manual_movement_step_size_delta", 0.01)
        )
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self._keyboard_input = None
        self._keyboard = None
        self._keyboard_subscription = None
        self.step_index = 0
        self.time = 0.0

        configured_urdf_path = getattr(robot_cfg, "isaac_model_path", None)
        if configured_urdf_path is None:
            configured_mjcf_path = Path(robot_cfg.mujoco_model_path)
            configured_urdf_path = Path("unitree_g1") / "urdf" / f"{configured_mjcf_path.stem}.urdf"
        source = Path(urdf_path or configured_urdf_path).expanduser()
        if not source.is_absolute():
            source = Path(SPARK_ROBOT_RESOURCE_DIR) / source
        source = source.resolve()
        resolved_usd = (
            import_urdf_to_usd(
                source,
                fix_base=self.fixed_base,
                allow_self_collision=self.allow_self_collision,
            )
            if usd_path is None
            else str(usd_path)
        )
        print(f"[SPARK] Isaac robot metadata: {source}", flush=True)
        print(f"[SPARK] Isaac robot plant: {resolved_usd}", flush=True)
        print(
            "[SPARK] Isaac physical self-contact: "
            f"{'enabled' if self.allow_self_collision else 'disabled'}",
            flush=True,
        )
        urdf_root = ET.parse(source)

        visualizer_cfgs = _interactive_kit_visualizer_configs(
            self.render_enabled,
            self.enable_viewer,
        )
        sim_cfg = sim_utils.SimulationCfg(
            dt=self.dt,
            device=self.device,
            render_interval=self.control_decimation,
            visualizer_cfgs=visualizer_cfgs,
        )
        self.sim = SimulationContext(sim_cfg)

        spawn_height = float(kwargs.get("spawn_height", 0.793))
        # Isaac Lab 3 rejects overlapping regex assignments.  Use an exact
        # entry for every movable URDF joint so selected-body and hand defaults
        # can be overridden without also matching a catch-all expression.
        init_joint_pos = {
            joint.get("name"): 0.0
            for joint in urdf_root.findall(".//joint")
            if joint.get("type") != "fixed"
        }
        urdf_joint_names = set(init_joint_pos)
        motor_name_map = _motor_to_joint_name()
        selected_joint_names = []
        selected_defaults = []
        selected_model_dof_ids = []
        for motor in robot_cfg.MujocoMotors:
            try:
                joint_name = motor_name_map[motor.name]
            except KeyError as exc:
                raise ValueError(f"Unsupported G1 motor for Isaac Lab: {motor.name}") from exc
            if joint_name in G1_VIRTUAL_JOINT_NAMES.values():
                continue
            dof = robot_cfg.MujocoDoF_to_DoF[getattr(robot_cfg.MujocoDoFs, motor.name)]
            value = float(robot_cfg.DefaultDoFVal[dof])
            if default_joint_positions:
                value = float(default_joint_positions.get(joint_name, value))
            selected_joint_names.append(joint_name)
            selected_defaults.append(value)
            selected_model_dof_ids.append(int(dof))
            init_joint_pos[joint_name] = value
        for spec in UNITREE_G1_GRIPPER_SPECS.values():
            init_joint_pos.update(
                {
                    name: value
                    for name, value in zip(spec.joint_names, spec.open_position)
                    if name in urdf_joint_names
                }
            )

        resolved_hand_kp = float(
            UNITREE_G1_GRIPPER_SPECS["left"].kp if hand_kp is None else hand_kp
        )
        resolved_hand_kd = float(
            UNITREE_G1_GRIPPER_SPECS["left"].kd if hand_kd is None else hand_kd
        )
        implicit_joint_stiffness = dict(joint_stiffness or {})
        implicit_joint_damping = dict(joint_damping or {})
        if (
            self.native_implicit_pd or self.hybrid_implicit_upper_body
        ) and self.enable_hand_control:
            for spec in UNITREE_G1_GRIPPER_SPECS.values():
                for name in spec.joint_names:
                    if name in urdf_joint_names:
                        implicit_joint_stiffness.setdefault(name, resolved_hand_kp)
                        implicit_joint_damping.setdefault(name, resolved_hand_kd)

        actuator_stiffness = 0.0
        actuator_damping = float(joint_passive_damping)
        actuator_armature = float(joint_armature)
        if self.native_implicit_pd or self.hybrid_implicit_upper_body:
            actuator_stiffness = {
                name: float(implicit_joint_stiffness.get(name, 0.0)) for name in init_joint_pos
            }
            actuator_damping = {
                name: float(implicit_joint_damping.get(name, joint_passive_damping))
                for name in init_joint_pos
            }
            actuator_armature = {
                name: float((joint_armature_map or {}).get(name, joint_armature))
                for name in init_joint_pos
            }

        @configclass
        class G1ParallelSceneCfg(InteractiveSceneCfg):
            ground = AssetBaseCfg(
                prim_path="/World/defaultGroundPlane",
                spawn=sim_utils.GroundPlaneCfg(
                    color=self.viewer_config["ground_color"],
                    physics_material=sim_utils.RigidBodyMaterialCfg(
                        static_friction=1.0,
                        dynamic_friction=1.0,
                        restitution=0.0,
                        friction_combine_mode="max",
                    ),
                ),
            )
            robot = ArticulationCfg(
                prim_path="{ENV_REGEX_NS}/Robot",
                spawn=sim_utils.UsdFileCfg(
                    usd_path=resolved_usd,
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(
                        disable_gravity=False,
                        retain_accelerations=False,
                        linear_damping=0.0,
                        angular_damping=0.0,
                        max_linear_velocity=1000.0,
                        max_angular_velocity=1000.0,
                        max_depenetration_velocity=1.0,
                    ),
                    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                        enabled_self_collisions=self.allow_self_collision,
                        # The URDF importer has already authored the fixed root
                        # when requested. Asking Isaac Lab to create a second
                        # fixed joint fails for imported assets whose
                        # articulation root is a non-rigid container prim.
                        fix_root_link=False,
                        solver_position_iteration_count=self.solver_position_iteration_count,
                        solver_velocity_iteration_count=self.solver_velocity_iteration_count,
                    ),
                ),
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=(0.0, 0.0, spawn_height),
                    joint_pos=init_joint_pos,
                    joint_vel={".*": 0.0},
                ),
                actuators={
                    "all": ImplicitActuatorCfg(
                        joint_names_expr=[".*"],
                        stiffness=actuator_stiffness,
                        damping=actuator_damping,
                        armature=actuator_armature,
                        friction=float(joint_friction),
                    )
                },
            )

        self.scene = InteractiveScene(
            G1ParallelSceneCfg(num_envs=self.num_envs, env_spacing=self.env_spacing)
        )
        self.robot = self.scene["robot"]
        if self.enable_camera:
            self._initialize_isaaclab_sensor_cameras()
        self.sim.reset()
        self.scene.update(self.dt)
        self._apply_link_mass_scales()
        IsaacAgent._apply_mujoco_scene_style(
            self,
            self.camera_eye,
            self.camera_target,
            vertical_fov=self.viewer_config["camera_vertical_fov"],
            viewer_config=self.viewer_config,
        )
        IsaacAgent._hide_imported_guide_geometry("/World/envs")
        IsaacAgent._apply_mujoco_robot_material_style("/World/envs")

        self.joint_names = tuple(self.robot.joint_names)
        missing = [name for name in selected_joint_names if name not in self.joint_names]
        if missing:
            raise ValueError(f"Isaac Lab articulation is missing configured joints: {missing}")
        self.body_joint_ids = self._torch.as_tensor(
            [self.joint_names.index(name) for name in selected_joint_names],
            device=self.device,
            dtype=self._torch.int32,
        )
        self.body_joint_names = tuple(selected_joint_names)
        self.body_model_dof_ids = tuple(selected_model_dof_ids)
        self.num_actuated = len(selected_joint_names)
        self.default_body_pos = self._torch.as_tensor(
            selected_defaults, device=self.device, dtype=self._torch.float32
        ).repeat(self.num_envs, 1)
        self.body_kp = self._torch.as_tensor(
            [
                robot_cfg.MujocoMotorKps[m]
                for m in robot_cfg.MujocoMotors
                if motor_name_map[m.name] not in G1_VIRTUAL_JOINT_NAMES.values()
            ],
            device=self.device,
            dtype=self._torch.float32,
        )
        self.body_kd = self._torch.as_tensor(
            [
                robot_cfg.MujocoMotorKds[m]
                for m in robot_cfg.MujocoMotors
                if motor_name_map[m.name] not in G1_VIRTUAL_JOINT_NAMES.values()
            ],
            device=self.device,
            dtype=self._torch.float32,
        )
        self.body_kp *= float(
            getattr(robot_cfg, "IsaacSimStiffnessScale", 1.0)
            if sim_stiffness_scale is None
            else sim_stiffness_scale
        )
        self.body_kd *= float(
            getattr(robot_cfg, "IsaacSimDampingScale", 2.0)
            if sim_damping_scale is None
            else sim_damping_scale
        )
        self.control_body_kp = self.body_kp.expand(self.num_envs, -1).clone()
        self.control_body_kd = self.body_kd.expand(self.num_envs, -1).clone()

        limits = {joint.get("name"): joint.find("limit") for joint in urdf_root.findall(".//joint")}
        self.body_effort_limit = self._torch.as_tensor(
            [
                abs(float(limits[name].get("effort", "inf")))
                if limits.get(name) is not None
                else np.inf
                for name in self.body_joint_names
            ],
            device=self.device,
            dtype=self._torch.float32,
        )
        self.target_body_pos = self.default_body_pos.clone()
        self.target_body_vel = self._torch.zeros_like(self.target_body_pos)
        self.feedforward_torque = self._torch.zeros_like(self.target_body_pos)
        self.last_control = self._torch.zeros_like(self.target_body_pos)

        self.hand_kp = resolved_hand_kp
        self.hand_kd = resolved_hand_kd
        self.gripper_joint_ids = {}
        self.gripper_targets = {}
        self.gripper_effort_limits = {}
        for side, spec in UNITREE_G1_GRIPPER_SPECS.items():
            # A body-only robot contract need not contain hand joints. Resolve
            # the optional gripper solely from the selected local asset, and
            # lock every unselected movable joint below.
            if not self.enable_hand_control:
                continue
            if not all(name in self.joint_names for name in spec.joint_names):
                continue
            self.gripper_joint_ids[side] = self._torch.as_tensor(
                [self.joint_names.index(name) for name in spec.joint_names],
                device=self.device,
                dtype=self._torch.int32,
            )
            self.gripper_targets[side] = self._torch.as_tensor(
                spec.open_position, device=self.device, dtype=self._torch.float32
            ).repeat(self.num_envs, 1)
            self.gripper_effort_limits[side] = self._torch.as_tensor(
                [
                    abs(float(limits[name].get("effort", "inf")))
                    if limits.get(name) is not None
                    else np.inf
                    for name in spec.joint_names
                ],
                device=self.device,
                dtype=self._torch.float32,
            )

        body_names = set(self.body_joint_names)
        hand_names = (
            {name for spec in UNITREE_G1_GRIPPER_SPECS.values() for name in spec.joint_names}
            if self.enable_hand_control
            else set()
        )
        locked_names = [
            name for name in self.joint_names if name not in body_names and name not in hand_names
        ]
        self.locked_joint_ids = self._torch.as_tensor(
            [self.joint_names.index(name) for name in locked_names],
            device=self.device,
            dtype=self._torch.int32,
        )
        self.locked_joint_targets = (
            self.robot.data.default_joint_pos.torch[:, self.locked_joint_ids].clone()
            if locked_names
            else None
        )
        # Match the neutral Isaac tensor-agent contract so fixed/mobile/arm
        # Unitree configurations can use the same batched benchmark runtime as
        # every other robot. Unitree keeps its specialized PhysX realization;
        # these aliases expose only the common selected-DoF view.
        self.joint_indices = self.body_joint_ids.to(dtype=self._torch.long)
        self.env_positions = self.scene.env_origins
        self.model_dof_count = len(robot_cfg.DoFs)
        self.body_model_dof_ids_tensor = self._torch.as_tensor(
            self.body_model_dof_ids,
            device=self.device,
            dtype=self._torch.long,
        )
        self.default_dof_pos = self._torch.as_tensor(
            [float(robot_cfg.DefaultDoFVal[dof]) for dof in robot_cfg.DoFs],
            device=self.device,
            dtype=self._torch.float32,
        ).repeat(self.num_envs, 1)
        self.default_all_joint_pos = self.robot.data.default_joint_pos.torch.clone()
        self.control_to_dof = []
        for control in robot_cfg.Control:
            control_name = control.name
            if control_name.startswith(("v", "a")):
                dof_name = control_name[1:]
                if dof_name in robot_cfg.DoFs.__members__:
                    self.control_to_dof.append((int(control), int(robot_cfg.DoFs[dof_name])))
        self._model_command_velocity = self._torch.zeros_like(self.default_dof_pos)
        self.reset()
        if self.render_enabled and self.enable_viewer:
            # Complete all lazy RTX camera, viewport, and presentation-layout
            # setup before accepting keyboard input. Mutating the Workspace
            # from the steady-state render loop can invalidate Kit's native
            # window callback iteration when Q/E arrives during startup.
            self.render_frame()
            _show_viewport_only()
            self.sim.render()
            self._viewport_layout_updates_remaining = 0
        if self.enable_keyboard_control:
            self._subscribe_keyboard()

    def _apply_link_mass_scales(self) -> None:
        """Apply policy-deployment mass adaptations through the tensor view.

        Using the articulation view keeps the operation identical across one
        and many cloned environments and avoids authoring per-clone USD
        overrides. OpenWBT, for example, halves every wrist-link mass before
        executing its WBT policies in Isaac Sim.
        """
        if not self.link_mass_scales:
            return
        physics_view = getattr(self.robot, "root_physx_view", None)
        if physics_view is None:
            raise RuntimeError("Isaac articulation does not expose a PhysX mass view")
        raw_masses = physics_view.get_masses()
        if hasattr(raw_masses, "torch"):
            masses = raw_masses.torch.clone()
        elif hasattr(raw_masses, "__dlpack__"):
            masses = self._torch.utils.dlpack.from_dlpack(raw_masses).clone()
        else:
            masses = self._torch.as_tensor(
                np.asarray(raw_masses), device=self.device, dtype=self._torch.float32
            ).clone()
        matched = []
        for body_index, body_name in enumerate(self.robot.body_names):
            scales = [scale for token, scale in self.link_mass_scales.items() if token in body_name]
            if not scales:
                continue
            scale = float(np.prod(scales))
            masses[:, body_index] *= scale
            matched.append((body_name, scale))
        if not matched:
            raise ValueError(
                f"No Isaac body names matched link_mass_scales: {tuple(self.link_mass_scales)}"
            )
        # PhysX keeps articulation indices on CPU even when body properties
        # live on CUDA.
        env_ids = self._torch.arange(self.num_envs, device="cpu", dtype=self._torch.int32)
        if hasattr(raw_masses, "__dlpack__") and not hasattr(raw_masses, "torch"):
            # Isaac 6's Warp-backed tensor view requires Warp arrays.
            import warp as wp

            physics_view.set_masses(
                wp.from_torch(masses.contiguous()),
                wp.from_torch(env_ids),
            )
        else:
            physics_view.set_masses(masses.contiguous(), env_ids)
        print(
            "[SPARK] Isaac link mass scales: "
            + ", ".join(f"{name}={scale:g}" for name, scale in matched),
            flush=True,
        )

    def _initialize_isaaclab_sensor_cameras(self) -> None:
        """Create native IsaacLab RGB-D sensors for the WBT tensor runtime."""

        from scipy.spatial.transform import Rotation

        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg

        horizontal_aperture = 20.955
        vertical_aperture = horizontal_aperture * self.camera_height / self.camera_width
        for name, raw_config in self.camera_config.items():
            config = dict(raw_config)
            camera_type = str(config.get("type", "free")).lower()
            safe_name = "".join(character if character.isalnum() else "_" for character in name)
            if camera_type == "fixed":
                body_name = config.get("body_name")
                if not body_name:
                    print(f"[camera] IsaacLab camera {name!r} has no body_name; skipping")
                    continue
                prim_path = f"/World/envs/env_0/Robot/{body_name}/SparkCamera_{safe_name}"
                position = tuple(float(value) for value in config.get("pos", (0.0, 0.0, 0.0)))
                xyaxes = np.asarray(config.get("xyaxes", [1, 0, 0, 0, 1, 0]), dtype=float)
                x_axis, y_axis = xyaxes[:3], xyaxes[3:6]
                x_axis /= max(float(np.linalg.norm(x_axis)), 1.0e-12)
                y_axis /= max(float(np.linalg.norm(y_axis)), 1.0e-12)
                rotation = np.column_stack((x_axis, y_axis, np.cross(x_axis, y_axis)))
                quaternion_xyzw = Rotation.from_matrix(rotation).as_quat()
            else:
                prim_path = f"/World/SparkCamera_{safe_name}"
                eye, target = sensor_camera_eye_target(config)
                position = tuple(float(value) for value in eye)
                quaternion_xyzw = np.roll(_look_at_quaternion_wxyz(eye, target), -1)

            fovy = float(config.get("fovy", 58.0))
            focal_length = vertical_aperture / (2.0 * np.tan(0.5 * np.deg2rad(fovy)))
            clipping_range = tuple(
                float(value) for value in config.get("clipping_range", (0.03, 10.0))
            )
            sensor = Camera(
                CameraCfg(
                    prim_path=prim_path,
                    update_period=0.0,
                    height=self.camera_height,
                    width=self.camera_width,
                    data_types=["rgb", "distance_to_image_plane"],
                    offset=CameraCfg.OffsetCfg(
                        pos=position,
                        rot=tuple(float(value) for value in quaternion_xyzw),
                        convention="opengl",
                    ),
                    spawn=sim_utils.PinholeCameraCfg(
                        focal_length=float(focal_length),
                        horizontal_aperture=horizontal_aperture,
                        vertical_aperture=float(vertical_aperture),
                        clipping_range=clipping_range,
                    ),
                )
            )
            self._isaaclab_sensor_cameras[name] = (sensor, config)

    @staticmethod
    def _camera_output_numpy(value):
        tensor = value.torch if hasattr(value, "torch") else value
        if hasattr(tensor, "detach"):
            tensor = tensor.detach().cpu().numpy()
        array = np.asarray(tensor)
        return array[0] if array.ndim >= 4 else array

    def _capture_isaaclab_camera_feedback(self) -> None:
        if not self._isaaclab_sensor_cameras:
            return
        import time

        now = time.perf_counter()
        period = 0.0 if self.camera_rate_hz <= 0.0 else 1.0 / self.camera_rate_hz
        if now - self._last_camera_capture_time < period:
            return
        feedback = {}
        for name, (camera, config) in self._isaaclab_sensor_cameras.items():
            camera.update(self.dt * self.control_decimation, force_recompute=True)
            rgb = self._camera_output_numpy(camera.data.output["rgb"])[..., :3].copy()
            depth = self._camera_output_numpy(camera.data.output["distance_to_image_plane"]).astype(
                np.float32, copy=True
            )
            if depth.ndim == 3 and depth.shape[-1] == 1:
                depth = depth[..., 0]
            feedback[name] = {
                "rgb": rgb,
                "depth": depth,
                "frame_id": config.get("frame_id", name),
            }
        self.camera_feedback = feedback
        if self.camera_display and feedback:
            if not submit_camera_display(self, feedback):
                self.camera_display = False
                print("[camera] OpenCV display process unavailable; disabling camera display")
        if feedback and not self._camera_ready_reported:
            shapes = ", ".join(
                f"{name}: rgb={data['rgb'].shape}, depth={data['depth'].shape}"
                for name, data in feedback.items()
            )
            print(f"[camera] IsaacLab RGB-D stream ready ({shapes})")
            self._camera_ready_reported = True
        self._last_camera_capture_time = now

    def _subscribe_keyboard(self) -> None:
        """Connect SPARK goal manipulation to the in-process Isaac viewport."""
        import carb
        import omni.appwindow

        app_window = omni.appwindow.get_default_app_window()
        if app_window is None:
            print("Isaac keyboard teleop unavailable: no application window", flush=True)
            return
        self._keyboard_input = carb.input.acquire_input_interface()
        self._keyboard = app_window.get_keyboard()
        self._keyboard_subscription = self._keyboard_input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )
        print(
            "Isaac keyboard teleop: O/P/B select right/left/base goal; "
            "Space selects obstacle; arrows+Q/E translate; 2-7 rotate; "
            "PageUp/PageDown add/remove obstacle; N toggles local/world; "
            "V toggles collision volumes; - and + adjust movement step.",
            flush=True,
        )

    def _on_keyboard_event(self, event, *args):
        import carb
        from scipy.spatial.transform import Rotation

        if event.type not in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_REPEAT,
        ):
            return True
        key = event.input
        keyboard = carb.input.KeyboardInput
        if key == keyboard.SPACE:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self._cycle_obstacle_debug_object()
            return False
        step = self.manual_step_size
        translations = {
            keyboard.RIGHT: (0.0, step, 0.0),
            keyboard.LEFT: (0.0, -step, 0.0),
            keyboard.UP: (-step, 0.0, 0.0),
            keyboard.DOWN: (step, 0.0, 0.0),
            keyboard.E: (0.0, 0.0, step),
            keyboard.Q: (0.0, 0.0, -step),
        }
        rotations = {
            keyboard.KEY_2: (0.0, 0.0, 0.1),
            keyboard.KEY_3: (0.0, 0.0, -0.1),
            keyboard.KEY_4: (0.0, 0.1, 0.0),
            keyboard.KEY_5: (0.0, -0.1, 0.0),
            keyboard.KEY_6: (0.1, 0.0, 0.0),
            keyboard.KEY_7: (-0.1, 0.0, 0.0),
        }
        if key in translations:
            delta = np.asarray(translations[key], dtype=float)
            if self.translate_in_local_frame:
                delta = self.debug_object[:3, :3] @ delta
            self.debug_object[:3, 3] += delta
        elif key in rotations:
            self.debug_object[:3, :3] @= Rotation.from_euler("xyz", rotations[key]).as_matrix()
        elif key == keyboard.O:
            self.debug_object = self.right_goal_debug_frame
            print("Isaac teleop selected right-hand goal", flush=True)
        elif key == keyboard.P:
            self.debug_object = self.left_goal_debug_frame
            print("Isaac teleop selected left-hand goal", flush=True)
        elif key == keyboard.B:
            self.debug_object = self.base_goal_debug_frame
            print("Isaac teleop selected base goal", flush=True)
        elif key == keyboard.PAGE_UP:
            self._add_obstacle()
        elif key == keyboard.PAGE_DOWN:
            self._remove_obstacle()
        elif key == keyboard.N:
            self.translate_in_local_frame = not self.translate_in_local_frame
            print(
                "Isaac teleop translation frame: "
                f"{'LOCAL' if self.translate_in_local_frame else 'WORLD'}",
                flush=True,
            )
        elif key == keyboard.V:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self.render_robot_collision_volumes = not self.render_robot_collision_volumes
                self._set_collision_volume_visibility(self.render_robot_collision_volumes)
                print(
                    "Isaac robot collision volumes: "
                    f"{'ON' if self.render_robot_collision_volumes else 'OFF'}",
                    flush=True,
                )
            return False
        elif key in tuple(
            value
            for value in (
                keyboard.KEY_8,
                getattr(keyboard, "RIGHT_BRACKET", None),
            )
            if value is not None
        ):
            self._set_selected_gripper_state(True)
        elif key in tuple(
            value
            for value in (
                keyboard.KEY_9,
                getattr(keyboard, "LEFT_BRACKET", None),
            )
            if value is not None
        ):
            self._set_selected_gripper_state(False)
        elif key in tuple(
            value
            for value in (
                getattr(keyboard, "MINUS", None),
                getattr(keyboard, "NUMPAD_SUBTRACT", None),
            )
            if value is not None
        ):
            self._adjust_manual_step_size(-self.manual_step_size_delta)
        elif key in tuple(
            value
            for value in (
                getattr(keyboard, "EQUAL", None),
                getattr(keyboard, "NUMPAD_ADD", None),
            )
            if value is not None
        ):
            self._adjust_manual_step_size(self.manual_step_size_delta)
        return True

    def _adjust_manual_step_size(self, delta) -> None:
        self.manual_step_size = float(
            np.clip(
                self.manual_step_size + float(delta),
                self.manual_step_size_min,
                self.manual_step_size_max,
            )
        )
        print(
            f"[debug] manual movement step size = {self.manual_step_size:.4f} m",
            flush=True,
        )

    def _add_obstacle(self) -> None:
        frame = np.eye(4, dtype=float)
        frame[:3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
        self.obstacle_debug_frame = np.concatenate((self.obstacle_debug_frame, frame[None]), axis=0)
        self.obstacle_debug_geom.append(
            Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug)
        )
        self.num_obstacle_debug += 1
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.obstacle_debug_selected = self.num_obstacle_debug - 1
        self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]

    def _remove_obstacle(self) -> None:
        if not self.num_obstacle_debug:
            return
        index = self.obstacle_debug_selected or 0
        self.obstacle_debug_frame = np.delete(self.obstacle_debug_frame, index, axis=0)
        del self.obstacle_debug_geom[index]
        self.num_obstacle_debug -= 1
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.obstacle_debug_selected = 0 if self.num_obstacle_debug else None
        self.debug_object = (
            self.obstacle_debug_frame[self.obstacle_debug_selected]
            if self.num_obstacle_debug
            else self.base_goal_debug_frame
        )

    def _cycle_obstacle_debug_object(self) -> None:
        if not self.num_obstacle_debug:
            return
        current = -1 if self.obstacle_debug_selected is None else self.obstacle_debug_selected
        self.obstacle_debug_selected = (current + 1) % self.num_obstacle_debug
        self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]

    def _set_selected_gripper_state(self, closed) -> None:
        available_sides = tuple(
            side for side in ("left", "right") if side in self.gripper_joint_ids
        )
        if self.debug_object is self.right_goal_debug_frame:
            sides = ("right",)
        elif self.debug_object is self.left_goal_debug_frame:
            sides = ("left",)
        elif len(available_sides) == 1:
            sides = available_sides
        else:
            print(
                "[gripper] Select the right hand with O or the left hand with P first",
                flush=True,
            )
            return
        for side in sides:
            if side not in available_sides:
                print(f"[gripper] The selected robot has no {side} gripper", flush=True)
                continue
            setattr(self, f"{side}_gripper_goal", bool(closed))
            setattr(self, f"{side}_gripper_debug_state", bool(closed))

    def set_visual_obstacles(self, local_positions, radius: float = 0.05, color=None) -> None:
        """Add non-physical benchmark markers to cloned environments.

        Parallel safety/collision tensors are intentionally a later stage;
        these markers keep benchmark layout and rendering testable without
        silently introducing a CPU collision path.
        """
        if not self.render_enabled:
            return
        import omni.usd
        from pxr import Gf, UsdGeom, Vt

        positions = np.asarray(local_positions, dtype=float)
        if positions.ndim == 2:
            positions = np.broadcast_to(positions[None, :, :], (self.num_envs, *positions.shape))
        if positions.ndim != 3 or positions.shape[0] != self.num_envs or positions.shape[2] != 3:
            raise ValueError(
                "local_positions must have shape (num_obstacles, 3) or "
                f"({self.num_envs}, num_obstacles, 3)"
            )
        stage = omni.usd.get_context().get_stage()
        origins = self.scene.env_origins.detach().cpu().numpy()
        translate_ops = getattr(self, "_visual_obstacle_translate_ops", None)
        if translate_ops is None:
            translate_ops = {}
            self._visual_obstacle_translate_ops = translate_ops
        for env_index, origin in enumerate(origins):
            for obstacle_index, local_position in enumerate(positions[env_index]):
                key = (env_index, obstacle_index)
                translate_op = translate_ops.get(key)
                if translate_op is None:
                    sphere = UsdGeom.Sphere.Define(
                        stage,
                        f"/World/SparkBenchmarkObstacles/env_{env_index}/obstacle_{obstacle_index}",
                    )
                    sphere.CreateRadiusAttr(float(radius))
                    rgba = np.asarray(
                        VizColor.obstacle_task if color is None else color,
                        dtype=float,
                    )
                    sphere.CreateDisplayColorAttr([Gf.Vec3f(*rgba[:3])])
                    sphere.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set(
                        [float(rgba[3])]
                    )
                    xform = UsdGeom.Xformable(sphere.GetPrim())
                    xform.ClearXformOpOrder()
                    translate_op = xform.AddTranslateOp()
                    translate_ops[key] = translate_op
                translate_op.Set(Gf.Vec3d(*(origin + local_position)))

    def set_visual_shared_obstacles(self, world_positions, radius: float = 0.05) -> None:
        """Render one world-space obstacle set shared by every cloned robot."""
        if not self.render_enabled:
            return
        import omni.usd
        from pxr import Gf, UsdGeom

        positions = np.asarray(world_positions, dtype=float).reshape(-1, 3)
        stage = omni.usd.get_context().get_stage()
        translate_ops = getattr(self, "_visual_shared_obstacle_translate_ops", None)
        if translate_ops is None:
            translate_ops = {}
            self._visual_shared_obstacle_translate_ops = translate_ops
        for index, position in enumerate(positions):
            translate_op = translate_ops.get(index)
            if translate_op is None:
                sphere = UsdGeom.Sphere.Define(
                    stage, f"/World/SparkSharedObstacles/obstacle_{index}"
                )
                sphere.CreateRadiusAttr(float(radius))
                rgba = np.asarray(VizColor.obstacle_task, dtype=float)
                sphere.CreateDisplayColorAttr([Gf.Vec3f(*rgba[:3])])
                sphere.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set([float(rgba[3])])
                xform = UsdGeom.Xformable(sphere.GetPrim())
                xform.ClearXformOpOrder()
                translate_op = xform.AddTranslateOp()
                translate_ops[index] = translate_op
            translate_op.Set(Gf.Vec3d(*position))

    def _set_collision_volume_visibility(self, visible: bool) -> None:
        """Show or hide persistent USD collision-volume overlays."""
        prims = getattr(self, "_visual_collision_prims", {})
        if not prims:
            return
        from pxr import UsdGeom

        for prim in prims.values():
            imageable = UsdGeom.Imageable(prim)
            if visible:
                imageable.MakeVisible()
            else:
                imageable.MakeInvisible()

    def set_visual_robot_collision_spheres(
        self,
        world_positions,
        radii,
        visible_mask=None,
        collision_mask=None,
        closest_distance=None,
        minimum_distance=0.05,
    ) -> None:
        """Create or update batched robot collision-volume visualization."""
        if not self.render_enabled:
            return
        if not self.render_robot_collision_volumes:
            self._set_collision_volume_visibility(False)
            return
        import omni.usd
        from pxr import Gf, UsdGeom, Vt
        from spark_utils import collision_volume_distance_color

        positions = np.asarray(world_positions, dtype=float)
        radii = np.asarray(radii, dtype=float).reshape(-1)
        if visible_mask is None:
            visible_mask = np.ones(radii.shape[0], dtype=bool)
        visible_mask = np.asarray(visible_mask, dtype=bool).reshape(-1)
        if collision_mask is None:
            collision_mask = np.zeros(positions.shape[:2], dtype=bool)
        collision_mask = np.asarray(collision_mask, dtype=bool)
        if closest_distance is None:
            closest_distance = np.where(collision_mask, 0.0, np.inf)
        closest_distance = np.asarray(closest_distance, dtype=float)
        if positions.ndim != 3 or positions.shape[0] != self.num_envs:
            raise ValueError("world_positions must have shape [num_envs, spheres, 3]")
        if positions.shape[1] != radii.shape[0]:
            raise ValueError("radii length must match sphere count")
        if visible_mask.shape[0] != radii.shape[0]:
            raise ValueError("visible_mask length must match sphere count")
        if collision_mask.shape != positions.shape[:2]:
            raise ValueError("collision_mask must have shape [num_envs, spheres]")
        if closest_distance.shape != positions.shape[:2]:
            raise ValueError("closest_distance must have shape [num_envs, spheres]")
        stage = omni.usd.get_context().get_stage()
        translate_ops = getattr(self, "_visual_collision_translate_ops", None)
        color_attrs = getattr(self, "_visual_collision_color_attrs", None)
        opacity_attrs = getattr(self, "_visual_collision_opacity_attrs", None)
        material_keys = getattr(self, "_visual_collision_material_keys", None)
        material_cache = getattr(self, "_debug_rgba_material_cache", None)
        prims = getattr(self, "_visual_collision_prims", None)
        if translate_ops is None:
            translate_ops = {}
            self._visual_collision_translate_ops = translate_ops
        if color_attrs is None:
            color_attrs = {}
            self._visual_collision_color_attrs = color_attrs
        if opacity_attrs is None:
            opacity_attrs = {}
            self._visual_collision_opacity_attrs = opacity_attrs
        if material_keys is None:
            material_keys = {}
            self._visual_collision_material_keys = material_keys
        if material_cache is None:
            material_cache = {}
            self._debug_rgba_material_cache = material_cache
        if prims is None:
            prims = {}
            self._visual_collision_prims = prims
        for env_index in range(self.num_envs):
            for sphere_index in range(positions.shape[1]):
                if not visible_mask[sphere_index]:
                    prim = prims.get((env_index, sphere_index))
                    if prim is not None:
                        UsdGeom.Imageable(prim).MakeInvisible()
                    continue
                key = (env_index, sphere_index)
                translate_op = translate_ops.get(key)
                if translate_op is None:
                    sphere = UsdGeom.Sphere.Define(
                        stage,
                        f"/World/SparkRobotCollision/env_{env_index}/sphere_{sphere_index}",
                    )
                    sphere.CreateRadiusAttr(float(radii[sphere_index]))
                    _disable_debug_prim_shadows(sphere.GetPrim())
                    color_attrs[key] = sphere.CreateDisplayColorAttr()
                    opacity_attrs[key] = sphere.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant)
                    xform = UsdGeom.Xformable(sphere.GetPrim())
                    xform.ClearXformOpOrder()
                    translate_op = xform.AddTranslateOp()
                    translate_ops[key] = translate_op
                    prims[key] = sphere.GetPrim()
                UsdGeom.Imageable(prims[key]).MakeVisible()
                translate_op.Set(Gf.Vec3d(*positions[env_index, sphere_index]))
                color = collision_volume_distance_color(
                    closest_distance[env_index, sphere_index],
                    minimum_distance,
                    opacity_scale=self.collision_volume_opacity_scale,
                    opacity_floor=self.collision_volume_opacity_floor,
                )
                color_attrs[key].Set(Vt.Vec3fArray([Gf.Vec3f(*color[:3])]))
                opacity_attrs[key].Set([float(color[3])])
                material_keys[key] = _bind_debug_rgba_material(
                    stage,
                    prims[key],
                    color,
                    material_cache,
                    previous_key=material_keys.get(key),
                )

    def set_visual_safety_constraints(
        self,
        witness_robot,
        witness_other,
        *,
        trigger_mask=None,
        violation_mask=None,
    ) -> None:
        """Render tensor safety diagnostics using the scalar SPARK colors.

        Trigger lines show nominal constraints that require correction. A
        violation line shows a positive residual that remains after filtering.
        One ``BasisCurves`` prim per category keeps the USD update cost bounded
        when many environments and collision pairs are visible.
        """
        if not self.render_enabled:
            return
        start = np.asarray(witness_robot, dtype=float)
        end = np.asarray(witness_other, dtype=float)
        if start.shape != end.shape or start.ndim != 3 or start.shape[-1] != 3:
            raise ValueError("safety witness points must share shape [num_envs, constraints, 3]")
        if start.shape[0] != self.num_envs:
            raise ValueError("safety witness batch must match num_envs")
        from isaacsim.util.debug_draw import _debug_draw

        draw = getattr(self, "_safety_debug_draw", None)
        if draw is None:
            draw = _debug_draw.acquire_debug_draw_interface()
            self._safety_debug_draw = draw
        draw.clear_lines()
        line_start, line_end, line_colors, line_widths = [], [], [], []

        def append_category(name, mask, color, width):
            if mask is not None:
                selected = np.asarray(mask, dtype=bool)
                if selected.shape != start.shape[:2]:
                    raise ValueError(f"{name}_mask must have shape {start.shape[:2]}")
                selected_start = start[selected]
                selected_end = end[selected]
                count = selected_start.shape[0]
                line_start.extend(map(tuple, selected_start.tolist()))
                line_end.extend(map(tuple, selected_end.tolist()))
                line_colors.extend([(*color, 1.0)] * count)
                line_widths.extend([float(width)] * count)

        # Match pipeline.visualization: unsafe/trigger blue and slack purple.
        # A one-pixel segment disappears against the 16-environment grid and
        # antialiased RTX floor. Keep the scalar viewer's colors but use a
        # diagnostic-width stroke so an active closest pair is observable.
        append_category("trigger", trigger_mask, (0.0, 32.0 / 255.0, 230.0 / 255.0), 3.0)
        append_category(
            "violation", violation_mask, (160.0 / 255.0, 32.0 / 255.0, 240.0 / 255.0), 4.0
        )
        if line_start:
            draw.draw_lines(line_start, line_end, line_colors, line_widths)

    def set_visual_environment_point_cloud(self, world_points, point_size: float = 0.003) -> None:
        """Render a perception-style point-sphere environment representation."""
        if not self.render_enabled:
            return
        points = np.asarray(world_points, dtype=float).reshape(-1, 3)
        from isaacsim.util.debug_draw import _debug_draw

        draw = getattr(self, "_point_cloud_debug_draw", None)
        if draw is None:
            draw = _debug_draw.acquire_debug_draw_interface()
            self._point_cloud_debug_draw = draw
        draw.clear_points()
        if len(points):
            draw.draw_points(
                list(map(tuple, points.tolist())),
                # Bright cyan remains visible against the gray floor, dark
                # robot materials, and both performance/quality RTX modes.
                [(0.0, 0.85, 1.0, 1.0)] * len(points),
                [float(point_size)] * len(points),
            )

    def set_visual_environment_mesh(self, vertices, faces) -> None:
        """Render the triangle mesh consumed by the mesh distance backend."""
        if not self.render_enabled:
            return
        import omni.usd
        from pxr import Gf, UsdGeom, Vt

        vertices = np.asarray(vertices, dtype=float).reshape(-1, 3)
        faces = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
        stage = omni.usd.get_context().get_stage()
        geometry = UsdGeom.Mesh.Get(stage, "/World/SparkEnvironmentMesh")
        if not geometry:
            geometry = UsdGeom.Mesh.Define(stage, "/World/SparkEnvironmentMesh")
            rgba = np.asarray(VizColor.obstacle_task, dtype=float)
            geometry.CreateDisplayColorAttr([Gf.Vec3f(*rgba[:3])])
            geometry.CreateDisplayOpacityAttr(Vt.FloatArray([float(rgba[3])]))
        geometry.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*point) for point in vertices]))
        geometry.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(faces)))
        geometry.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces.reshape(-1).tolist()))

    def set_visual_goals(
        self,
        base_positions,
        left_arm_positions=None,
        right_arm_positions=None,
        base_radius: float = 0.15,
        arm_radius: float = 0.05,
    ) -> None:
        """Render per-environment base and arm targets as non-physical spheres."""
        if not self.render_enabled:
            return
        import omni.usd
        from pxr import Gf, UsdGeom

        # Match ``render_value_based_debug_info`` while using Isaac's stronger
        # goal opacity so the bright viewport does not wash markers out.
        goal_color = tuple(float(value) for value in VizColor.goal[:3])
        goals = {
            "base": (base_positions, goal_color, base_radius),
            "left_arm": (left_arm_positions, goal_color, arm_radius),
            "right_arm": (right_arm_positions, goal_color, arm_radius),
        }
        stage = omni.usd.get_context().get_stage()
        origins = self.scene.env_origins.detach().cpu().numpy()
        translate_ops = getattr(self, "_visual_goal_translate_ops", None)
        if translate_ops is None:
            translate_ops = {}
            self._visual_goal_translate_ops = translate_ops
        for name, (positions, color, radius) in goals.items():
            prim_paths = [
                f"/World/SparkBenchmarkGoals/env_{env_index}/{name}"
                for env_index in range(self.num_envs)
            ]
            if positions is None:
                # Goal types can be disabled by the selected benchmark case.
                # Hide persistent prims if a caller changes modes at runtime;
                # more importantly, do not create zero-offset arm markers at
                # the robot waist for base-only Sport/WBT tasks.
                for prim_path in prim_paths:
                    prim = stage.GetPrimAtPath(prim_path)
                    if prim.IsValid():
                        UsdGeom.Imageable(prim).MakeInvisible()
                continue
            positions = np.asarray(positions, dtype=float).reshape(self.num_envs, 3)
            for env_index, (origin, local_position, prim_path) in enumerate(
                zip(origins, positions, prim_paths)
            ):
                key = (name, env_index)
                translate_op = translate_ops.get(key)
                if translate_op is None:
                    sphere = UsdGeom.Sphere.Define(
                        stage,
                        prim_path,
                    )
                    sphere.CreateRadiusAttr(float(radius))
                    sphere.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                    sphere.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set(
                        [ISAAC_GOAL_OPACITY]
                    )
                    xform = UsdGeom.Xformable(sphere.GetPrim())
                    xform.ClearXformOpOrder()
                    translate_op = xform.AddTranslateOp()
                    translate_ops[key] = translate_op
                else:
                    UsdGeom.Imageable(stage.GetPrimAtPath(prim_path)).MakeVisible()
                translate_op.Set(Gf.Vec3d(*(origin + local_position)))

    def _as_batch(self, value, width: int, name: str):
        tensor = self._torch.as_tensor(value, device=self.device, dtype=self._torch.float32)
        if tensor.ndim == 1:
            if tensor.shape[0] != width:
                raise ValueError(f"{name} must have {width} entries")
            tensor = tensor.unsqueeze(0).expand(self.num_envs, -1)
        if tensor.shape != (self.num_envs, width):
            raise ValueError(
                f"{name} must have shape ({self.num_envs}, {width}), got {tuple(tensor.shape)}"
            )
        return tensor

    def _set_gripper_targets(self, action_info: dict) -> None:
        if not self.enable_hand_control:
            return
        for side, ids in self.gripper_joint_ids.items():
            spec = UNITREE_G1_GRIPPER_SPECS[side]
            control_key = f"{side}_gripper_control"
            goal_key = f"{side}_gripper_goal"
            if control_key in action_info:
                target = self._as_batch(action_info[control_key], 7, control_key)
                lower = self._torch.as_tensor(spec.lower_limit, device=self.device)
                upper = self._torch.as_tensor(spec.upper_limit, device=self.device)
                self.gripper_targets[side] = self._torch.clamp(target, lower, upper)
                continue
            goal = action_info.get(goal_key)
            if goal is None and side == "right":
                goal = action_info.get("gripper_goal")
            if goal is None:
                continue
            closed = self._torch.as_tensor(goal, device=self.device, dtype=self._torch.bool)
            if closed.ndim == 0:
                closed = closed.repeat(self.num_envs)
            if closed.shape != (self.num_envs,):
                raise ValueError(f"{goal_key} must be scalar or shape ({self.num_envs},)")
            opened = self._torch.as_tensor(spec.open_position, device=self.device)
            shut = self._torch.as_tensor(spec.closed_position, device=self.device)
            self.gripper_targets[side] = self._torch.where(closed[:, None], shut, opened)

    def reset(self, agent_reset_info=None, env_ids=None, **kwargs) -> None:
        if env_ids is None:
            env_ids = self._torch.arange(self.num_envs, device=self.device, dtype=self._torch.long)
        else:
            env_ids = self._torch.as_tensor(env_ids, device=self.device, dtype=self._torch.long)
        root_pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        root_pose[:, :3] += self.scene.env_origins[env_ids]
        requested_root_pose = kwargs.get("root_pose_w")
        if requested_root_pose is not None:
            requested_root_pose = self._torch.as_tensor(
                requested_root_pose, device=self.device, dtype=root_pose.dtype
            )
            if requested_root_pose.shape != root_pose.shape:
                raise ValueError(
                    f"root_pose_w must have shape {tuple(root_pose.shape)}, "
                    f"got {tuple(requested_root_pose.shape)}"
                )
            root_pose.copy_(requested_root_pose)
        root_vel = self.robot.data.default_root_vel.torch[env_ids].clone()
        joint_pos = self.robot.data.default_joint_pos.torch[env_ids].clone()
        joint_vel = self.robot.data.default_joint_vel.torch[env_ids].clone()
        self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=root_vel, env_ids=env_ids)
        self.robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=joint_vel, env_ids=env_ids)
        self.target_body_pos[env_ids] = self.default_body_pos[env_ids]
        self.target_body_vel[env_ids] = 0.0
        self.feedforward_torque[env_ids] = 0.0
        self.last_control[env_ids] = 0.0
        self._model_command_velocity[env_ids] = 0.0
        self.control_body_kp[env_ids] = self.body_kp
        self.control_body_kd[env_ids] = self.body_kd
        self.scene.reset(env_ids)
        self.scene.write_data_to_sim()
        self.sim.forward()
        self.scene.update(0.0)
        if env_ids.numel() == self.num_envs:
            self.step_index = 0
            self.time = 0.0

    def send_control(self, control, **kwargs) -> None:
        action_info = dict(kwargs.get("action_info") or {})
        if "target_actuated_pos" in action_info:
            self.target_body_pos = self._as_batch(
                action_info["target_actuated_pos"], self.num_actuated, "target_actuated_pos"
            ).clone()
        if "target_actuated_vel" in action_info:
            self.target_body_vel = self._as_batch(
                action_info["target_actuated_vel"], self.num_actuated, "target_actuated_vel"
            ).clone()
        else:
            self.target_body_vel.zero_()
        if "feedforward_torque" in action_info:
            self.feedforward_torque = self._as_batch(
                action_info["feedforward_torque"], self.num_actuated, "feedforward_torque"
            ).clone()
        else:
            self.feedforward_torque.zero_()
        if "motor_kps" in action_info:
            self.control_body_kp = self._as_batch(
                action_info["motor_kps"], self.num_actuated, "motor_kps"
            ).clone()
        if "motor_kds" in action_info:
            self.control_body_kd = self._as_batch(
                action_info["motor_kds"], self.num_actuated, "motor_kds"
            ).clone()
        if control is not None:
            self.last_control = self._as_batch(control, self.num_actuated, "control").clone()
        self._set_gripper_targets(action_info)

    def _apply_efforts(self) -> None:
        if self.native_implicit_pd:
            self.robot.set_joint_position_target_index(
                target=self.target_body_pos, joint_ids=self.body_joint_ids
            )
            self.robot.set_joint_velocity_target_index(
                target=self.target_body_vel, joint_ids=self.body_joint_ids
            )
            if self.enable_hand_control:
                for side, ids in self.gripper_joint_ids.items():
                    self.robot.set_joint_position_target_index(
                        target=self.gripper_targets[side], joint_ids=ids
                    )
                    self.robot.set_joint_velocity_target_index(
                        target=self._torch.zeros_like(self.gripper_targets[side]),
                        joint_ids=ids,
                    )
            if self.locked_joint_targets is not None:
                self.robot.set_joint_position_target_index(
                    target=self.locked_joint_targets, joint_ids=self.locked_joint_ids
                )
                self.robot.set_joint_velocity_target_index(
                    target=self._torch.zeros_like(self.locked_joint_targets),
                    joint_ids=self.locked_joint_ids,
                )
            return
        pos = self.robot.data.joint_pos.torch[:, self.body_joint_ids]
        vel = self.robot.data.joint_vel.torch[:, self.body_joint_ids]
        effort = (
            self.feedforward_torque
            + self.control_body_kp * (self.target_body_pos - pos)
            + self.control_body_kd * (self.target_body_vel - vel)
        )
        effort = self._torch.clamp(effort, -self.body_effort_limit, self.body_effort_limit)
        if self.hybrid_implicit_upper_body:
            # Match the qualified scalar WBT realization: its locomotion
            # network owns explicit lower-body effort-PD, while the waist and
            # arms use PhysX implicit drives to avoid exciting low-inertia
            # upper-body joints. Joint order is WBT's 12 legs + 17 upper body.
            effort[:, 12:] = 0.0
        self.robot.set_joint_effort_target_index(target=effort, joint_ids=self.body_joint_ids)

        if self.hybrid_implicit_upper_body:
            upper_ids = self.body_joint_ids[12:]
            self.robot.set_joint_position_target_index(
                target=self.target_body_pos[:, 12:], joint_ids=upper_ids
            )
            self.robot.set_joint_velocity_target_index(
                target=self.target_body_vel[:, 12:], joint_ids=upper_ids
            )

        if self.enable_hand_control and self.hybrid_implicit_upper_body:
            for side, ids in self.gripper_joint_ids.items():
                self.robot.set_joint_position_target_index(
                    target=self.gripper_targets[side], joint_ids=ids
                )
                self.robot.set_joint_velocity_target_index(
                    target=self._torch.zeros_like(self.gripper_targets[side]),
                    joint_ids=ids,
                )
        elif self.enable_hand_control:
            for side, ids in self.gripper_joint_ids.items():
                hand_pos = self.robot.data.joint_pos.torch[:, ids]
                hand_vel = self.robot.data.joint_vel.torch[:, ids]
                hand_effort = (
                    self.hand_kp * (self.gripper_targets[side] - hand_pos) - self.hand_kd * hand_vel
                )
                limit = self.gripper_effort_limits[side]
                hand_effort = self._torch.clamp(hand_effort, -limit, limit)
                self.robot.set_joint_effort_target_index(target=hand_effort, joint_ids=ids)

        if self.locked_joint_targets is not None:
            locked_pos = self.robot.data.joint_pos.torch[:, self.locked_joint_ids]
            locked_vel = self.robot.data.joint_vel.torch[:, self.locked_joint_ids]
            locked_effort = 300.0 * (self.locked_joint_targets - locked_pos) - 12.0 * locked_vel
            self.robot.set_joint_effort_target_index(
                target=locked_effort, joint_ids=self.locked_joint_ids
            )

    def _post_control_processing(self, **kwargs) -> None:
        for _ in range(self.control_decimation):
            self._apply_efforts()
            self.scene.write_data_to_sim()
            # Physics substeps belong to one control action. Rendering every
            # 5 ms substep multiplied Kit work by the control decimation and
            # made the four-environment viewer run at ~13 policy Hz.
            self.sim.step(render=False)
            self.scene.update(self.dt)
        if (
            self.render_enabled
            and self.render_on_step
            and self.step_index % self.render_decimation == 0
        ):
            self._update_simulation_info_overlay()
            self.sim.render()
            if self._viewport_layout_updates_remaining > 0:
                _show_viewport_only()
                self._viewport_layout_updates_remaining -= 1
        self.step_index += 1
        self.time += self.dt * self.control_decimation

    def step(self, control, **kwargs) -> None:
        has_virtual_planar_base = {
            "LinearX",
            "LinearY",
            "RotYaw",
        }.issubset(self.robot_cfg.DoFs.__members__)
        if control is not None and "hold_dof_indices" in kwargs and has_virtual_planar_base:
            self._step_configured_mobile_base(control)
            return
        if control is not None and "hold_dof_indices" in kwargs:
            # The shared first-order tensor benchmark supplies generalized
            # velocities directly. Convert them to the position/velocity
            # targets consumed by Unitree's simulator-owned PD realization.
            velocity = self._as_batch(control, self.num_actuated, "control")
            current = self.robot.data.joint_pos.torch[:, self.body_joint_ids]
            self.target_body_pos = current + velocity * (self.dt * self.control_decimation)
            self.target_body_vel = velocity.clone()
            self.feedforward_torque.zero_()
            self.last_control = velocity.clone()
            self._post_control_processing(**kwargs)
            return
        self.send_control(control, **kwargs)
        self._post_control_processing(**kwargs)

    def _synchronize_kinematic_articulation(self) -> None:
        """Propagate directly written articulation state to rendered links.

        Mobile-base mode, like MuJoCo's modeled-dynamics agent, teleports the
        generalized configuration without taking a PhysX dynamics step.  The
        tensor state changes immediately, but Fabric/USD link transforms need
        an explicit articulation forward-kinematics update before rendering.
        """
        physics_view = getattr(self.sim, "physics_sim_view", None)
        if physics_view is not None:
            physics_view.update_articulations_kinematic()

    def _step_configured_mobile_base(self, model_velocity) -> None:
        """Advance a config-level planar base plus its selected body joints.

        The mobile-base robot configuration represents x/y/yaw as virtual
        generalized coordinates rather than PhysX articulation joints.  This
        method realizes those three controls on the free root while applying
        the remaining first-order controls to the selected waist/arm joints.
        """

        velocity = self._as_batch(model_velocity, self.model_dof_count, "control")
        period = float(self.dt * self.control_decimation)
        x_id = int(self.robot_cfg.DoFs.LinearX)
        y_id = int(self.robot_cfg.DoFs.LinearY)
        yaw_id = int(self.robot_cfg.DoFs.RotYaw)

        root_pose = self.robot.data.root_pose_w.torch.clone()
        qx, qy, qz, qw = root_pose[:, 3:7].unbind(dim=-1)
        yaw = self._torch.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy.square() + qz.square()),
        )
        cosine, sine = self._torch.cos(yaw), self._torch.sin(yaw)
        root_pose[:, 0] += period * (cosine * velocity[:, x_id] - sine * velocity[:, y_id])
        root_pose[:, 1] += period * (sine * velocity[:, x_id] + cosine * velocity[:, y_id])
        yaw = yaw + period * velocity[:, yaw_id]
        root_pose[:, 3] = 0.0
        root_pose[:, 4] = 0.0
        root_pose[:, 5] = self._torch.sin(0.5 * yaw)
        root_pose[:, 6] = self._torch.cos(0.5 * yaw)

        joint_pos = self.robot.data.joint_pos.torch.clone()
        current_body = joint_pos[:, self.body_joint_ids]
        body_velocity = velocity[:, self.body_model_dof_ids_tensor]
        desired_body = current_body + period * body_velocity
        joint_pos[:, self.body_joint_ids] = desired_body
        env_ids = self._torch.arange(self.num_envs, device=self.device, dtype=self._torch.long)
        self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=self._torch.zeros_like(self.robot.data.root_vel_w.torch),
            env_ids=env_ids,
        )
        self.robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(
            velocity=self._torch.zeros_like(joint_pos), env_ids=env_ids
        )
        self.target_body_pos = desired_body
        self.target_body_vel = body_velocity.clone()
        self.last_control = body_velocity.clone()
        self._model_command_velocity = velocity.clone()
        self.scene.write_data_to_sim()
        self.sim.forward()
        self.scene.update(period)
        self._synchronize_kinematic_articulation()
        self.step_index += 1
        self.time += period

    def step_kinematic_mobile_base(
        self,
        body_velocity,
        body_joint_target,
        *,
        joint_velocity_limit=0.55,
    ) -> None:
        """Advance an ideal planar base and upper body without leg dynamics.

        This is the Isaac counterpart of SPARK's abstract mobile-base model.
        It deliberately bypasses lower-body WBT tracking so parallel safety
        and navigation can be evaluated independently of learned locomotion.
        """
        velocity = self._as_batch(body_velocity, 3, "body_velocity")
        upper_target = self._as_batch(body_joint_target, 17, "body_joint_target")
        period = float(self.dt * self.control_decimation)
        root_pose = self.robot.data.root_pose_w.torch.clone()
        qx, qy, qz, qw = root_pose[:, 3:7].unbind(dim=-1)
        yaw = self._torch.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy.square() + qz.square()),
        )
        c, s = self._torch.cos(yaw), self._torch.sin(yaw)
        root_pose[:, 0] += period * (c * velocity[:, 0] - s * velocity[:, 1])
        root_pose[:, 1] += period * (s * velocity[:, 0] + c * velocity[:, 1])
        yaw = yaw + period * velocity[:, 2]
        root_pose[:, 3] = 0.0
        root_pose[:, 4] = 0.0
        root_pose[:, 5] = self._torch.sin(0.5 * yaw)
        root_pose[:, 6] = self._torch.cos(0.5 * yaw)

        joint_pos = self.robot.data.joint_pos.torch.clone()
        current_body = joint_pos[:, self.body_joint_ids]
        desired_body = self.default_body_pos.clone()
        maximum_step = (
            self._torch.as_tensor(
                joint_velocity_limit, device=self.device, dtype=self._torch.float32
            )
            * period
        )
        desired_body[:, 12:] = current_body[:, 12:] + self._torch.clamp(
            upper_target - current_body[:, 12:], -maximum_step, maximum_step
        )
        joint_pos[:, self.body_joint_ids] = desired_body
        env_ids = self._torch.arange(self.num_envs, device=self.device, dtype=self._torch.long)
        self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=self._torch.zeros_like(self.robot.data.root_vel_w.torch),
            env_ids=env_ids,
        )
        self.robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(
            velocity=self._torch.zeros_like(joint_pos), env_ids=env_ids
        )
        self.target_body_pos = desired_body
        self.scene.write_data_to_sim()
        self.sim.forward()
        self.scene.update(period)
        if self.render_enabled and self.step_index % self.render_decimation == 0:
            # Isaac 6's Kit visualizer does not reliably consume a pure
            # tensor-state teleport for cloned GPU articulations.  A physics
            # fetch marks the articulation transforms dirty through the same
            # path used by WBT.  Restore the exact ideal configuration
            # immediately afterwards so this remains a kinematic plant rather
            # than allowing gravity/contact dynamics into its state.
            self.sim.step(render=False)
            self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
            self.robot.write_root_velocity_to_sim_index(
                root_velocity=self._torch.zeros_like(self.robot.data.root_vel_w.torch),
                env_ids=env_ids,
            )
            self.robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
            self.robot.write_joint_velocity_to_sim_index(
                velocity=self._torch.zeros_like(joint_pos), env_ids=env_ids
            )
            self.scene.write_data_to_sim()
            self.sim.forward()
            self.scene.update(0.0)
            self._synchronize_kinematic_articulation()
            if self.render_on_step:
                self._update_simulation_info_overlay()
                self.sim.render()
            # The dynamics path performs this from _post_control_processing;
            # mobile-base mode has its own stepping path and must do the same
            # so Kit remains a viewport-only window rather than an editor.
            if self._viewport_layout_updates_remaining > 0:
                _show_viewport_only()
                self._viewport_layout_updates_remaining -= 1
        self.step_index += 1
        self.time += period

    def render_frame(self) -> None:
        """Present articulation and overlays after one atomic visual update."""
        if not self.render_enabled:
            return
        protected_state = self._snapshot_camera_render_state() if self.enable_camera else None
        try:
            self._flush_debug_primitives()
            self._synchronize_kinematic_articulation()
            self._update_simulation_info_overlay()
            self.sim.render()
            if self.enable_camera:
                self._capture_isaaclab_camera_feedback()
            if self._viewport_layout_updates_remaining > 0:
                _show_viewport_only()
                self._viewport_layout_updates_remaining -= 1
        finally:
            if protected_state is not None:
                self._restore_camera_render_state(protected_state)

    def _flush_debug_primitives(self) -> None:
        """Author the scalar teleop goal, obstacle, and safety overlays."""

        IsaacAgent._flush_debug_primitives(self)

    def _snapshot_camera_render_state(self):
        """Snapshot the authoritative PhysX state before pumping RTX."""

        return (
            self.robot.data.root_pose_w.torch.clone(),
            self.robot.data.root_vel_w.torch.clone(),
            self.robot.data.joint_pos.torch.clone(),
            self.robot.data.joint_vel.torch.clone(),
        )

    def _restore_camera_render_state(self, state) -> None:
        """Discard any physics advanced by Kit while producing a sensor frame.

        IsaacLab's native RTX renderer pumps Kit's application loop and may
        temporarily enable automatic simulation playback.  SPARK owns physics
        stepping explicitly, so camera rendering must not add uncontrolled
        substeps between WBT commands.
        """

        root_pose, root_velocity, joint_position, joint_velocity = state
        env_ids = self._torch.arange(self.num_envs, device=self.device, dtype=self._torch.long)
        self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=root_velocity, env_ids=env_ids)
        self.robot.write_joint_position_to_sim_index(position=joint_position, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=joint_velocity, env_ids=env_ids)
        self.scene.write_data_to_sim()
        self.sim.forward()
        self.scene.update(0.0)

    def _model_dof_feedback(self):
        """Return the robot-config DoF view, including a virtual planar base."""

        position = self.default_dof_pos.clone()
        velocity = self._torch.zeros_like(position)
        position[:, self.body_model_dof_ids_tensor] = self.robot.data.joint_pos.torch[
            :, self.body_joint_ids
        ]
        velocity[:, self.body_model_dof_ids_tensor] = self.robot.data.joint_vel.torch[
            :, self.body_joint_ids
        ]
        members = self.robot_cfg.DoFs.__members__
        if {"LinearX", "LinearY", "RotYaw"}.issubset(members):
            x_id = int(self.robot_cfg.DoFs.LinearX)
            y_id = int(self.robot_cfg.DoFs.LinearY)
            yaw_id = int(self.robot_cfg.DoFs.RotYaw)
            root_pose = self.robot.data.root_pose_w.torch
            root_velocity = self.robot.data.root_vel_w.torch
            qx, qy, qz, qw = root_pose[:, 3:7].unbind(dim=-1)
            yaw = self._torch.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy.square() + qz.square()),
            )
            position[:, x_id] = root_pose[:, 0] - self.env_positions[:, 0]
            position[:, y_id] = root_pose[:, 1] - self.env_positions[:, 1]
            position[:, yaw_id] = yaw
            cosine, sine = self._torch.cos(yaw), self._torch.sin(yaw)
            velocity[:, x_id] = cosine * root_velocity[:, 0] + sine * root_velocity[:, 1]
            velocity[:, y_id] = -sine * root_velocity[:, 0] + cosine * root_velocity[:, 1]
            velocity[:, yaw_id] = root_velocity[:, 5]
        return position, velocity

    def get_feedback(self):
        self._poll_renderer_input()
        root_pose = self.robot.data.root_pose_w.torch
        root_vel = self.robot.data.root_vel_w.torch
        feedback = {
            "time": self.time,
            "step_index": self.step_index,
            "root_pose_w": root_pose,
            "root_velocity_w": root_vel,
            # Learned locomotion policies are trained with base-frame angular
            # velocity. Keep world velocity as a separate diagnostic signal.
            "root_angular_velocity_b": self.robot.data.root_ang_vel_b.torch,
            "body_joint_pos": self.robot.data.joint_pos.torch[:, self.body_joint_ids],
            "body_joint_vel": self.robot.data.joint_vel.torch[:, self.body_joint_ids],
            "target_actuated_pos": self.target_body_pos,
            "motor_kps": self.control_body_kp,
            "motor_kds": self.control_body_kd,
            "control": self.last_control,
        }
        model_position, model_velocity = self._model_dof_feedback()
        feedback["dof_pos_fbk"] = model_position
        feedback["dof_vel_fbk"] = model_velocity
        for side, ids in self.gripper_joint_ids.items():
            feedback[f"{side}_gripper_pos_fbk"] = self.robot.data.joint_pos.torch[:, ids]
            feedback[f"{side}_gripper_vel_fbk"] = self.robot.data.joint_vel.torch[:, ids]
        if not self.scalar_api:
            return feedback

        root = root_pose[0].detach().cpu().numpy().astype(float, copy=True)
        root_velocity = root_vel[0].detach().cpu().numpy().astype(float, copy=True)
        body_pos = feedback["body_joint_pos"][0].detach().cpu().numpy().astype(float, copy=True)
        body_vel = feedback["body_joint_vel"][0].detach().cpu().numpy().astype(float, copy=True)
        # IsaacLab stores root quaternions as xyzw; SONIC and SPARK's scalar
        # policy boundary use MuJoCo's wxyz convention.
        qpos = np.concatenate((root[:3], root[[6, 3, 4, 5]], body_pos))
        qvel = np.concatenate(
            (
                root_velocity[:3],
                feedback["root_angular_velocity_b"][0].detach().cpu().numpy(),
                body_vel,
            )
        )
        model_qpos = np.asarray(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs],
            dtype=float,
        )
        model_qvel = np.zeros(max(0, self.num_dof - 1), dtype=float)
        model_qpos[:7] = qpos[:7]
        model_qvel[:6] = qvel[:6]
        for body_index, model_index in enumerate(self.body_model_dof_ids):
            model_qpos[model_index] = body_pos[body_index]
            if model_index > 0 and model_index - 1 < model_qvel.size:
                model_qvel[model_index - 1] = body_vel[body_index]
        for side, ids in self.gripper_joint_ids.items():
            positions = self.robot.data.joint_pos.torch[0, ids].detach().cpu().numpy()
            velocities = self.robot.data.joint_vel.torch[0, ids].detach().cpu().numpy()
            for joint_name, position, velocity in zip(
                UNITREE_G1_GRIPPER_SPECS[side].joint_names, positions, velocities
            ):
                enum_name = "".join(
                    part.capitalize() for part in joint_name.removesuffix("_joint").split("_")
                )
                if not hasattr(self.robot_cfg.DoFs, enum_name):
                    continue
                model_index = int(getattr(self.robot_cfg.DoFs, enum_name))
                model_qpos[model_index] = float(position)
                if model_index > 0 and model_index - 1 < model_qvel.size:
                    model_qvel[model_index - 1] = float(velocity)
        self.dof_pos_fbk = model_qpos.copy()
        self.dof_vel_fbk = model_qvel.copy()
        self.dof_pos_cmd = model_qpos.copy()
        self.dof_vel_cmd = self.dof_vel_fbk.copy()
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)
        robot_base_frame = np.eye(4, dtype=float)
        qx, qy, qz, qw = root[3:7]
        robot_base_frame[:3, :3] = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ]
        )
        robot_base_frame[:3, 3] = root[:3]
        scalar = {
            "time": self.time,
            "step_index": self.step_index,
            "state": self.robot_cfg.compose_state_from_dof(model_qpos, model_qvel),
            "robot_base_frame": robot_base_frame,
            "dof_pos_fbk": model_qpos.copy(),
            "dof_vel_fbk": self.dof_vel_fbk.copy(),
            "dof_pos_cmd": self.dof_pos_cmd.copy(),
            "dof_vel_cmd": self.dof_vel_cmd.copy(),
            "dof_acc_cmd": self.dof_acc_cmd.copy(),
            "control": self.last_control[0].detach().cpu().numpy().astype(float, copy=True),
            "qpos_fbk": model_qpos.copy(),
            "qvel_fbk": model_qvel.copy(),
            "body_qpos_fbk": qpos.copy(),
            "body_qvel_fbk": qvel.copy(),
            "obstacle_debug_frame": self.obstacle_debug_frame.copy(),
            "obstacle_debug_geom": self.obstacle_debug_geom,
            "obstacle_debug_velocity": self.obstacle_debug_velocity.copy(),
            "robot_goal_left_offset": self.left_goal_debug_frame.copy(),
            "robot_goal_right_offset": self.right_goal_debug_frame.copy(),
            "robot_goal_base_offset": self.base_goal_debug_frame.copy(),
            "left_gripper_goal": self.left_gripper_goal,
            "right_gripper_goal": self.right_gripper_goal,
            "left_gripper_debug_state": self.left_gripper_debug_state,
            "right_gripper_debug_state": self.right_gripper_debug_state,
            "camera_feedback": IsaacAgent.get_camera_feedback(self),
        }
        return scalar

    def attach_simulation_app(self, simulation_app) -> None:
        self._simulation_app = simulation_app

    def attach_renderer_process(self, renderer_process) -> None:
        self._renderer_process = renderer_process
        self._poll_renderer_input()

    def _poll_renderer_input(self) -> None:
        if self._renderer_process is None:
            return
        state = self._renderer_process.poll_input()
        if state is None:
            return
        self.obstacle_debug_frame = np.asarray(state["obstacle_debug_frame"], dtype=float).copy()
        self.obstacle_debug_geom = list(state["obstacle_debug_geom"])
        self.num_obstacle_debug = len(self.obstacle_debug_geom)
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.left_goal_debug_frame = np.asarray(state["robot_goal_left_offset"], dtype=float).copy()
        self.right_goal_debug_frame = np.asarray(
            state["robot_goal_right_offset"], dtype=float
        ).copy()
        self.base_goal_debug_frame = np.asarray(state["robot_goal_base_offset"], dtype=float).copy()
        for name in (
            "left_gripper_goal",
            "right_gripper_goal",
            "left_gripper_debug_state",
            "right_gripper_debug_state",
        ):
            if name in state:
                setattr(self, name, bool(state[name]))

    def _close_renderer_process(self):
        if self._renderer_process is None:
            return None
        renderer = self._renderer_process
        self._renderer_process = None
        self.renderer_process_status = renderer.close(force=True)
        return self.renderer_process_status

    def close_viewer(self) -> None:
        if self._keyboard_subscription is not None:
            try:
                self._keyboard_input.unsubscribe_to_keyboard_events(
                    self._keyboard, self._keyboard_subscription
                )
            except (AttributeError, RuntimeError):
                pass
            self._keyboard_subscription = None
        self._close_renderer_process()
        close_camera_display(self)

    def is_running(self) -> bool:
        if self._renderer_process is not None:
            return bool(self._renderer_process.is_alive)
        return self._simulation_app is None or self._simulation_app.is_running()

    def render(self) -> None:
        if self._renderer_process is not None:
            feedback = self.get_feedback()
            self._renderer_process.submit(
                {
                    "sequence_id": self.step_index,
                    "dof_pos": np.asarray(feedback["dof_pos_fbk"], dtype=float).copy(),
                    "robot_base_frame": np.asarray(
                        feedback["robot_base_frame"], dtype=float
                    ).copy(),
                    "debug_primitives": self._debug_primitives,
                    "obstacle_debug_colors": [geom.color for geom in self.obstacle_debug_geom],
                    "left_gripper_goal": bool(self.left_gripper_goal),
                    "right_gripper_goal": bool(self.right_gripper_goal),
                }
            )
            self._debug_primitives = []
            return
        if self.render_enabled:
            self.render_frame()

    @staticmethod
    def _visible_debug_color(color) -> bool:
        value = np.asarray(color, dtype=float).reshape(-1)
        return value.size < 4 or float(value[3]) > 0.0

    @staticmethod
    def _rgba(color):
        return IsaacAgent._rgba(color)

    def render_sphere(self, pos, mat, size, color):
        if self._visible_debug_color(color):
            self._debug_primitives.append(
                {
                    "type": "sphere",
                    "pos": np.asarray(pos, float),
                    "size": np.asarray(size, float),
                    "color": color,
                }
            )

    def render_box(self, pos, mat, size, color):
        if self._visible_debug_color(color):
            self._debug_primitives.append(
                {
                    "type": "box",
                    "pos": np.asarray(pos, float),
                    "mat": np.asarray(mat, float).reshape(3, 3),
                    "size": np.asarray(size, float),
                    "color": color,
                }
            )

    def render_line_segment(self, pos1, pos2, radius=0.002, color=VizColor.goal):
        if self._visible_debug_color(color):
            self._debug_primitives.append(
                {
                    "type": "line",
                    "start": np.asarray(pos1, float),
                    "end": np.asarray(pos2, float),
                    "radius": float(radius),
                    "color": color,
                }
            )

    def render_pixel_line_segment(self, pos1, pos2, width=2.0, color=VizColor.goal, **kwargs):
        self.render_line_segment(pos1, pos2, radius=max(float(width), 1.0) * 0.001, color=color)

    def render_coordinate_frame(self, frame, size=0.1):
        frame = np.asarray(frame, dtype=float).reshape(4, 4)
        origin = frame[:3, 3]
        self.render_line_segment(origin, origin + size * frame[:3, 0], color=VizColor.x_axis)
        self.render_line_segment(origin, origin + size * frame[:3, 1], color=VizColor.y_axis)
        self.render_line_segment(origin, origin + size * frame[:3, 2], color=VizColor.z_axis)

    def render_surface(self, triangles, color):
        for triangle in np.asarray(triangles, dtype=float).reshape(-1, 3, 3):
            self.render_line_segment(triangle[0], triangle[1], color=color)
            self.render_line_segment(triangle[1], triangle[2], color=color)
            self.render_line_segment(triangle[2], triangle[0], color=color)

    def get_body_kinematics(self, *, include_jacobian: bool = True):
        """Return GPU-resident link poses and optional geometric Jacobians.

        The public body and joint order is the same order exposed by
        ``body_names`` and ``joint_names``.  Keeping this accessor on the
        agent avoids making safety policies depend on Isaac Lab internals.
        """
        result = {
            "body_names": tuple(self.robot.body_names),
            "joint_names": self.joint_names,
            "body_position_w": self.robot.data.body_pos_w.torch,
            "body_quaternion_w": self.robot.data.body_quat_w.torch,
            "body_joint_ids": self.body_joint_ids.to(dtype=self._torch.long),
        }
        if include_jacobian:
            jacobian = self.robot.data.body_link_jacobian_w.torch
            if jacobian.shape[1] == len(result["body_names"]) - 1:
                root = self._torch.zeros(
                    self.num_envs,
                    1,
                    jacobian.shape[2],
                    jacobian.shape[3],
                    device=jacobian.device,
                    dtype=jacobian.dtype,
                )
                jacobian = self._torch.cat((root, jacobian), dim=1)
            if jacobian.shape[1] != len(result["body_names"]):
                raise RuntimeError(
                    "Unitree Isaac link/Jacobian layout mismatch: "
                    f"{len(result['body_names'])} bodies versus "
                    f"{jacobian.shape[1]} Jacobian rows"
                )
            result["body_jacobian_w"] = jacobian
        return result

    @property
    def command_pos(self):
        """Selected-DoF command used by the shared tensor diagnostics."""

        command, _ = self._model_dof_feedback()
        command[:, self.body_model_dof_ids_tensor] = self.target_body_pos
        return command

    def get_all_joint_positions(self):
        """Return all articulation joint positions for passive-drift checks."""

        return self.robot.data.joint_pos.torch

    def close(self) -> None:
        self._close_simulation_info_overlay()
        close_camera_display(self)
        self._isaaclab_sensor_cameras.clear()
        self.robot = None
        self.scene = None
        self.sim = None
