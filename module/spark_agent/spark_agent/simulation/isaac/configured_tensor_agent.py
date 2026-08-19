"""Robot-agnostic batched Isaac articulation adapter."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from spark_agent.base.base_agent import BaseAgent
from spark_agent.simulation.camera_display import close_camera_display
from spark_agent.simulation.isaac.isaac_agent import (
    ISAAC_COLLISION_VOLUME_OPACITY_FLOOR,
    ISAAC_COLLISION_VOLUME_OPACITY_SCALE,
    IsaacAgent,
    _rpy_to_quaternion_wxyz,
    _show_viewport_only,
    apply_urdf_visual_palette,
    augment_urdf_planar_base,
    compose_urdf_instances,
    fit_viewer_config_to_prim,
    import_urdf_to_usd,
)
from spark_agent.simulation.isaac.viewer_info import IsaacViewerInfoMixin
from spark_agent.simulation.viewer_config import (
    camera_eye_target,
    normalize_sensor_camera_config,
    normalize_viewer_config,
)
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import VizColor


def _power_of_two_capacity(required: int, minimum: int) -> int:
    """Round a PhysX GPU buffer requirement up to a practical capacity."""

    required = max(int(required), int(minimum), 1)
    return 1 << (required - 1).bit_length()


def _fit_grid_viewer_config(
    viewer_config: dict,
    env_positions,
    *,
    preserve_orientation: bool = False,
) -> dict:
    """Expand a single-robot camera fit to include a cloned environment grid."""

    config = dict(viewer_config)
    positions = np.asarray(env_positions, dtype=float)
    if positions.ndim != 2 or positions.shape[0] <= 1 or positions.shape[1] < 2:
        return config
    minimum = positions[:, :2].min(axis=0)
    maximum = positions[:, :2].max(axis=0)
    center = 0.5 * (minimum + maximum)
    planar_diagonal = float(np.linalg.norm(maximum - minimum))
    vertical_fov = math.radians(float(config["camera_vertical_fov"]))
    tangent = math.tan(0.5 * vertical_fov)
    # fit_viewer_config_to_prim chooses distance=robot_extent/tan(fov/2).
    # Recover that extent, add the clone-grid diagonal, and retain a small
    # documentation margin around the projected scene.
    robot_extent = float(config["camera_distance"]) * tangent
    scene_extent = planar_diagonal + robot_extent
    required_distance = 1.35 * scene_extent / max(2.0 * tangent, 1.0e-6)
    lookat = list(config["camera_lookat"])
    lookat[:2] = center.tolist()
    config.update(
        camera_lookat=tuple(lookat),
        camera_distance=max(float(config["camera_distance"]), required_distance),
        grid_extent=max(
            float(config["grid_extent"]),
            0.75 * (float(np.max(maximum - minimum)) + robot_extent),
        ),
    )
    if not preserve_orientation:
        # Avoid looking exactly along a square grid diagonal, which projects
        # two clone pairs on top of one another. Benchmark callers can retain
        # their MuJoCo camera orientation while still using the grid fit.
        config.update(camera_azimuth=120.0, camera_elevation=-45.0)
    return config


class ConfiguredIsaacTensorAgent(IsaacViewerInfoMixin, BaseAgent):
    """Run a declarative articulation in cloned Isaac environments.

    The core Isaac articulation view is used instead of an Isaac Lab asset
    wrapper. This keeps fixed industrial URDFs compatible with Isaac Sim 6 and
    avoids host round-trips for state, command integration, and feedback.
    """

    def __init__(
        self,
        robot_cfg,
        *,
        num_envs: int = 1,
        env_spacing: float = 2.5,
        device: str = "cuda:0",
        render: bool = False,
        enable_viewer: bool | None = None,
        render_on_step: bool = True,
        render_decimation: int = 1,
        asset_cache_dir: str | None = None,
        urdf_path: str | None = None,
        allow_self_collision: bool | None = None,
        stiffness: float | None = None,
        damping: float | None = None,
        sim_position_error_limit: float | None = 0.05,
        dt: float | None = None,
        control_decimation: int | None = None,
        dynamics_backend: str = "simulator",
        use_sim_dynamics: bool | None = None,
        scalar_api: bool = False,
        viewer_config: dict | None = None,
        preserve_viewer_orientation: bool = False,
        **kwargs,
    ) -> None:
        super().__init__(robot_cfg)
        if num_envs < 1:
            raise ValueError("num_envs must be positive")
        if scalar_api and num_envs != 1:
            raise ValueError("scalar_api requires num_envs=1")
        spec = getattr(robot_cfg, "isaac_articulation", None)
        if spec is None:
            raise ValueError(
                f"{type(robot_cfg).__name__} does not declare isaac_articulation metadata"
            )
        self.allow_self_collision = (
            spec.allow_self_collision
            if allow_self_collision is None
            else bool(allow_self_collision)
        )

        # Isaac imports must occur after the application is initialized.
        import torch
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.cloner import GridCloner
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        self._torch = torch
        self.num_envs = int(num_envs)
        self.env_spacing = float(env_spacing)
        self.device = str(device)
        self.render_enabled = bool(render if enable_viewer is None else enable_viewer)
        self._initialize_viewer_simulation_info(**kwargs)
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
        self.render_on_step = bool(render_on_step)
        self.render_decimation = max(1, int(render_decimation))
        self.scalar_api = bool(scalar_api)
        resolved_viewer_config = dict(viewer_config or {})
        if viewer_config is None and self.num_envs > 1:
            grid_side = math.ceil(math.sqrt(self.num_envs))
            grid_extent = max(2.5, (grid_side - 1) * self.env_spacing)
            resolved_viewer_config.update(
                camera_lookat=(0.0, 0.0, 0.8),
                camera_distance=1.45 * grid_extent,
                camera_azimuth=135.0,
                camera_elevation=-50.0,
                grid_extent=max(5.0, 0.75 * grid_extent),
            )
        self.viewer_config = normalize_viewer_config(resolved_viewer_config)
        self.camera_eye, self.camera_target = camera_eye_target(self.viewer_config)
        self.enable_camera = bool(kwargs.get("enable_camera", False))
        self.camera_width = int(kwargs.get("camera_width", 640))
        self.camera_height = int(kwargs.get("camera_height", 480))
        self.camera_rate_hz = float(kwargs.get("camera_rate_hz", 30.0))
        self.camera_display = bool(kwargs.get("camera_display", False))
        self._camera_config_input = kwargs.get("camera_config")
        self.camera_config = {}
        self.camera_feedback = {}
        self._sensor_cameras = {}
        self._camera_display_process = None
        self._camera_ready_reported = False
        self._last_camera_capture_time = -np.inf
        self._viewport_layout_updates_remaining = int(kwargs.get("viewport_layout_updates", 0))
        self._simulation_app = None
        self._closed = False
        self._viewport_interaction_guards = []
        self._has_committed_initial_reset = False
        self.step_index = 0
        self.time = 0.0
        self._debug_root = "/World/SparkTensorDebug"
        self._debug_primitives = []
        self._debug_solid_cache = {"sphere": [], "box": []}
        self._debug_prim_count = 0

        if use_sim_dynamics is not None:
            dynamics_backend = "simulator" if use_sim_dynamics else "model"
        self.dynamics_backend = str(dynamics_backend).lower()
        if self.dynamics_backend not in {"model", "simulator"}:
            raise ValueError("dynamics_backend must be 'model' or 'simulator'")
        self.use_sim_dynamics = self.dynamics_backend == "simulator"

        simulator_spec = robot_cfg.simulator_dynamics
        self.dt = float(simulator_spec.physics_dt if dt is None else dt)
        self.control_decimation = int(
            simulator_spec.control_decimation if control_decimation is None else control_decimation
        )
        if self.dt <= 0.0 or self.control_decimation < 1:
            raise ValueError("dt and control_decimation must be positive")
        self.control_period = self.dt * self.control_decimation
        self.drive_stiffness = float(spec.stiffness if stiffness is None else stiffness)
        self.drive_damping = float(spec.damping if damping is None else damping)
        self.sim_position_error_limit = None
        if sim_position_error_limit is not None:
            self.sim_position_error_limit = float(sim_position_error_limit)
            if self.sim_position_error_limit <= 0.0:
                raise ValueError("sim_position_error_limit must be positive or None")
        self.joint_gain_overrides = dict(
            (name, (float(joint_stiffness), float(joint_damping)))
            for name, joint_stiffness, joint_damping in spec.joint_gain_overrides
        )

        source = Path(urdf_path or spec.urdf_path).expanduser()
        if not source.is_absolute():
            source = Path(SPARK_ROBOT_RESOURCE_DIR) / source
        source = Path(
            apply_urdf_visual_palette(
                source,
                getattr(robot_cfg, "visual_link_colors", ()),
                cache_dir=asset_cache_dir,
            )
        )
        if spec.instances:
            source = Path(compose_urdf_instances(source, spec.instances, cache_dir=asset_cache_dir))
        if spec.planar_base is not None:
            source = Path(
                augment_urdf_planar_base(
                    source,
                    spec.planar_base,
                    cache_dir=asset_cache_dir,
                )
            )
        source = source.resolve()
        resolved_usd = import_urdf_to_usd(
            source,
            fix_base=spec.fixed_base,
            merge_fixed_joints=spec.merge_fixed_joints,
            allow_self_collision=self.allow_self_collision,
            cache_dir=asset_cache_dir,
        )
        urdf_root = ET.parse(source)
        movable_joints = {
            joint.attrib["name"]: joint
            for joint in urdf_root.findall(".//joint")
            if joint.attrib.get("type") != "fixed"
        }
        missing = [name for name in spec.joint_names if name not in movable_joints]
        if missing:
            raise ValueError(f"Isaac URDF is missing configured joints: {missing}")

        # Contact-pair counts grow approximately linearly for isolated cloned
        # robot environments.  A 2x margin keeps the 1k-environment stress
        # gate below capacity without making small jobs reserve Isaac Lab's
        # maximum aggregate buffers.
        pair_capacity = _power_of_two_capacity(self.num_envs * 1024, 2**18)
        # PhysX counts aggregate *shape* pairs here, not merely articulation
        # pairs. A 16,384-slot allowance per environment, rounded upward,
        # gives the validated 1,024-clone stress case a deterministic margin
        # without imposing that cost on small scenes.
        aggregate_capacity = _power_of_two_capacity(self.num_envs * 16384, 2**18)
        self.sim = World(
            physics_dt=self.dt,
            rendering_dt=self.dt * self.control_decimation,
            backend="torch",
            device=self.device,
            sim_params={
                # Isaac Core defaults target small scenes. Scale these GPU
                # buffers with the clone count so 1k+ articulations do not
                # silently drop broad-phase interactions while small jobs
                # retain a modest memory footprint.
                "gpu_found_lost_pairs_capacity": pair_capacity,
                "gpu_found_lost_aggregate_pairs_capacity": aggregate_capacity,
                "gpu_total_aggregate_pairs_capacity": pair_capacity,
                "gpu_collision_stack_size": 2**26,
            },
        )
        self.sim.scene.add_default_ground_plane(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        )
        stage = omni.usd.get_context().get_stage()
        source_env_path = "/World/envs/env_0"
        UsdGeom.Xform.Define(stage, source_env_path)
        add_reference_to_stage(resolved_usd, f"{source_env_path}/Robot")
        source_robot_path = f"{source_env_path}/Robot"
        articulation_root = next(
            (
                prim
                for prim in stage.Traverse()
                if (
                    str(prim.GetPath()) == source_robot_path
                    or str(prim.GetPath()).startswith(f"{source_robot_path}/")
                )
                and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            ),
            None,
        )
        if articulation_root is None:
            raise RuntimeError(f"No articulation root found below {source_robot_path}")
        PhysxSchema.PhysxArticulationAPI.Apply(
            articulation_root
        ).CreateEnabledSelfCollisionsAttr().Set(self.allow_self_collision)
        cloner = GridCloner(spacing=self.env_spacing)
        cloner.define_base_env(source_env_path)
        env_paths = cloner.generate_paths("/World/envs/env", self.num_envs)
        self.env_positions = self._torch.as_tensor(
            cloner.clone(
                source_prim_path=source_env_path,
                prim_paths=env_paths,
                replicate_physics=True,
                base_env_path="/World/envs",
            ),
            device=self.device,
            dtype=self._torch.float32,
        )
        cloner.filter_collisions(
            physicsscene_path="/physicsScene",
            collision_root_path="/World/collisions",
            prim_paths=env_paths,
            global_paths=["/World/defaultGroundPlane"],
        )
        self.articulation = self.sim.scene.add(
            Articulation(
                prim_paths_expr="/World/envs/env_.*/Robot",
                name="spark_parallel_robots",
                reset_xform_properties=False,
            )
        )
        self.sim.reset()
        root_positions, root_orientations = self.articulation.get_world_poses()
        if self.num_envs > 1:
            observed_offsets = root_positions - root_positions[0]
            declared_offsets = self.env_positions - self.env_positions[0]
            if not self._torch.allclose(
                observed_offsets,
                declared_offsets,
                rtol=0.0,
                atol=1.0e-5,
            ):
                # Isaac Sim 6's fixed-base URDF replication can preserve the
                # cloned articulation data while collapsing every root to the
                # source world pose. Apply the GridCloner layout explicitly so
                # physics, Fabric rendering, and recorded clone grids agree.
                root_positions = root_positions[0].unsqueeze(0) + self.env_positions
                self.articulation.set_world_poses(root_positions, root_orientations)
        if any(abs(value) > 1.0e-12 for value in spec.base_translation):
            root_positions = root_positions + self._torch.as_tensor(
                spec.base_translation,
                device=self.device,
                dtype=self._torch.float32,
            )
            self.articulation.set_world_poses(root_positions, root_orientations)
        self.kinematic_origins = self.env_positions + self._torch.as_tensor(
            spec.base_translation,
            device=self.device,
            dtype=self._torch.float32,
        )
        if any(abs(value) > 1.0e-12 for value in spec.base_rpy):
            base_orientation = self._torch.as_tensor(
                _rpy_to_quaternion_wxyz(spec.base_rpy),
                device=self.device,
                dtype=self._torch.float32,
            ).expand(self.num_envs, -1)
            left_w, left_xyz = root_orientations[:, :1], root_orientations[:, 1:]
            right_w, right_xyz = base_orientation[:, :1], base_orientation[:, 1:]
            root_orientations = self._torch.cat(
                (
                    left_w * right_w - (left_xyz * right_xyz).sum(dim=1, keepdim=True),
                    left_w * right_xyz
                    + right_w * left_xyz
                    + self._torch.cross(left_xyz, right_xyz, dim=1),
                ),
                dim=1,
            )
            self.articulation.set_world_poses(root_positions, root_orientations)
        self.default_root_positions = root_positions.detach().clone()
        self.default_root_orientations = root_orientations.detach().clone()
        if viewer_config is None:
            self.viewer_config = fit_viewer_config_to_prim("/World/envs", self.viewer_config)
        if self.num_envs > 1:
            self.viewer_config = _fit_grid_viewer_config(
                self.viewer_config,
                self.env_positions.detach().cpu().numpy(),
                preserve_orientation=preserve_viewer_orientation,
            )
        self.camera_eye, self.camera_target = camera_eye_target(self.viewer_config)
        self.camera_config = normalize_sensor_camera_config(
            self._camera_config_input, self.viewer_config
        )
        if self.render_enabled:
            IsaacAgent._apply_mujoco_scene_style(
                self,
                self.camera_eye,
                self.camera_target,
                vertical_fov=self.viewer_config["camera_vertical_fov"],
                viewer_config=self.viewer_config,
            )
            IsaacAgent._disable_viewport_editing(self)
        IsaacAgent._hide_imported_guide_geometry("/World/envs")
        if self.enable_camera:
            IsaacAgent._initialize_sensor_cameras(self, "/World/envs/env_0/Robot")

        self.joint_names = tuple(self.articulation.dof_names)
        missing = [name for name in spec.joint_names if name not in self.joint_names]
        if missing:
            raise ValueError(f"Isaac articulation view is missing configured joints: {missing}")
        self.joint_indices = self._torch.as_tensor(
            [self.joint_names.index(name) for name in spec.joint_names],
            device=self.device,
            dtype=self._torch.long,
        )
        self.all_joint_indices = self._torch.arange(
            len(self.joint_names), device=self.device, dtype=self._torch.long
        )
        selected_defaults = [float(robot_cfg.DefaultDoFVal[dof]) for dof in robot_cfg.DoFs]
        self.default_dof_pos = self._torch.as_tensor(
            selected_defaults, device=self.device, dtype=self._torch.float32
        ).repeat(self.num_envs, 1)
        self.default_all_joint_pos = self.articulation.get_joint_positions().detach().clone()
        missing_defaults = [
            name for name, _position in spec.joint_position_defaults if name not in self.joint_names
        ]
        if missing_defaults:
            raise ValueError(
                f"Isaac articulation is missing joints with declared defaults: {missing_defaults}"
            )
        for name, position in spec.joint_position_defaults:
            self.default_all_joint_pos[:, self.joint_names.index(name)] = float(position)
        self.default_all_joint_pos[:, self.joint_indices] = self.default_dof_pos
        for gripper in spec.grippers:
            if not all(name in self.joint_names for name in gripper.joint_names):
                continue
            indices = self._torch.as_tensor(
                [self.joint_names.index(name) for name in gripper.joint_names],
                device=self.device,
                dtype=self._torch.long,
            )
            self.default_all_joint_pos[:, indices] = self._torch.as_tensor(
                gripper.open_positions,
                device=self.device,
                dtype=self._torch.float32,
            )
        self.command_pos = self.default_dof_pos.clone()
        self.command_vel = self._torch.zeros_like(self.command_pos)
        self.last_control = self._torch.zeros(
            (self.num_envs, self.num_control),
            device=self.device,
            dtype=self._torch.float32,
        )
        gains_shape = (self.num_envs, len(self.joint_names))
        kps = self._torch.full(
            gains_shape,
            self.drive_stiffness,
            device=self.device,
            dtype=self._torch.float32,
        )
        kds = self._torch.full(
            gains_shape,
            self.drive_damping,
            device=self.device,
            dtype=self._torch.float32,
        )
        missing_gain_overrides = [
            name for name in self.joint_gain_overrides if name not in self.joint_names
        ]
        if missing_gain_overrides:
            raise ValueError(
                "Isaac articulation is missing joints with declared gain overrides: "
                f"{missing_gain_overrides}"
            )
        for name, (stiffness, damping) in self.joint_gain_overrides.items():
            index = self.joint_names.index(name)
            kps[:, index] = stiffness
            kds[:, index] = damping
        self.articulation.set_gains(
            kps=kps,
            kds=kds,
        )

        limits = []
        for name in spec.joint_names:
            limit = movable_joints[name].find("limit")
            limits.append(
                (
                    float(limit.attrib.get("lower", "-inf")) if limit is not None else -math.inf,
                    float(limit.attrib.get("upper", "inf")) if limit is not None else math.inf,
                )
            )
        self.position_lower = self._torch.as_tensor(
            [item[0] for item in limits], device=self.device, dtype=self._torch.float32
        )
        self.position_upper = self._torch.as_tensor(
            [item[1] for item in limits], device=self.device, dtype=self._torch.float32
        )
        self.dynamics_model = robot_cfg.create_dynamics_model()
        self.active_control_names = set(self.dynamics_model.control_names)
        self.control_index_by_name = {control.name: int(control) for control in robot_cfg.Control}
        self.dof_index_by_name = {dof.name: int(dof) for dof in robot_cfg.DoFs}
        prefix = "v" if self.dynamics_model.order == 1 else "a"
        self.control_to_dof = [
            (control_index, self.dof_index_by_name[control_name.removeprefix(prefix)])
            for control_name, control_index in self.control_index_by_name.items()
            if control_name in self.active_control_names
            and control_name.startswith(prefix)
            and control_name.removeprefix(prefix) in self.dof_index_by_name
        ]
        self.planar_indices = self._resolve_planar_indices(prefix)
        self.reset()

    def _resolve_planar_indices(self, prefix: str):
        names = ("LinearX", "LinearY", "RotYaw")
        controls = tuple(prefix + name for name in names)
        if not all(name in self.dof_index_by_name for name in names):
            return None
        if not all(name in self.control_index_by_name for name in controls):
            return None
        return (
            tuple(self.dof_index_by_name[name] for name in names),
            tuple(self.control_index_by_name[name] for name in controls),
        )

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

    def reset(self, agent_reset_info=None, env_ids=None, **kwargs) -> None:
        del agent_reset_info, kwargs
        if env_ids is None:
            env_ids = self._torch.arange(self.num_envs, device=self.device, dtype=self._torch.long)
        else:
            env_ids = self._torch.as_tensor(env_ids, device=self.device, dtype=self._torch.long)
        joint_pos = self.default_all_joint_pos[env_ids]
        joint_vel = self._torch.zeros_like(joint_pos)
        self.articulation.set_world_poses(
            self.default_root_positions[env_ids],
            self.default_root_orientations[env_ids],
            indices=env_ids,
        )
        self.articulation.set_joint_positions(joint_pos, indices=env_ids)
        self.articulation.set_joint_velocities(joint_vel, indices=env_ids)
        self.command_pos[env_ids] = self.default_dof_pos[env_ids]
        self.command_vel[env_ids] = 0.0
        self.last_control[env_ids] = 0.0
        self._set_targets(env_ids)
        if not self._has_committed_initial_reset:
            # World has no separate forward-kinematics call before its first
            # tensor step in Isaac Sim 6.  Commit the initial articulation
            # once, then restore the exact state after gravity was evaluated.
            self.sim.step(render=False)
            self.articulation.set_world_poses(
                self.default_root_positions[env_ids],
                self.default_root_orientations[env_ids],
                indices=env_ids,
            )
            self.articulation.set_joint_positions(joint_pos, indices=env_ids)
            self.articulation.set_joint_velocities(joint_vel, indices=env_ids)
            self._set_targets(env_ids)
            self._has_committed_initial_reset = True
        if not self.use_sim_dynamics:
            # The initialization step can move a gravity-loaded articulation.
            # Model-owned execution is an explicit kinematic projection, so
            # restore its exact state after PhysX has created the tensor view.
            self._set_targets(env_ids)
            physics_view = getattr(self.sim, "physics_sim_view", None)
            if physics_view is not None:
                physics_view.update_articulations_kinematic()
        if env_ids.numel() == self.num_envs:
            self.step_index = 0
            self.time = 0.0

    def _set_targets(self, env_ids=None) -> None:
        if env_ids is None:
            env_ids = self._torch.arange(self.num_envs, device=self.device, dtype=self._torch.long)
        target = self.default_all_joint_pos[env_ids].clone()
        target[:, self.joint_indices] = self.command_pos[env_ids]
        if not self.use_sim_dynamics:
            velocity = self._torch.zeros_like(target)
            velocity[:, self.joint_indices] = self.command_vel[env_ids]
            self.articulation.set_joint_positions(target, indices=env_ids)
            self.articulation.set_joint_velocities(velocity, indices=env_ids)
            return
        self.articulation.set_joint_position_targets(target, indices=env_ids)
        target_velocity = self._torch.zeros_like(target)
        target_velocity[:, self.joint_indices] = self.command_vel[env_ids]
        self.articulation.set_joint_velocity_targets(target_velocity, indices=env_ids)

    def _synchronize_command_state_from_simulator(
        self, active_mask=None, hold_dof_indices=None
    ) -> None:
        """Anchor simulator-owned integration to measured articulation state.

        A physical drive can lag its requested position. Continuing to
        integrate velocity commands from the previous request accumulates a
        target beyond the intended pose, which appears as overshoot when the
        articulation catches up. MuJoCo's simulator-owned adapters already
        integrate from feedback for this reason; keep the shared Isaac tensor
        adapter under the same state-ownership contract.
        """

        measured_position = self.articulation.get_joint_positions(joint_indices=self.joint_indices)
        if active_mask is None:
            active_mask = self._torch.ones(
                self.num_envs, device=self.device, dtype=self._torch.bool
            )
        else:
            active_mask = self._torch.as_tensor(
                active_mask, device=self.device, dtype=self._torch.bool
            ).reshape(self.num_envs)
        if hold_dof_indices is None:
            hold_dof_indices = self._torch.empty(0, device=self.device, dtype=self._torch.long)
        else:
            hold_dof_indices = self._torch.as_tensor(
                hold_dof_indices, device=self.device, dtype=self._torch.long
            ).reshape(-1)
        held_position = self.command_pos[:, hold_dof_indices].clone()
        self.command_pos[active_mask] = measured_position[active_mask]
        self.command_pos[:, hold_dof_indices] = held_position
        if self.dynamics_model.order == 2:
            measured_velocity = self.articulation.get_joint_velocities(
                joint_indices=self.joint_indices
            )
            self.command_vel[active_mask] = measured_velocity[active_mask]
            self.command_vel[~active_mask] = 0.0
            self.command_vel[:, hold_dof_indices] = 0.0

    def _bound_simulator_command_state(self, active_mask=None, hold_dof_indices=None) -> None:
        """Bound tensor drive preload without discarding gravity support.

        The scalar configured Isaac adapter retains its integrated
        target and caps its distance from measured feedback. This applies to
        both dynamics orders: re-anchoring a zero-velocity first-order target
        to feedback on every cycle ratchets a gravity-loaded arm downward.
        Re-anchoring the target itself on every CUDA cycle removes the
        position-drive preload required to support a gravity-loaded arm and
        leaves a persistent tracking error.  Apply the same bounded-target
        contract to cloned articulations.
        """

        measured_position = self.articulation.get_joint_positions(joint_indices=self.joint_indices)
        if active_mask is None:
            active_mask = self._torch.ones(
                self.num_envs, device=self.device, dtype=self._torch.bool
            )
        else:
            active_mask = self._torch.as_tensor(
                active_mask, device=self.device, dtype=self._torch.bool
            ).reshape(self.num_envs)
        if self.sim_position_error_limit is not None:
            error = self._torch.clamp(
                self.command_pos - measured_position,
                -self.sim_position_error_limit,
                self.sim_position_error_limit,
            )
            self.command_pos[active_mask] = measured_position[active_mask] + error[active_mask]
        self.command_vel[~active_mask] = 0.0
        if hold_dof_indices is not None:
            hold_dof_indices = self._torch.as_tensor(
                hold_dof_indices, device=self.device, dtype=self._torch.long
            ).reshape(-1)
            self.command_pos[:, hold_dof_indices] = measured_position[:, hold_dof_indices]
            self.command_vel[:, hold_dof_indices] = 0.0

    def _advance_first_order(self, control) -> None:
        derivative = self._torch.zeros_like(self.command_pos)
        for control_index, dof_index in self.control_to_dof:
            derivative[:, dof_index] = control[:, control_index]
        if (
            self.planar_indices is not None
            and getattr(self.robot_cfg, "dynamics_variant", "") == "unicycle"
        ):
            (x_index, y_index, yaw_index), (vx_index, _, yaw_rate_index) = self.planar_indices
            yaw = self.command_pos[:, yaw_index]
            speed = control[:, vx_index]
            derivative[:, x_index] = self._torch.cos(yaw) * speed
            derivative[:, y_index] = self._torch.sin(yaw) * speed
            derivative[:, yaw_index] = control[:, yaw_rate_index]
        elif self.planar_indices is not None:
            (x_index, y_index, yaw_index), (vx_index, vy_index, yaw_rate_index) = (
                self.planar_indices
            )
            yaw = self.command_pos[:, yaw_index]
            cosine = self._torch.cos(yaw)
            sine = self._torch.sin(yaw)
            vx = control[:, vx_index]
            vy = control[:, vy_index]
            derivative[:, x_index] = cosine * vx - sine * vy
            derivative[:, y_index] = sine * vx + cosine * vy
            derivative[:, yaw_index] = control[:, yaw_rate_index]
        self.command_pos += self.control_period * derivative
        self.command_vel.copy_(derivative)

    def _advance_second_order(self, control) -> None:
        acceleration = self._torch.zeros_like(self.command_vel)
        for control_index, dof_index in self.control_to_dof:
            acceleration[:, dof_index] = control[:, control_index]
        if (
            self.planar_indices is not None
            and getattr(self.robot_cfg, "dynamics_variant", "") == "bicycle"
        ):
            (x_index, y_index, yaw_index), (ax_index, _, steering_index) = self.planar_indices
            p = self.robot_cfg.bicycle_params
            yaw = self.command_pos[:, yaw_index]
            cosine = self._torch.cos(yaw)
            sine = self._torch.sin(yaw)
            vx_world = self.command_vel[:, x_index]
            vy_world = self.command_vel[:, y_index]
            yaw_rate = self.command_vel[:, yaw_index]
            vx = cosine * vx_world + sine * vy_world
            vy = -sine * vx_world + cosine * vy_world
            minimum_speed = self._torch.full_like(vx, p.minimum_forward_speed)
            vx_effective = self._torch.where(
                vx == 0.0,
                minimum_speed,
                self._torch.sign(vx) * self._torch.maximum(self._torch.abs(vx), minimum_speed),
            )
            steering = self._torch.clamp(
                control[:, steering_index], -p.steering_limit, p.steering_limit
            )
            force_x = p.mass * control[:, ax_index]
            alpha_front = self._torch.atan2(vy + p.front_length * yaw_rate, vx_effective) - steering
            alpha_rear = self._torch.atan2(vy - p.rear_length * yaw_rate, vx_effective)
            force_y_front = -p.front_cornering_stiffness * alpha_front
            force_y_rear = -p.rear_cornering_stiffness * alpha_rear
            force_x_front = p.driven_front_fraction * force_x
            force_x_rear = (1.0 - p.driven_front_fraction) * force_x
            dvx = (
                force_x_front * self._torch.cos(steering)
                - force_y_front * self._torch.sin(steering)
                + force_x_rear
            ) / p.mass + yaw_rate * vy
            dvy = (
                force_x_front * self._torch.sin(steering)
                + force_y_front * self._torch.cos(steering)
                + force_y_rear
            ) / p.mass - yaw_rate * vx
            d_yaw_rate = (
                p.front_length
                * (
                    force_x_front * self._torch.sin(steering)
                    + force_y_front * self._torch.cos(steering)
                )
                - p.rear_length * force_y_rear
            ) / p.yaw_inertia
            acceleration[:, x_index] = (
                cosine * dvx - sine * dvy - yaw_rate * (sine * vx + cosine * vy)
            )
            acceleration[:, y_index] = (
                sine * dvx + cosine * dvy + yaw_rate * (cosine * vx - sine * vy)
            )
            acceleration[:, yaw_index] = d_yaw_rate
        elif self.planar_indices is not None:
            (x_index, y_index, yaw_index), (ax_index, ay_index, yaw_acc_index) = self.planar_indices
            yaw = self.command_pos[:, yaw_index]
            cosine = self._torch.cos(yaw)
            sine = self._torch.sin(yaw)
            vx_world = self.command_vel[:, x_index]
            vy_world = self.command_vel[:, y_index]
            yaw_rate = self.command_vel[:, yaw_index]
            vx_local = cosine * vx_world + sine * vy_world
            vy_local = -sine * vx_world + cosine * vy_world
            acceleration[:, x_index] = (
                -vx_local * yaw_rate * sine
                - vy_local * yaw_rate * cosine
                + cosine * control[:, ax_index]
                - sine * control[:, ay_index]
            )
            acceleration[:, y_index] = (
                vx_local * yaw_rate * cosine
                - vy_local * yaw_rate * sine
                + sine * control[:, ax_index]
                + cosine * control[:, ay_index]
            )
            acceleration[:, yaw_index] = control[:, yaw_acc_index]
        if (
            self.use_sim_dynamics
            and getattr(getattr(self, "robot_cfg", None), "dynamics_variant", "") == "bicycle"
        ):
            # A planar bicycle has no gravity-supporting position preload.
            # Convert its force-equivalent acceleration into a PhysX drive
            # displacement (force / stiffness) after re-anchoring to measured
            # state. A time-integrated micro-target under-commands the heavy
            # base, while retaining it creates steady preload and overshoot.
            if self.planar_indices is None:
                self.command_vel += self.control_period * acceleration
                self.command_pos += self.control_period * self.command_vel
            else:
                (x_index, y_index, yaw_index), _ = self.planar_indices
                p = self.robot_cfg.bicycle_params
                stiffness = max(self.drive_stiffness, 1.0e-6)
                self.command_pos[:, x_index] += p.mass * acceleration[:, x_index] / stiffness
                self.command_pos[:, y_index] += p.mass * acceleration[:, y_index] / stiffness
                self.command_pos[:, yaw_index] += (
                    p.yaw_inertia * acceleration[:, yaw_index] / stiffness
                )
        else:
            self.command_pos += self.control_period * self.command_vel
            self.command_vel += self.control_period * acceleration

    def send_control(self, control, **kwargs) -> None:
        active_mask = kwargs.get("active_mask")
        hold_dof_indices = kwargs.get("hold_dof_indices")
        control = self._as_batch(control, self.num_control, "control")
        self.last_control.copy_(control)
        held_position = None
        if hold_dof_indices is not None:
            hold_dof_indices = self._torch.as_tensor(
                hold_dof_indices, device=self.device, dtype=self._torch.long
            ).reshape(-1)
            if hold_dof_indices.numel():
                # A held joint must retain its drive target across the complete
                # simulator-control update.  Replacing that target with the
                # measured position after every step ratchets gravity sag into
                # the command and can eventually fold an otherwise supported
                # torso onto the mobile base.
                held_position = self.command_pos[:, hold_dof_indices].clone()
        bicycle_simulator = (
            self.use_sim_dynamics
            and getattr(getattr(self, "robot_cfg", None), "dynamics_variant", "") == "bicycle"
        )
        if self.use_sim_dynamics and bicycle_simulator:
            self._synchronize_command_state_from_simulator(
                active_mask, hold_dof_indices=hold_dof_indices
            )
        if self.dynamics_model.order == 1:
            self._advance_first_order(control)
        else:
            self._advance_second_order(control)
        if self.use_sim_dynamics:
            self._bound_simulator_command_state(active_mask, hold_dof_indices=hold_dof_indices)
        if held_position is not None:
            self.command_pos[:, hold_dof_indices] = held_position
            self.command_vel[:, hold_dof_indices] = 0.0
        self.command_pos.copy_(
            self._torch.clamp(self.command_pos, self.position_lower, self.position_upper)
        )
        self._set_targets()

    def _post_control_processing(self, **kwargs) -> None:
        del kwargs
        if self.use_sim_dynamics:
            for _ in range(self.control_decimation):
                self.sim.step(render=False)
        else:
            physics_view = getattr(self.sim, "physics_sim_view", None)
            if physics_view is not None:
                physics_view.update_articulations_kinematic()
        if (
            self.render_enabled
            and self.render_on_step
            and self.step_index % self.render_decimation == 0
        ):
            self._update_simulation_info_overlay()
            self.sim.render()
            if self.enable_camera:
                IsaacAgent._capture_camera_feedback(self)
            if self._viewport_layout_updates_remaining > 0:
                _show_viewport_only()
                self._viewport_layout_updates_remaining -= 1
        self.step_index += 1
        self.time += self.control_period

    def get_feedback(self):
        dof_pos = self.articulation.get_joint_positions(joint_indices=self.joint_indices)
        dof_vel = self.articulation.get_joint_velocities(joint_indices=self.joint_indices)
        state = (
            dof_pos
            if self.dynamics_model.order == 1
            else self._torch.cat((dof_pos, dof_vel), dim=-1)
        )
        root_position, root_orientation = self.articulation.get_world_poses()
        feedback = {
            "state": state,
            "time": self.time,
            "step_index": self.step_index,
            "dof_pos_fbk": dof_pos,
            "dof_vel_fbk": dof_vel,
            "dof_pos_cmd": self.command_pos,
            "dof_vel_cmd": self.command_vel,
            "control": self.last_control,
            "root_position_w": root_position,
            "root_orientation_wxyz": root_orientation,
        }
        if not self.scalar_api:
            return feedback
        scalar_feedback = {
            key: (
                value[0].detach().cpu().numpy().astype(float, copy=True)
                if hasattr(value, "ndim") and value.ndim > 0
                else value
            )
            for key, value in feedback.items()
        }
        scalar_feedback["camera_feedback"] = IsaacAgent.get_camera_feedback(self)
        return scalar_feedback

    def get_body_kinematics(self, *, include_jacobian: bool = True):
        """Return batched link poses and optional Jacobians in the tensor contract."""

        body_names = tuple(self.articulation.body_names)
        transforms = self.articulation._physics_view.get_link_transforms()
        transforms = transforms.reshape(self.num_envs, len(body_names), 7)
        result = {
            "body_names": body_names,
            "joint_names": tuple(self.joint_names),
            "body_position_w": transforms[..., :3],
            "body_quaternion_w": transforms[..., 3:7],
            "body_joint_ids": self.joint_indices,
        }
        if not include_jacobian:
            return result
        jacobian = self.articulation.get_jacobians(clone=False)
        # PhysX omits the fixed articulation-root row from its Jacobian tensor
        # while retaining that root in link transforms/body names.  Pad it so
        # collision models can use one stable body index for both tensors.
        if jacobian.shape[1] == len(body_names) - 1:
            root = self._torch.zeros(
                self.num_envs,
                1,
                jacobian.shape[2],
                jacobian.shape[3],
                device=jacobian.device,
                dtype=jacobian.dtype,
            )
            jacobian = self._torch.cat((root, jacobian), dim=1)
        if jacobian.shape[1] != len(body_names):
            raise RuntimeError(
                "Isaac link/Jacobian layout mismatch: "
                f"{len(body_names)} bodies versus {jacobian.shape[1]} Jacobian rows"
            )
        result["body_jacobian_w"] = jacobian
        return result

    @staticmethod
    def _rgba(color):
        return IsaacAgent._rgba(color)

    def render_sphere(self, pos, mat, size, color) -> None:
        IsaacAgent.render_sphere(self, pos, mat, size, color)

    def render_box(self, pos, mat, size, color) -> None:
        IsaacAgent.render_box(self, pos, mat, size, color)

    def render_line_segment(self, pos1, pos2, radius=0.002, color=(0.1, 1.0, 0.2, 1.0)):
        IsaacAgent.render_line_segment(self, pos1, pos2, radius=radius, color=color)

    def set_visual_obstacles(self, local_positions, radius: float = 0.05, color=None) -> None:
        """Create or move persistent per-clone benchmark obstacle markers."""

        if not self.render_enabled:
            return
        import omni.usd
        from pxr import Gf, UsdGeom, Vt

        positions = np.asarray(local_positions, dtype=float)
        if positions.ndim == 2:
            positions = np.broadcast_to(positions[None], (self.num_envs, *positions.shape))
        if positions.ndim != 3 or positions.shape[0] != self.num_envs or positions.shape[2] != 3:
            raise ValueError(
                "local_positions must have shape (num_obstacles, 3) or "
                f"({self.num_envs}, num_obstacles, 3)"
            )
        rgba = np.asarray(VizColor.obstacle_task if color is None else color, dtype=float)
        if rgba.shape != (4,):
            raise ValueError("obstacle color must be RGBA")

        stage = omni.usd.get_context().get_stage()
        origins = self.env_positions.detach().cpu().numpy()
        cache = getattr(self, "_visual_obstacle_cache", None)
        if cache is None:
            cache = {}
            self._visual_obstacle_cache = cache
        active_keys = set()
        for env_index, origin in enumerate(origins):
            for obstacle_index, local_position in enumerate(positions[env_index]):
                key = (env_index, obstacle_index)
                active_keys.add(key)
                entry = cache.get(key)
                if entry is None:
                    sphere = UsdGeom.Sphere.Define(
                        stage,
                        f"/World/SparkBenchmarkObstacles/env_{env_index}/obstacle_{obstacle_index}",
                    )
                    radius_attr = sphere.CreateRadiusAttr()
                    color_attr = sphere.CreateDisplayColorAttr()
                    opacity_attr = sphere.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant)
                    xform = UsdGeom.Xformable(sphere.GetPrim())
                    xform.ClearXformOpOrder()
                    entry = {
                        "prim": sphere.GetPrim(),
                        "radius": radius_attr,
                        "color": color_attr,
                        "opacity": opacity_attr,
                        "translate": xform.AddTranslateOp(),
                    }
                    cache[key] = entry
                UsdGeom.Imageable(entry["prim"]).MakeVisible()
                entry["radius"].Set(float(radius))
                entry["color"].Set(Vt.Vec3fArray([Gf.Vec3f(*rgba[:3])]))
                entry["opacity"].Set([float(rgba[3])])
                entry["translate"].Set(Gf.Vec3d(*(origin + local_position)))
        for key, entry in cache.items():
            if key not in active_keys:
                UsdGeom.Imageable(entry["prim"]).MakeInvisible()

    def set_visual_safety_constraints(
        self,
        witness_robot,
        witness_other,
        *,
        trigger_mask=None,
        violation_mask=None,
    ) -> None:
        """Stage batched closest-pair lines using the scalar SPARK colors."""

        if not self.render_enabled:
            return
        start = np.asarray(witness_robot, dtype=float)
        end = np.asarray(witness_other, dtype=float)
        if start.shape != end.shape or start.ndim != 3 or start.shape[-1] != 3:
            raise ValueError("safety witness points must share shape [num_envs, constraints, 3]")
        if start.shape[0] != self.num_envs:
            raise ValueError("safety witness batch must match num_envs")

        def validated_mask(name, value):
            if value is None:
                return None
            selected = np.asarray(value, dtype=bool)
            if selected.shape != start.shape[:2]:
                raise ValueError(f"{name}_mask must have shape {start.shape[:2]}")
            return selected

        self._visual_safety_constraints = (
            start,
            end,
            validated_mask("trigger", trigger_mask),
            validated_mask("violation", violation_mask),
        )

    def _flush_visual_safety_constraints(self) -> None:
        """Draw staged lines after generic primitives have cleared debug draw."""

        visual = getattr(self, "_visual_safety_constraints", None)
        if visual is None:
            return
        start, end, trigger_mask, violation_mask = visual
        from isaacsim.util.debug_draw import _debug_draw

        draw = getattr(self, "_safety_debug_draw", None)
        if draw is None:
            draw = _debug_draw.acquire_debug_draw_interface()
            self._safety_debug_draw = draw
        # DebugDraw retains lines across frames.  Clear the previous witness
        # set before drawing this frame so reset/resampling cannot leave a
        # segment pointing at an obstacle that no longer exists there.
        draw.clear_lines()
        line_start, line_end, line_colors, line_widths = [], [], [], []

        def append_category(mask, color):
            if mask is None:
                return
            selected_start = start[mask]
            selected_end = end[mask]
            count = selected_start.shape[0]
            line_start.extend(map(tuple, selected_start.tolist()))
            line_end.extend(map(tuple, selected_end.tolist()))
            line_colors.extend([(*color, 1.0)] * count)
            line_widths.extend([1.0] * count)

        # Match pipeline.visualization: unsafe/trigger blue and slack purple.
        append_category(trigger_mask, (0.0, 32.0 / 255.0, 230.0 / 255.0))
        append_category(violation_mask, (160.0 / 255.0, 32.0 / 255.0, 240.0 / 255.0))
        if line_start:
            draw.draw_lines(line_start, line_end, line_colors, line_widths)

    def render(self) -> None:
        """Flush backend-neutral benchmark primitives into the Isaac stage."""

        if not self.render_enabled:
            self._debug_primitives.clear()
            return
        IsaacAgent._flush_debug_primitives(self)
        self._flush_visual_safety_constraints()
        self._update_simulation_info_overlay()
        self.sim.render()
        if self.enable_camera:
            IsaacAgent._capture_camera_feedback(self)
        if self._viewport_layout_updates_remaining > 0:
            _show_viewport_only()
            self._viewport_layout_updates_remaining -= 1

    def attach_simulation_app(self, simulation_app) -> None:
        self._simulation_app = simulation_app

    def is_running(self) -> bool:
        return not self._closed and (
            self._simulation_app is None or self._simulation_app.is_running()
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._close_simulation_info_overlay()
        close_camera_display(self)
        self._sensor_cameras.clear()
        sim = self.sim
        if sim is not None:
            sim.stop()
            sim.clear()
        self.articulation = None
        self.sim = None
        try:
            from isaacsim.core.api import World

            World.clear_instance()
        except (ImportError, RuntimeError):
            pass
