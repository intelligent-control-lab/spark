"""Process-isolated Isaac renderer with bidirectional teleoperation state."""

from __future__ import annotations

import multiprocessing as mp
import time
import traceback

import numpy as np

from spark_agent.simulation.isaac.viewer_info import ISAAC_SIMULATION_INFO_WINDOW_TITLE


def _send(connection, value):
    try:
        connection.send(value)
        return True
    except (BrokenPipeError, EOFError, OSError):
        return False


def _get_latest(connection):
    latest = None
    while connection.poll():
        try:
            latest = connection.recv()
        except (EOFError, OSError):
            break
    return latest


def _keyboard_state(agent):
    return {
        "obstacle_debug_frame": agent.obstacle_debug_frame.copy(),
        "obstacle_debug_geom": list(agent.obstacle_debug_geom),
        "robot_goal_left_offset": agent.left_goal_debug_frame.copy(),
        "robot_goal_right_offset": agent.right_goal_debug_frame.copy(),
        "robot_goal_base_offset": agent.base_goal_debug_frame.copy(),
        "left_gripper_goal": bool(agent.left_gripper_goal),
        "right_gripper_goal": bool(agent.right_gripper_goal),
        "left_gripper_debug_state": bool(agent.left_gripper_debug_state),
        "right_gripper_debug_state": bool(agent.right_gripper_debug_state),
        "show_contact_forces": bool(agent.show_contact_forces),
        "show_contact_wrench": bool(agent.show_contact_wrench),
        "render_robot_collision_volumes": bool(agent.render_robot_collision_volumes),
    }


def _apply_latest_robot_state(
    agent,
    dof_pos,
    robot_base_frame,
    *,
    fixed_base,
    left_gripper_goal=False,
    right_gripper_goal=False,
):
    """Reassert the authoritative model state before every presentation frame.

    Isaac/Kit may perform more than one application update between two control
    packets.  A direct articulation write therefore cannot be treated as a
    one-shot notification: a later Kit update can expose the previous Fabric
    pose even though SPARK's retained debug geometry already represents the
    newest state.  Applying the latest complete robot state on every viewer
    frame keeps the mesh and overlays on the same snapshot.
    """
    dof_pos = np.asarray(dof_pos, dtype=float)
    joint_pos = dof_pos[agent._joint_dof_indices]
    agent.articulation.set_joint_positions(
        agent._torch.as_tensor(joint_pos, device=agent.device, dtype=agent._torch.float32),
        joint_indices=agent.joint_indices,
    )

    if robot_base_frame is not None and not fixed_base:
        from scipy.spatial.transform import Rotation

        robot_base_frame = np.asarray(robot_base_frame, dtype=float)
        quaternion_xyzw = Rotation.from_matrix(robot_base_frame[:3, :3]).as_quat()
        agent.articulation.set_world_pose(
            position=robot_base_frame[:3, 3],
            orientation=np.roll(quaternion_xyzw, 1),
        )
    agent._update_gripper_targets(
        {
            "left_gripper_goal": bool(left_gripper_goal),
            "right_gripper_goal": bool(right_gripper_goal),
        }
    )
    # The renderer is a visualization articulation, but its hand should still
    # move continuously. Limit each 50 Hz presentation update instead of
    # teleporting between open and closed qpos configurations.
    agent._write_gripper_configuration(max_step=0.06)


def _show_viewport_only():
    """Hide Isaac editor chrome without hiding the viewport compositor."""
    import omni.ui

    viewport = None
    layout_changed = False
    for window in omni.ui.Workspace.get_windows():
        title = str(getattr(window, "title", ""))
        # Isaac creates a late render-product inspection window named like
        # ``Viewport/Viewport0 Sensors Output``.  It is not the interactive
        # viewport and, when left visible, covers nearly the entire rendered
        # scene with an opaque dark panel.
        is_sensor_output = "Sensors Output" in title
        if title.startswith("Viewport") and not is_sensor_output:
            viewport = window
            if not window.visible:
                window.visible = True
                layout_changed = True
            if getattr(window, "dock_tab_bar_enabled", False):
                window.dock_tab_bar_enabled = False
                layout_changed = True
            if getattr(window, "dock_tab_bar_visible", False):
                window.dock_tab_bar_visible = False
                layout_changed = True
            find_layer = getattr(window, "_find_viewport_layer", None)
            if find_layer is not None:
                menubar = find_layer("Menubar", "menubar")
                if menubar is not None and getattr(menubar, "visible", False):
                    menubar.visible = False
                    layout_changed = True
        elif title == ISAAC_SIMULATION_INFO_WINDOW_TITLE:
            if not window.visible:
                window.visible = True
                layout_changed = True
        elif title != "DockSpace":
            if window.visible:
                window.visible = False
                layout_changed = True
    if viewport is not None and layout_changed:
        viewport.focus()


def _run_renderer(frame_receiver, input_sender, status_sender, stop_receiver, options):
    simulation_app = None
    agent = None
    try:
        from isaacsim import SimulationApp

        launch_config = {
            "headless": False,
            # In Isaac Sim 6 ``hide_ui`` also hides the viewport widget and
            # leaves a live but entirely black native window. Keep the UI
            # compositor active; auxiliary panels are hidden after startup.
            "hide_ui": False,
            "width": options["width"],
            "height": options["height"],
            "window_width": options["width"],
            "window_height": options["height"],
            "renderer": options["renderer"],
            "anti_aliasing": options["anti_aliasing"],
            "samples_per_pixel_per_frame": options["samples_per_pixel_per_frame"],
            "denoiser": options["denoiser"],
            "max_bounces": options["max_bounces"],
            "limit_cpu_threads": options["limit_cpu_threads"],
            "extra_args": [
                "--/exts/omni.kit.hotkeys.core/hotkeys_enabled=false",
                "--/app/viewport/defaults/noTitleBar=true",
                "--/persistent/app/viewport/noPadding=true",
                "--/exts/omni.kit.viewport.window/startup/dockTabInvisible=true",
            ],
        }
        simulation_app = SimulationApp(launch_config)

        import spark_robot
        from spark_agent.simulation.isaac.unitree_g1.unitree_g1_isaac_agent import (
            UnitreeG1IsaacAgent,
        )

        robot_cfg = getattr(spark_robot, options["robot_cfg_class_name"])()
        agent = UnitreeG1IsaacAgent(
            robot_cfg,
            fixed_base=options["fixed_base"],
            dynamics_backend="model",
            use_sim_dynamics=False,
            render=True,
            enable_viewer=True,
            viewer_show_simulation_info=options["viewer_show_simulation_info"],
            render_on_step=False,
            enable_keyboard_control=options["enable_keyboard_control"],
            enable_hand_control=options["enable_hand_control"],
            enable_camera=options["enable_camera"],
            camera_width=options["camera_width"],
            camera_height=options["camera_height"],
            camera_rate_hz=options["camera_rate_hz"],
            camera_display=options["camera_display"],
            camera_config=options["camera_config"],
            device=options["device"],
            obstacle_debug={
                "num_obstacle": options["num_obstacle_debug"],
                "manual_movement_step_size": options["manual_movement_step_size"],
            },
            viewer_config=options["viewer_config"],
        )
        agent.reset()
        _show_viewport_only()
        simulation_app.update()
        _send(input_sender, _keyboard_state(agent))
        _send(status_sender, {"ready": True})

        viewer_hz = float(options["viewer_hz"])
        render_period = 0.0 if viewer_hz <= 0.0 else 1.0 / viewer_hz
        next_render = time.perf_counter()
        next_input_update = next_render
        frames = 0
        consumed_frames = 0
        latest_debug_primitives = []
        latest_dof_pos = None
        latest_robot_base_frame = None
        latest_left_gripper_goal = False
        latest_right_gripper_goal = False
        # Workspace mutation is completed before the input-ready signal above.
        # Re-enumerating and hiding windows during live keyboard dispatch can
        # invalidate Kit's native window callback iteration.
        layout_updates_remaining = 0
        start = next_render
        while not stop_receiver.poll() and simulation_app.is_running():
            # Space is a SPARK obstacle-selection key, but some Kit layouts
            # also bind it to play/pause. Kinematic rendering must remain live.
            if not agent.sim.is_playing():
                agent.sim.play()
            frame = _get_latest(frame_receiver)
            if frame is not None:
                latest_dof_pos = np.asarray(frame["dof_pos"], dtype=float).copy()
                root_pose = frame.get("robot_base_frame")
                latest_robot_base_frame = (
                    None if root_pose is None else np.asarray(root_pose, dtype=float).copy()
                )
                latest_debug_primitives = list(frame.get("debug_primitives", ()))
                latest_left_gripper_goal = bool(frame.get("left_gripper_goal", False))
                latest_right_gripper_goal = bool(frame.get("right_gripper_goal", False))
                simulation_info = frame.get("simulation_info")
                if simulation_info is not None:
                    agent.set_viewer_simulation_info(simulation_info, replace=True)
                colors = frame.get("obstacle_debug_colors")
                if colors is not None:
                    for geom, color in zip(agent.obstacle_debug_geom, colors):
                        geom.color = color
                consumed_frames += 1

            if latest_dof_pos is not None:
                agent.dof_pos_cmd = latest_dof_pos.copy()
                agent.dof_pos_fbk = latest_dof_pos.copy()
                _apply_latest_robot_state(
                    agent,
                    latest_dof_pos,
                    latest_robot_base_frame,
                    fixed_base=options["fixed_base"],
                    left_gripper_goal=latest_left_gripper_goal,
                    right_gripper_goal=latest_right_gripper_goal,
                )

            # Re-render the latest complete debug scene until it is replaced.
            # Agent.render() consumes this list, so a fresh copy is required on
            # every presentation frame to avoid one-frame visibility gaps.
            agent._debug_primitives = list(latest_debug_primitives)
            agent._synchronize_kinematic_articulation()
            agent.render()
            if layout_updates_remaining > 0:
                _show_viewport_only()
                layout_updates_remaining -= 1
            now = time.perf_counter()
            if now >= next_input_update:
                _send(input_sender, _keyboard_state(agent))
                # Match the 50 Hz control contract.  A 20 Hz input channel
                # made short bracket/F/C presses easy to overwrite or miss
                # between authoritative state packets.
                next_input_update = now + 0.02
            frames += 1

            if render_period > 0.0:
                next_render += render_period
                remaining = next_render - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)
                elif remaining < -render_period:
                    next_render = time.perf_counter()

        elapsed = time.perf_counter() - start
        _send(
            status_sender,
            {
                "stopped": True,
                "viewer_hz": frames / elapsed if elapsed > 0.0 else 0.0,
                "frames": frames,
                "consumed_frames": consumed_frames,
            },
        )
    except BaseException:
        _send(status_sender, {"error": traceback.format_exc()})
    finally:
        if agent is not None:
            agent.close()
        if simulation_app is not None:
            simulation_app.close(wait_for_replicator=False)
        for connection in (
            frame_receiver,
            input_sender,
            status_sender,
            stop_receiver,
        ):
            connection.close()


class IsaacRendererProcess:
    """Own Isaac rendering in a child process and exchange latest-only frames."""

    def __init__(
        self,
        *,
        robot_cfg_class_name,
        fixed_base=True,
        enable_keyboard_control=True,
        enable_hand_control=False,
        viewer_show_simulation_info=False,
        num_obstacle_debug=1,
        manual_movement_step_size=0.02,
        viewer_hz=70.0,
        render_quality="performance",
        renderer=None,
        anti_aliasing=None,
        width=None,
        height=None,
        limit_cpu_threads=8,
        device="cpu",
        viewer_config=None,
        enable_camera=False,
        camera_width=1280,
        camera_height=720,
        camera_rate_hz=30.0,
        camera_display=False,
        camera_config=None,
        startup_timeout=90.0,
    ):
        from spark_agent.simulation.isaac.render_quality import isaac_render_preset

        render_config = isaac_render_preset(
            render_quality,
            renderer=renderer,
            anti_aliasing=anti_aliasing,
            width=width,
            height=height,
        )
        context = mp.get_context("spawn")
        frame_receiver, self._frame_sender = context.Pipe(duplex=False)
        self._input_receiver, input_sender = context.Pipe(duplex=False)
        self._status_receiver, status_sender = context.Pipe(duplex=False)
        stop_receiver, self._stop_sender = context.Pipe(duplex=False)
        self._closed = False
        self._process = context.Process(
            target=_run_renderer,
            args=(
                frame_receiver,
                input_sender,
                status_sender,
                stop_receiver,
                {
                    "robot_cfg_class_name": robot_cfg_class_name,
                    "fixed_base": bool(fixed_base),
                    "enable_keyboard_control": bool(enable_keyboard_control),
                    "enable_hand_control": bool(enable_hand_control),
                    "viewer_show_simulation_info": bool(viewer_show_simulation_info),
                    "num_obstacle_debug": int(num_obstacle_debug),
                    "manual_movement_step_size": float(manual_movement_step_size),
                    "viewer_hz": float(viewer_hz),
                    "render_quality": render_config["quality"],
                    "renderer": render_config["renderer"],
                    "anti_aliasing": int(render_config["anti_aliasing"]),
                    "width": int(render_config["width"]),
                    "height": int(render_config["height"]),
                    "samples_per_pixel_per_frame": int(
                        render_config["samples_per_pixel_per_frame"]
                    ),
                    "denoiser": bool(render_config["denoiser"]),
                    "max_bounces": int(render_config["max_bounces"]),
                    "limit_cpu_threads": int(limit_cpu_threads),
                    "device": device,
                    "viewer_config": dict(viewer_config or {}),
                    "enable_camera": bool(enable_camera),
                    "camera_width": int(camera_width),
                    "camera_height": int(camera_height),
                    "camera_rate_hz": float(camera_rate_hz),
                    "camera_display": bool(camera_display),
                    "camera_config": dict(camera_config or {}),
                },
            ),
            name="spark-isaac-renderer",
        )
        self._process.start()
        for child_endpoint in (
            frame_receiver,
            input_sender,
            status_sender,
            stop_receiver,
        ):
            child_endpoint.close()
        deadline = time.monotonic() + float(startup_timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.close(force=True)
                raise TimeoutError(
                    "Isaac renderer did not initialize in time. Check that no "
                    "other Isaac viewer is still using the GPU."
                )
            if self._status_receiver.poll(min(0.25, remaining)):
                status = self._status_receiver.recv()
                break
            if self._process.is_alive():
                continue
            exitcode = self._process.exitcode
            self.close(force=True)
            raise RuntimeError(
                "Isaac renderer exited during native initialization "
                f"(exit code {exitcode}); no Python traceback was produced"
            )
        if "error" in status:
            self.close(force=True)
            raise RuntimeError(status["error"])
        if not status.get("ready"):
            self.close(force=True)
            raise RuntimeError(f"Unexpected Isaac renderer startup status: {status}")

    @property
    def is_alive(self):
        return not self._closed and self._process.is_alive()

    def submit(self, frame):
        if self.is_alive:
            _send(self._frame_sender, frame)

    def poll_input(self):
        return _get_latest(self._input_receiver)

    def close(self, *, force=False):
        if self._closed:
            return None
        self._closed = True
        _send(self._stop_sender, True)
        # Kit can spend more than 15 seconds releasing RTX/viewport resources
        # even after the renderer loop has exited.  Terminating during that
        # native shutdown leaks multiprocessing semaphores and makes the next
        # viewer launch unreliable, so allow one complete graceful teardown.
        self._process.join(timeout=45.0)
        if self._process.is_alive() and force:
            self._process.terminate()
            self._process.join(timeout=5.0)
        status = _get_latest(self._status_receiver)
        if not self._process.is_alive():
            for connection in (
                self._frame_sender,
                self._input_receiver,
                self._status_receiver,
                self._stop_sender,
            ):
                connection.close()
            self._process.close()
        return status
