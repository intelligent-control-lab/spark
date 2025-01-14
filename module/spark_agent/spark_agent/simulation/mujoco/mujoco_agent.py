import numpy as np
from spark_agent.simulation.simulation_agent import SimulationAgent
from spark_robot import RobotConfig
from spark_utils import Geometry, VizColor
import mujoco
import mujoco.viewer
from mujoco.glfw import glfw
import time
import os
from spark_robot import SPARK_ROBOT_RESOURCE_DIR

class MujocoAgent(SimulationAgent):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)

        self.model = mujoco.MjModel.from_xml_path(os.path.join(SPARK_ROBOT_RESOURCE_DIR, kwargs["mujoco_model"]))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data, key_callback=self.key_callback)
        self.viewer_setup()
        self.renderer =  mujoco.Renderer(self.model)
        self.dt = kwargs["dt"]
        self.model.opt.timestep = self.dt

        # obstacle from simulation (e.g., keyboard controlled)
        self.obstacle_debug = kwargs.get("obstacle_debug", dict(num_obstacle=0, manual_movement_step_size=0.1))
        self.obstacle_debug_geom = []
        self.num_obstacle_debug = self.obstacle_debug["num_obstacle"]
        self.manual_step_size = self.obstacle_debug["manual_movement_step_size"]
        self.obstacle_debug_frame = np.zeros((self.num_obstacle_debug, 4, 4))
        if self.num_obstacle_debug > 0:
            self.obstacle_debug_frame = np.stack([np.eye(4) for _ in range(self.num_obstacle_debug)], axis=0)
        for frame in self.obstacle_debug_frame:
            frame[:3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
            
        self.obstacle_debug_geom = [Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug) for _ in range(self.num_obstacle_debug)]
        self.obstacle_debug_selected = 0
        self.num_obstacle_debug_change_buf = 0
            
    def viewer_setup(self):
        # self.viewer.cam.trackbodyid = 0         # id of the body to track ()
        # self.viewer.cam.distance = self.model.stat.extent * 3       # how much you "zoom in", model.stat.extent is the max limits of the arena
        self.viewer.cam.distance = 2
        self.viewer.cam.lookat[0] = 0         # x,y,z offset from the object (works if trackbodyid=-1)
        self.viewer.cam.lookat[1] = 0
        self.viewer.cam.lookat[2] = 0.8
        self.viewer.cam.elevation = -10           # camera rotation around the axis in the plane going through the frame origin (if 0 you just see a line)
        self.viewer.cam.azimuth = 180             # camera rotation around the camera's vertical axis
        self.viewer.opt.geomgroup = 1   

    def add_obstacle(self):
        # add one obstacle and move pointer to that obstacle
        self.num_obstacle_debug += 1
        self.obstacle_debug_frame = np.concatenate([self.obstacle_debug_frame, np.eye(4)[None, :, :]], axis=0)
        self.obstacle_debug_frame[-1, :3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
        self.obstacle_debug_geom.append(Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug))
        self.obstacle_debug_selected = self.num_obstacle_debug - 1
    
    def remove_obstacle(self):
        # remove pointed obstacle
        if self.num_obstacle_debug > 0:
            self.num_obstacle_debug -= 1
            self.obstacle_debug_frame = np.concatenate(
                [self.obstacle_debug_frame[:self.obstacle_debug_selected, :, :],
                 self.obstacle_debug_frame[self.obstacle_debug_selected+1:, :, :]],
                axis=0)
            self.obstacle_debug_geom = self.obstacle_debug_geom[:self.obstacle_debug_selected] + \
                                        self.obstacle_debug_geom[self.obstacle_debug_selected+1:]
            self.obstacle_debug_selected = 0 if self.num_obstacle_debug > 0 else None

    # ---------------------------------- helpers --------------------------------- #
    def key_callback(self, key):
        
        # perform obstacle movement / swtich when there are >0 obstacles
        if self.num_obstacle_debug > 0:
            selected = self.obstacle_debug_selected
            step = self.manual_step_size
            if key == glfw.KEY_RIGHT:      # Move +Y
                if selected is not None:
                    self.obstacle_debug_frame[selected, 1, 3] += step
            elif key == glfw.KEY_LEFT:     # Move -Y
                if selected is not None:
                    self.obstacle_debug_frame[selected, 1, 3] -= step
            elif key == glfw.KEY_UP:       # Move -X
                if selected is not None:
                    self.obstacle_debug_frame[selected, 0, 3] -= step
            elif key == glfw.KEY_DOWN:     # Move +X
                if selected is not None:
                    self.obstacle_debug_frame[selected, 0, 3] += step
            elif key == glfw.KEY_E:         # Move +Z
                if selected is not None:
                    self.obstacle_debug_frame[selected, 2, 3] += step
            elif key == glfw.KEY_Q:         # Move -Z
                if selected is not None:
                    self.obstacle_debug_frame[selected, 2, 3] -= step
            elif key == glfw.KEY_SPACE:
                self.obstacle_debug_selected = (self.obstacle_debug_selected + 1) % self.num_obstacle_debug
            
        if key == glfw.KEY_PAGE_UP:
            self.num_obstacle_debug_change_buf += 1
        elif key == glfw.KEY_PAGE_DOWN:
            self.num_obstacle_debug_change_buf -= 1
    
    # todo generalize to arbitrary Geometry types
    def render_sphere(self, pos, size, color):
        ''' Render a radial area in the environment '''
        pos = np.asarray(pos)
        if pos.shape == (2,):
            pos = np.r_[pos, 0]  # Z coordinate 0
        mujoco.mjv_initGeom(
            self.renderer._scene.geoms[self.renderer._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=size,
            pos=pos.flatten(),
            mat=np.eye(3).flatten(),
            rgba=color,
            )
        self.renderer._scene.ngeom += 1
        if self.viewer:
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=size * np.ones(3),
                pos=pos.flatten(),
                mat=np.eye(3).flatten(),
                rgba=color,
            )
            self.viewer.user_scn.ngeom += 1
    
    def render_line_segment(self, pos1, pos2, radius, color):
        """ Render a line segment in the environment """
        pos1 = np.asarray(pos1)
        pos2 = np.asarray(pos2)
        midpoint = (pos1 + pos2) / 2
        length = np.linalg.norm(pos2 - pos1)

        direction = (pos2 - pos1) / length

        # Create a rotation matrix to align the capsule/cylinder with the line segment
        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, direction)
        axis_len = np.linalg.norm(axis)

        if axis_len > 1e-6:
            axis = axis / axis_len
            angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
            quat = np.zeros(4)
            mujoco.mju_axisAngle2Quat(quat, axis, angle)  # Compute quaternion
            rot_matrix = np.zeros((3, 3)).flatten()
            mujoco.mju_quat2Mat(rot_matrix, quat) 
        else:
            rot_matrix = np.eye(3)

        # Render in the main renderer
        mujoco.mjv_initGeom(
            self.renderer._scene.geoms[self.renderer._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[radius, length / 2, 0.0],  # Capsule radius and half-length
            pos=midpoint.flatten(),
            mat=rot_matrix.flatten(),
            rgba=color,
        )
        self.renderer._scene.ngeom += 1

        # Render in the viewer, if available
        if self.viewer:
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[radius, length / 2, 0.0],
                pos=midpoint.flatten(),
                mat=rot_matrix.flatten(),
                rgba=color,
            )
            self.viewer.user_scn.ngeom += 1


    
    def render_arrow(self, start, end, color):
        start = np.asarray(start)
        end = np.asarray(end)
        arrow_dir = end - start
        arrow_length = np.linalg.norm(arrow_dir)

        # If the arrow length is zero, do not render
        if arrow_length < 1e-10:
            return

        # Normalize the direction
        z = arrow_dir / arrow_length

        # Construct an orientation matrix for the arrow:
        # The arrow will point along the z-axis of this frame.
        # We need two other perpendicular axes (x and y) to form a proper rotation matrix.
        # Pick a temporary axis that is not parallel to z to form orthonormal basis.
        if abs(z[0]) < 0.9:
            tmp = np.array([1.0, 0.0, 0.0])
        else:
            tmp = np.array([0.0, 1.0, 0.0])

        y = np.cross(z, tmp)
        y /= np.linalg.norm(y)
        x = np.cross(y, z)
        mat = np.column_stack((x, y, z)).flatten()

        # Define arrow sizes:
        # For mjGEOM_ARROW, size = [shaft_width, head_width, arrow_length].
        # Adjust these as necessary to achieve a desirable look.
        shaft_radius = 0.05 * arrow_length
        head_radius = 0.1 * arrow_length
        size = np.array([shaft_radius, head_radius, arrow_length])

        pos = start  # Arrow base (pos) is at the start

        # Add arrow to the renderer scene
        mujoco.mjv_initGeom(
            self.renderer._scene.geoms[self.renderer._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_ARROW,
            size=size,
            pos=pos,
            mat=mat,
            rgba=np.array(color),
        )
        self.renderer._scene.ngeom += 1

        # Add arrow to the viewer scene if a viewer is available
        if self.viewer:
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_ARROW,
                size=size,
                pos=pos,
                mat=mat,
                rgba=np.array(color)
            )
            self.viewer.user_scn.ngeom += 1

    def render_obstacle_debug(self):
        for i, (frame, geom) in enumerate(zip(self.obstacle_debug_frame, self.obstacle_debug_geom)):
            if geom.type == 'sphere':
                self.render_sphere(frame[:3, 3], geom.attributes["radius"]*np.ones(3), geom.color)
                if i == self.obstacle_debug_selected:
                    self.render_arrow(frame[:3, 3] + [0, 0, 0.15],
                                      frame[:3, 3] + [0, 0, 0.01],
                                      VizColor.obstacle_debug)
            else:
                raise ValueError(f'Unknown geometry type: {geom.type}')

    # -------------------------------- simulation -------------------------------- #

    def set_dof_pos(self, dof_pos: np.ndarray) -> None:
        qpos = np.zeros(self.model.nq)
        
        for mj_dof in self.robot_cfg.MujocoDoFs:
            dof = self.robot_cfg.MujocoDoF_to_DoF[mj_dof]
            qpos[mj_dof] = dof_pos[dof]
        self.data.qpos = qpos
        self.model.opt.gravity[:] = [0, 0, 0]  # Disable gravity
        self.data.qvel[:] = 0                  # Clear velocities
        self.data.qacc[:] = 0                  # Clear accelerations
        self.data.qfrc_applied[:] = 0          # Clear applied forces
        self.data.xfrc_applied[:, :] = 0       # Clear external forces

    def reset(self) -> None:
        # todo
        pass
    
    def mujoco_step(self):
        mujoco.mj_step(self.model, self.data)
        if self.viewer:
            self.viewer.user_scn.ngeom = 0
        self.renderer._scene.ngeom = 0
        time.sleep(self.model.opt.timestep)
        
    def render(self):

        # render virtual obstacle
        self.render_obstacle_debug()

        # update mujoco scene
        self.renderer.update_scene(self.data)
        self.viewer.sync()

    # ---------------------------------------------------------------------------- #
    #                                 base routines                                #
    # ---------------------------------------------------------------------------- #

    def _send_control_modeled_dynamics(self, command, **kwargs):

        # invoke modeled dynamics
        x = self.compose_state()
        x_dot = self.robot_cfg.dynamics_xdot(x, command)
        
        # integration
        x += x_dot * self.dt
        
        # get command
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof(x_dot)
        
        # set state by overriding dof_pos
        self.set_dof_pos(self.dof_pos_cmd)
    
    def post_control_processing(self, **kwargs):
        
        num_obstacle_debug_change = self.num_obstacle_debug_change_buf
        self.num_obstacle_debug_change_buf -= num_obstacle_debug_change # prevent miss counting keyboard input
        
        while num_obstacle_debug_change > 0:
            self.add_obstacle()
            num_obstacle_debug_change -= 1
            
        while num_obstacle_debug_change < 0:
            self.remove_obstacle()
            num_obstacle_debug_change += 1
            
        self.mujoco_step()

    def get_feedback(self) -> None:

        ret = {}

        # feedback robot frame in world frame
        # Get body ID

        # Extract global position and orientation
        global_position = self.data.body("robot").xpos.copy()  # [x, y, z]
        global_orientation = self.data.body("robot").xmat.copy().reshape(3, 3)  # 3x3 rotation matrix

        # Construct the 4x4 transformation matrix
        robot_base_frame = np.eye(4)  # Start with identity matrix
        robot_base_frame[:3, :3] = global_orientation  # Top-left 3x3 is the rotation matrix
        robot_base_frame[:3, 3] = global_position 
        
        ret["robot_base_frame"] = robot_base_frame

        # feedback dof pos
        dof_pos_fbk = np.zeros(self.num_dof)
        for dof in self.robot_cfg.DoFs:
            mj_dof = self.robot_cfg.DoF_to_MujocoDoF[dof]
            dof_pos_fbk[dof] = self.data.qpos[mj_dof]

        ret["dof_pos_fbk"] = dof_pos_fbk

        if self.dof_pos_cmd is None:
            self.dof_pos_cmd = dof_pos_fbk
        if self.dof_vel_cmd is None:
            self.dof_vel_cmd = np.zeros(self.num_dof)
        if self.dof_acc_cmd is None:
            self.dof_acc_cmd = np.zeros(self.num_dof)

        # commanded dof
        ret["dof_pos_cmd"] = self.dof_pos_cmd
        ret["dof_vel_cmd"] = self.dof_vel_cmd
        ret["dof_acc_cmd"] = self.dof_acc_cmd

        # dynamics state
        ret["state"] = self.compose_state()

        # virtual obstacle
        ret["obstacle_debug_frame"] = self.obstacle_debug_frame
        ret["obstacle_debug_geom"] = self.obstacle_debug_geom

        return ret