import numpy as np
import mujoco
from mujoco.glfw import glfw
from spark_agent.simulation.simulation_agent import SimulationAgent
from spark_robot import RobotConfig
from spark_utils import Geometry, VizColor


class MujocoAgent(SimulationAgent):
    """
    A class for controlling a Mujoco-based simulation agent.
    """

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        """
        Initializes the Mujoco agent with robot configuration and simulation parameters.

        Args:
            robot_cfg (RobotConfig): The configuration for the robot.
            **kwargs: Additional keyword arguments such as simulation dynamics, model path, viewer settings, etc.
        """
        super().__init__(robot_cfg)

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
        self.viewer.cam.distance = 2  # Set zoom level (distance from the object)
        self.viewer.cam.lookat[0] = 0  # X offset for the camera focus point
        self.viewer.cam.lookat[1] = 0  # Y offset for the camera focus point
        self.viewer.cam.lookat[2] = 0.8  # Z offset for the camera focus point

        # Camera rotations and angles
        self.viewer.cam.elevation = -10  # Camera rotation around the X axis (up/down)
        self.viewer.cam.azimuth = 180  # Camera rotation around the Y axis (left/right)

        # Set geometry group for visualization
        self.viewer.opt.geomgroup = 1  # Set which geometry group to visualize

    # ---------------------------------- User Interface Helpers--------------------------------- #

    def _add_obstacle(self):
        """
        Adds a new obstacle to the simulation and moves the pointer to the newly added obstacle.
        The obstacle is a sphere with a random position near a defined point.
        """
        self.num_obstacle_debug += 1  # Increment the obstacle count
        # Append a new obstacle's frame (4x4 identity matrix) to the list of obstacle frames
        self.obstacle_debug_frame = np.concatenate([self.obstacle_debug_frame, np.eye(4)[None, :, :]], axis=0)
        
        # Set random position for the new obstacle near the point [0.6, 0.0, 0.793]
        self.obstacle_debug_frame[-1, :3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
        self.obstacle_debug_frame_last = np.concatenate(
            [self.obstacle_debug_frame_last, self.obstacle_debug_frame[-1][None, :, :]], axis=0)
        # Add a new sphere obstacle with a radius of 0.05 and a color for visualization
        self.obstacle_debug_geom.append(Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug))
        
        # Set the newly added obstacle as the selected one
        self.obstacle_debug_selected = self.num_obstacle_debug - 1

    def _remove_obstacle(self):
        """
        Removes the currently selected obstacle from the simulation.
        """
        if self.num_obstacle_debug > 0:
            self.num_obstacle_debug -= 1  # Decrement the obstacle count
            
            # Remove the selected obstacle's frame and geometry
            self.obstacle_debug_frame = np.concatenate(
                [self.obstacle_debug_frame[:self.obstacle_debug_selected, :, :],
                self.obstacle_debug_frame[self.obstacle_debug_selected + 1:, :, :]],
                axis=0)
            self.obstacle_debug_frame_last = np.concatenate(
                [self.obstacle_debug_frame_last[:self.obstacle_debug_selected, :],
                self.obstacle_debug_frame_last[self.obstacle_debug_selected + 1:, :]],
                axis=0)
            self.obstacle_debug_geom = self.obstacle_debug_geom[:self.obstacle_debug_selected] + \
                                        self.obstacle_debug_geom[self.obstacle_debug_selected + 1:]
            
            # Update the selected obstacle
            self.obstacle_debug_selected = 0 if self.num_obstacle_debug > 0 else None

    def _key_callback(self, key):
        """
        Handles key events for obstacle movement and selection.
        """
        if self.debug_object is None:
            self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]  # Get the selected obstacle geometry
            
        # Handle movement keys for the selected obstacle
        if key == glfw.KEY_RIGHT:  # Move +Y
            self.debug_object[1, 3] += self.manual_step_size
        elif key == glfw.KEY_LEFT:  # Move -Y
            self.debug_object[1, 3] -= self.manual_step_size
        elif key == glfw.KEY_UP:  # Move -X
            self.debug_object[0, 3] -= self.manual_step_size
        elif key == glfw.KEY_DOWN:  # Move +X
            self.debug_object[0, 3] += self.manual_step_size
        elif key == glfw.KEY_E:  # Move +Z
            self.debug_object[2, 3] += self.manual_step_size
        elif key == glfw.KEY_Q:  # Move -Z
            self.debug_object[2, 3] -= self.manual_step_size
        elif key == glfw.KEY_2:  # Rotate +Yaw
            rotation = R.from_euler('xyz', [0, 0, 0.1], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
        elif key == glfw.KEY_3:  # Rotate -Yaw
            rotation = R.from_euler('xyz', [0, 0, -0.1], degrees=False)
            self.debug_object[:3, :3] @= rotation.as_matrix()
                
        # Switch to the next obstacle when SPACE is pressed
        elif key == glfw.KEY_SPACE:
            self.obstacle_debug_selected = (self.obstacle_debug_selected + 1) % self.num_obstacle_debug
            self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]
   
        # Adjust the number of obstacles based on PAGE_UP/PAGE_DOWN keys
        elif key == glfw.KEY_PAGE_UP:
            self.num_obstacle_debug_change_buf += 1
        elif key == glfw.KEY_PAGE_DOWN:
            self.num_obstacle_debug_change_buf -= 1
            
        elif key == glfw.KEY_O:
            self.debug_object = self.right_goal_debug_frame
        elif key == glfw.KEY_P:
            self.debug_object = self.left_goal_debug_frame
        
    # ---------------------------------- Render Helpers --------------------------------- #

    def render(self):
        """Renders the simulation environment, including obstacles and scene updates."""
        
        # Render the virtual obstacle for debugging
        self._render_obstacle_debug()

        # If renderer is available, update the scene with current model data
        if self.renderer:
            self.renderer.update_scene(self.data)
        
        # If viewer is available, synchronize the viewer with the current scene state
        if self.viewer:
            self.viewer.sync()

    def render_sphere(self, pos, mat, size, color):
        """Render a radial area (sphere) in the environment."""
        
        if self.renderer is None:
            return

        pos = np.asarray(pos)
        if pos.shape == (2,):
            pos = np.r_[pos, 0]  # Add Z-coordinate if only 2D position is given
        
        # Render the sphere in the main renderer scene
        mujoco.mjv_initGeom(
            self.renderer._scene.geoms[self.renderer._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=size,
            pos=pos.flatten(),
            mat=np.eye(3).flatten(),
            rgba=color,
        )
        self.renderer._scene.ngeom += 1
        
        # Render the sphere in the viewer scene if a viewer is available
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

    def render_box(self, pos, mat, size, color):
        """Render a rectangular area (box) in the environment."""
        
        if self.renderer is None:
            return

        pos = np.asarray(pos)
        if pos.shape == (2,):
            pos = np.r_[pos, 0]  # Add Z-coordinate if only 2D position is given
        
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

        pos1 = np.asarray(pos1)
        pos2 = np.asarray(pos2)
        
        # Compute midpoint and direction of the line segment
        midpoint = (pos1 + pos2) / 2
        length = np.linalg.norm(pos2 - pos1)
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
        mujoco.mjv_initGeom(
            self.renderer._scene.geoms[self.renderer._scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[radius, length / 2, 0.0],  # Capsule size: [radius, half-length]
            pos=midpoint.flatten(),
            mat=rot_matrix.flatten(),
            rgba=color,
        )
        self.renderer._scene.ngeom += 1

        # Render the capsule in the viewer scene if a viewer is available
        if self.viewer:
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[radius, length / 2, 0.0],  # Capsule size: [radius, half-length]
                pos=midpoint.flatten(),
                mat=rot_matrix.flatten(),
                rgba=color,
            )
            self.viewer.user_scn.ngeom += 1

    def render_surface(self, points, color=(0.8, 0.3, 0.3, 1)):
        """Render the convex hull of a set of 3D points as triangles."""
        
        # Center point of the convex hull (not currently used in rendering)
        center_point = np.mean(points.reshape(-1, 3), axis=0)

        # Loop through each simplex (triangle) in the convex hull
        for simplex in points:
            v1, v2, v3 = simplex

            # Compute edge directions and normal
            dir_x = (v2 - v1)
            dir_y = (v3 - v1)
            dir_z = np.cross(dir_x, dir_y) / np.linalg.norm(np.cross(dir_x, dir_y))
            
            # Compute the transformation matrix for rendering the triangle
            mat = np.vstack([dir_x, dir_y, dir_z]).T.flatten()

            # Render the first side of the triangle
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_TRIANGLE,
                size=[1, 1, 1],  # Triangle size
                pos=v1.flatten(),
                mat=mat.flatten(),
                rgba=color,
            )
            self.viewer.user_scn.ngeom += 1

            # Render the second side of the triangle (flipped normal)
            mat = np.vstack([dir_y, dir_x, -dir_z]).T.flatten()
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_TRIANGLE,
                size=[1, 1, 1],  # Triangle size
                pos=v1.flatten(),
                mat=mat.flatten(),
                rgba=color,
            )
            self.viewer.user_scn.ngeom += 1

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

    def _render_obstacle_debug(self):
        """Render obstacles for debugging purposes."""
        for i, (frame, geom) in enumerate(zip(self.obstacle_debug_frame, self.obstacle_debug_geom)):
            if geom.type == 'sphere':
                self.render_sphere(
                    pos=frame[:3, 3], 
                    mat=frame[:3, :3],  
                    size=geom.attributes["radius"] * np.ones(3), 
                    color=geom.color
                )
                if i == self.obstacle_debug_selected:
                    self.render_arrow(
                        frame[:3, 3] + [0, 0, 0.15],
                        frame[:3, 3] + [0, 0, 0.01],
                        VizColor.obstacle_debug
                    )
            elif geom.type == 'box':
                self.render_box(
                    pos=frame[:3, 3], 
                    mat=frame[:3, :3],  
                    size=np.array([geom.attributes["length"], 
                                geom.attributes["width"], 
                                geom.attributes["height"]]), 
                    color=geom.color
                )
                if i == self.obstacle_debug_selected:
                    self.render_arrow(
                        frame[:3, 3] + [0, 0, 0.15],
                        frame[:3, 3] + [0, 0, 0.01],
                        VizColor.obstacle_debug
                    )
            else:
                raise ValueError(f'Unknown geometry type: {geom.type}')
