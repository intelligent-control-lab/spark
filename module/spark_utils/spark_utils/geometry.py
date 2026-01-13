import numpy as np
from numba import njit, prange
from typing import List

class VizColor:
    '''
    Class for defining colors for visualization.
    format: R, G, B, A
    '''
    
    # objects
    goal = [48/255, 245/255, 93/255, 0.3]
    collision_volume = [0.1, 0.1, 0.1, 0.7]
    collision_volume_ignored = [0.1, 0.1, 0.1, 0.0]
    obstacle_debug = [0.5, 0.5, 0.5, 0.7]
    obstacle_task = [0.5, 0.5, 0.5, 0.7]
    
    # safety index
    safe = [0, 1, 0, 0.5]
    hold = [245/255, 243/255, 48/255, 0.5] # yellow
    unsafe = [0/255, 32/255, 230/255, 0.5] # blue
    safe_zone = [255/255, 165/255, 0/255, 0.5] # orange
    
    # constraint
    violation = [160/255, 32/255, 240/255, 0.5] # purple
    
    # collision
    collision = [255/255, 0/255, 0/255, 0.8] # red

    # nan
    not_a_number = [255/255, 0/255, 0/255, 0.4] # red

    # coordinate frame axes
    x_axis = [1, 0, 0, 1]
    y_axis = [0, 1, 0, 1]
    z_axis = [0, 0, 1, 1]


class Geometry:
    def __init__(self, type, **kwargs):
        self.type = type
        self.attributes = {}
        self.color = kwargs.get("color", np.array([1, 1, 1, 0.5]))

        # Define required attributes for each geometry type
        required_attr = []
        if self.type == 'sphere':
            required_attr = ['radius']
            self.type_code = 0  # Used for parallel computation to identify type
            self.size = kwargs.get('radius', 1.0) * np.ones(3)  # Default radius if not provided
        elif self.type == 'box':
            required_attr = ['length', 'width', 'height']
            self.type_code = 1  # Used for parallel computation to identify type
            self.size = np.array([
                kwargs.get('length', 1.0), 
                kwargs.get('width', 1.0), 
                kwargs.get('height', 1.0)
            ], dtype=np.float32)
        else:
            raise ValueError(f'Unknown geometry type: {self.type}')

        # Assign required attributes
        for attr in required_attr:
            if attr in kwargs:
                self.attributes[attr] = kwargs[attr]
            else:
                raise ValueError(f'Missing required attribute: {attr} for geometry type: {self.type}')
    
    def get_attributes(self):
        """Return geometry attributes as a dict."""
        return self.attributes
    
class Volume():
    def __init__(self, frame, geometry, velocity=np.zeros(6)):
        self.frame = frame
        self.velocity = velocity
        self.geometry = geometry

@njit
def sphere_to_sphere_distance(frame1, size1, frame2, size2):
    """
    Compute distance from a sphere to a sphere.
    """
    s1_x, s1_y, s1_z = frame1[0][3], frame1[1][3], frame1[2][3]
    s2_x, s2_y, s2_z = frame2[0][3], frame2[1][3], frame2[2][3]
    
    # Compute Euclidean distance
    dx, dy, dz = s2_x - s1_x, s2_y - s1_y, s2_z - s1_z
    d = (dx**2 + dy**2 + dz**2) ** 0.5  # Euclidean distance
    
    d -= (size1[0] + size2[0])
    
    return d

@njit
def sphere_to_box_distance(frame1, size1, frame2, size2):
    """
    Compute distance from a sphere to a box.
    """
    s_x, s_y, s_z = frame1[0][3], frame1[1][3], frame1[2][3]
    b_x, b_y, b_z = frame2[0][3], frame2[1][3], frame2[2][3]
    
    radius = size1[0]
    box_half_size_x, box_half_size_y, box_half_size_z = size2[0] / 2, size2[1] / 2, size2[2] / 2
    
    # Compute local point coordinates (pos1 - center2)
    local_x = s_x - b_x
    local_y = s_y - b_y
    local_z = s_z - b_z

    # Transform to box's local frame (using rotation matrix)
    r00, r01, r02 = frame2[0][0], frame2[0][1], frame2[0][2]
    r10, r11, r12 = frame2[1][0], frame2[1][1], frame2[1][2]
    r20, r21, r22 = frame2[2][0], frame2[2][1], frame2[2][2]

    local_point_x = r00 * local_x + r10 * local_y + r20 * local_z
    local_point_y = r01 * local_x + r11 * local_y + r21 * local_z
    local_point_z = r02 * local_x + r12 * local_y + r22 * local_z
    
    # Clamp to box extents
    clamped_x = min(max(local_point_x, -box_half_size_x), box_half_size_x)
    clamped_y = min(max(local_point_y, -box_half_size_y), box_half_size_y)
    clamped_z = min(max(local_point_z, -box_half_size_z), box_half_size_z)

    # Transform clamped point back to world frame
    closest_x = r00 * clamped_x + r01 * clamped_y + r02 * clamped_z + b_x
    closest_y = r10 * clamped_x + r11 * clamped_y + r12 * clamped_z + b_y
    closest_z = r20 * clamped_x + r21 * clamped_y + r22 * clamped_z + b_z
    
    dx, dy, dz = s_x - closest_x, s_y - closest_y, s_z - closest_z
    d = (dx**2 + dy**2 + dz**2) ** 0.5 # Euclidean distance
    d -= radius
        
    return d 

@njit
def compute_pairwise_distance_info(frame_list_1, velocity_list_1, type_list_1, size_list_1,
                                   frame_list_2, velocity_list_2, type_list_2, size_list_2):
    """
    Compute pairwise distance information between two sets of frames.
    
    Args:
        frame_list_1 (np.ndarray): [N1, 4, 4] Transforms for frame in list 1.
        velocity_list_1 (np.ndarray): [N1, 6] Velocities for frames in list 1.
        frame_list_2 (np.ndarray): [N2, 4, 4] Transforms for frame in list 2.
        velocity_list_2 (np.ndarray): [N1, 6] Velocities for frames in list 2.
    
    Returns:
        dist_raw (np.ndarray): Pairwise distances [N1, N2].
        vel (np.ndarray): Pairwise velocities [N1, N2, 3].
        norm (np.ndarray): Pairwise normals [N1, N2, 3].
        curv (np.ndarray): Pairwise curvatures [N1, N2].
    """
    N1 = frame_list_1.shape[0]
    N2 = frame_list_2.shape[0]
    dist = np.zeros((N1, N2, 3))
    vel = np.zeros((N1, N2, 3))
    norm = np.zeros((N1, N2, 3))
    curv = np.zeros((N1, N2, 3, 3))
    
    for i in range(N1):
        for j in range(N2):
            
            # Retrieve frames, types and sizes
            frame1, frame2 = frame_list_1[i], frame_list_2[j]
            velocity1, velocity2 = velocity_list_1[i], velocity_list_2[j]
            type1, type2 = type_list_1[i], type_list_2[j]
            size1, size2 = size_list_1[i], size_list_2[j]
            

            # Sphere-Sphere
            if type1 == 0 and type2 == 0:
                # print(frame1.shape, frame2.shape)
                d, v, n, c = sphere_to_sphere_distance_info(frame1, velocity1, size1, frame2, velocity2, size2)
            else: 
                #TODO: Sphere-Box
                raise NotImplementedError("Currently only Sphere-Sphere distance calculation is implemented. Make sure to use approximated safety index si1a or si2a")

            # Store result
            dist[i, j] = np.array(d)
            vel[i, j] = np.array(v)
            norm[i, j] = np.array(n)
            curv[i, j] = np.array(c)
            

    return dist, vel, norm, curv

@njit
def sphere_to_sphere_distance_info(frame1, velocity1, size1, frame2, velocity2, size2):
    """
    Compute distance, velocity, normal, and curvature for two spheres.
    
    Args:
        frame1, frame2: Transformation matrices for the two spheres.
        velocity1, velocity2: Velocities of the two spheres.
        size1, size2: Sizes (radii) of the two spheres.
    
    Returns:
        d: Distance between the two spheres.
        v: Relative velocity vector between the two spheres.
        n: Normalized normal vector from sphere 1 to sphere 2.
        c: Curvature matrix for the pair of spheres.
    """
    s1_x, s1_y, s1_z = frame1[0][3], frame1[1][3], frame1[2][3]
    s2_x, s2_y, s2_z = frame2[0][3], frame2[1][3], frame2[2][3]
    v1_x, v1_y, v1_z = velocity1[0], velocity1[1], velocity1[2]
    v2_x, v2_y, v2_z = velocity2[0], velocity2[1], velocity2[2]
    radius1 = size1[0]
    radius2 = size2[0]
    
    # Compute Euclidean distance
    dx, dy, dz = s2_x - s1_x, s2_y - s1_y, s2_z - s1_z
    dist = np.array([dx, dy, dz])
    d_center = (dx**2 + dy**2 + dz**2) ** 0.5  # Euclidean distance
    d_min = d_center - (radius1 + radius2)  # Minimum distance between surfaces of spheres
    
    vx, vy, vz = v1_x - v2_x, v1_y - v2_y, v1_z - v2_z
    # Relative velocity vector
    v = [vx, vy, vz]
    # Avoid division by zero
    if abs(d_center) > 1e-8:  # Avoid division by zero
        n = [dx / d_center, dy / d_center, dz / d_center]   # Normalized vector from sphere 1 to sphere 2
        # Compute outer product dist * dist^T (3x3 matrix)
        outer_product = [[dist[i] * dist[j] for j in range(3)] for i in range(3)]

        # Compute curvature matrix
        c = [[(1 / d_center) - (outer_product[i][j] / d_center**3) for j in range(3)] for i in range(3)]
    else:
        # If spheres are at the same position, set normal to a default direction
        # This can happen in singular cases where both spheres overlap exactly
        n = [0.0, 0.0, 0.0]
        c = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    
    d = [-n[0] * d_min, -n[1] * d_min, -n[2] * d_min]  # Distance vector from sphere 1 to sphere 2
        
    return d, v, n, c
    
    

@njit
def compute_phi_and_extract_best(d_min, n, k, v, normal, x_vals):
    """Compute Phi over the 3D grid and extract the 1000 points closest to Phi=0."""

    # Allocate Phi array
    grid_size = len(x_vals)
    Phi = np.zeros((grid_size, grid_size, grid_size))

    # Preallocate storage for valid points (store x, y, z, and |Phi|)
    max_store = grid_size**3  # Overestimate the number of points
    candidates = np.empty((max_store, 4))  # (x, y, z, |Phi|)
    count = 0  # Counter for stored points
    v = np.dot(v, normal) * normal
    # Compute Phi in parallel   
    for i in prange(grid_size):
        x = x_vals[i]
        for j in range(grid_size):
            y = x_vals[j]
            for k_idx in range(grid_size):
                z = x_vals[k_idx]
                
                norm_D = (x**2 + y**2 + z**2) ** 0.5  # Compute the norm of the distance vector
                if norm_D > 1e-8:  # Avoid division by zero
                    phi_value = d_min**n - norm_D**n + k * (v[0]*x + v[1]*y + v[2]*z) / norm_D
                else:
                    phi_value = d_min**n  # Handle singularity at origin

                Phi[i, j, k_idx] = phi_value
    return Phi

@njit
def compute_distance_kernel(frame_list_1, type_list_1, size_list_1, 
                             frame_list_2, type_list_2, size_list_2):
    # Ensure the input shapes are correct
    assert frame_list_1.shape[0] == frame_list_2.shape[0], "Batch size must be the same for both frame lists"
    
    B = frame_list_1.shape[0]
    N1 = frame_list_1.shape[1]
    N2 = frame_list_2.shape[1]
    
    dist = np.inf * np.ones((B, N1, N2))
    
    for b in range(B):
        for i in range(N1):
            for j in range(N2):
                # Retrieve frames, types and sizes
                frame1, frame2 = frame_list_1[b, i], frame_list_2[b, j]
                type1, type2 = type_list_1[i], type_list_2[j]
                size1, size2 = size_list_1[i], size_list_2[j]

                # Sphere-Sphere
                if type1 == 0 and type2 == 0:
                    # print(frame1.shape, frame2.shape)
                    d = sphere_to_sphere_distance(frame1, size1, frame2, size2)

                # Sphere-Box
                elif type1 == 0 and type2 == 1:
                    d = sphere_to_box_distance(frame1, size1, frame2, size2)
                elif type1 == 1 and type2 == 0:
                    d = sphere_to_box_distance(frame2, size2, frame1, size1)
                else:
                    # Currently, only Sphere-Sphere and Sphere-Box are implemented
                    # Raise an error for unsupported types
                    raise NotImplementedError("Currently only Sphere-Sphere and Sphere-Box distance calculations are implemented.")
                
                # Store result
                dist[b, i, j] = d

    return dist

def compute_pairwise_distance(frame_list_1, geom_list_1, frame_list_2, geom_list_2):
    """
    Wrapper for pairwise distance computation.
    
    Compute pairwise distances between spheres.
    
    Args:
        frame_list_1: [B, N1, 4, 4] transforms for set 1.
        geom_list_1: N1 geometry objects in set 1.
        frame_list_2: [B, N2, 4, 4] transforms for set 2.
        geom_list_2: N2 geometry objects in set 2.
    
    Returns:
        np.ndarray: Pairwise distances [B, N1, N2].
        
    """

    type_list_1 = np.array([geom.type_code for geom in geom_list_1])
    size_list_1 = np.array([geom.size for geom in geom_list_1])
    type_list_2 = np.array([geom.type_code for geom in geom_list_2])
    size_list_2 = np.array([geom.size for geom in geom_list_2])
    frame_list_1 = np.array(frame_list_1)
    frame_list_2 = np.array(frame_list_2)
    
    if len(frame_list_1.shape) == 3:
        frame_list_1 = frame_list_1[np.newaxis, :, :, :]  # Add batch dimension if missing
    if len(frame_list_2.shape) == 3:
        frame_list_2 = frame_list_2[np.newaxis, :, :, :]
    
    dist = compute_distance_kernel(frame_list_1, type_list_1, size_list_1, frame_list_2, type_list_2, size_list_2)
    
    return dist

def compute_masked_distance_matrix(frame_list_1, geom_list_1, frame_list_2, geom_list_2, mask=None):
    
    if len(frame_list_1) == 0 or len(frame_list_2) == 0:
        return None, None
    
    # compute pairwise distances
    dist_mat = compute_pairwise_distance(
        frame_list_1 = frame_list_1,
        geom_list_1  = geom_list_1,
        frame_list_2 = frame_list_2,
        geom_list_2  = geom_list_2
    )
    assert dist_mat.shape[0] == 1, "Batch size > 1 not supported"
    dist_mat = dist_mat[0]
    
    if mask is None:
        mask = np.ones_like(dist_mat, dtype=bool)
    
    dist_mat[~mask] = np.inf
    # Row and column indices of the minimum value in masked matrix
    dist_mat_masked = dist_mat[mask]
    
    if len(dist_mat_masked) == 0:
        return dist_mat, None
    
    indices_self_masked = np.argwhere(mask)  # Get the row and column indices of unmasked positions
    index_min_dist = indices_self_masked[np.argmin(dist_mat_masked).reshape(-1)]
    return dist_mat, index_min_dist