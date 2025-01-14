import numpy as np
from numba import njit
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
    hold = [245/255, 243/255, 48/255, 0.5]
    unsafe = [255/255, 84/255, 48/255, 1.0]
    
    # constraint
    slack_positive = [255/255, 84/255, 48/255, 0.5]
    # slack_positive = [51/255, 255/255, 233/255, 0.5]

class Geometry:
    def __init__(self, type, **kwargs):
        self.type = type
        self.attributes = {}
        self.color = kwargs.get("color", np.array([1, 1, 1, 0.5]))

        # Define required attributes for each geometry type
        required_attr = []
        if self.type == 'sphere':
            required_attr = ['radius']
        elif self.type == 'box':
            # Add required attributes for boxes
            required_attr = ['length', 'width', 'height']
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


@njit
def compute_distances_spheres(frame_list_1, radii_1, frame_list_2, radii_2):
    """
    Compute pairwise distances between spheres.
    
    Args:
        frame_list_1 (np.ndarray): [N1, 4, 4] transforms for set 1.
        radii_1 (np.ndarray): Radii of spheres in set 1.
        frame_list_2 (np.ndarray): [N2, 4, 4] transforms for set 2.
        radii_2 (np.ndarray): Radii of spheres in set 2.
    
    Returns:
        np.ndarray: Pairwise distances [N1, N2].
    """
    N1 = frame_list_1.shape[0]
    N2 = frame_list_2.shape[0]
    dist = np.zeros((N1, N2))

    for i in range(N1):
        for j in range(N2):
            pos1 = frame_list_1[i, :3, 3]
            pos2 = frame_list_2[j, :3, 3]
            dist[i, j] = np.linalg.norm(pos1 - pos2) - radii_1[i] - radii_2[j]

    return dist


def compute_pairwise_dist(frame_list_1: List[np.ndarray], geom_list_1: List[Geometry],
                          frame_list_2: List[np.ndarray], geom_list_2: List[Geometry]):
    """
    Wrapper for pairwise distance computation.
    
    Compute pairwise distances between spheres.
    
    Args:
        frame_list_1: N1 [4, 4] transforms for set 1.
        geom_list_1: N1 geometry objects in set 1.
        frame_list_2: N2 [4, 4] transforms for set 2.
        geom_list_2: N2 geometry objects in set 2.
    
    Returns:
        np.ndarray: Pairwise distances [N1, N2].
        
    """
    # Extract sphere attributes
    radii_1 = np.array([geom.attributes['radius'] for geom in geom_list_1 if geom.type == 'sphere'])
    radii_2 = np.array([geom.attributes['radius'] for geom in geom_list_2 if geom.type == 'sphere'])

    # Filter frames for spheres
    frames_1 = np.array([frame for geom, frame in zip(geom_list_1, frame_list_1) if geom.type == 'sphere'])
    frames_2 = np.array([frame for geom, frame in zip(geom_list_2, frame_list_2) if geom.type == 'sphere'])

    # Compute distances for spheres
    if len(frames_1) > 0 and len(frames_2) > 0:
        dist = compute_distances_spheres(frames_1, radii_1, frames_2, radii_2)
    else:
        dist = np.zeros((len(frame_list_1), len(frame_list_2)))

    return dist
