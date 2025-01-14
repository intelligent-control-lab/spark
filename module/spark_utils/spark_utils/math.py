import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_to_rotation_matrix(q):
    """
    Converts a quaternion into a 3x3 rotation matrix.

    Parameters:
    q (array-like): Quaternion [w, x, y, z]

    Returns:
    np.ndarray: 3x3 rotation matrix
    """
    w, x, y, z = q

    R = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
    ])
    return R

def transformation_to_pos_quat(transform_matrix):
    """
    Convert a 4x4 transformation matrix to position and quaternion.
    
    :param transform_matrix: 4x4 numpy array representing the transformation matrix
    :return: position (tuple) and quaternion (tuple)
    """
    # Extract position
    position = transform_matrix[:3, 3]

    # Extract rotation matrix
    rotation_matrix = transform_matrix[:3, :3]

    # Convert rotation matrix to quaternion
    qw = np.sqrt(1.0 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2.0
    if qw > 0.00001:  # Check for numerical stability
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4.0 * qw)
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4.0 * qw)
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4.0 * qw)
    else:
        qx = rotation_matrix[2, 1] + rotation_matrix[1, 2]
        qy = rotation_matrix[0, 2] + rotation_matrix[2, 0]
        qz = rotation_matrix[1, 0] + rotation_matrix[0, 1]
        qw = 0.0

    return np.array(position), np.array([qx, qy, qz, qw])

def pos_quat_to_transformation(position, quaternion):
    """
    Convert position and quaternion to a 4x4 transformation matrix.
    
    :param position: Tuple representing (x, y, z)
    :param quaternion: Tuple representing (qx, qy, qz, qw)
    :return: 4x4 numpy array representing the transformation matrix
    """
    qx, qy, qz, qw = quaternion

    # Create rotation matrix from quaternion
    r11 = 1 - 2 * (qy**2 + qz**2)
    r12 = 2 * (qx * qy - qw * qz)
    r13 = 2 * (qx * qz + qw * qy)
    r21 = 2 * (qx * qy + qw * qz)
    r22 = 1 - 2 * (qx**2 + qz**2)
    r23 = 2 * (qy * qz - qw * qx)
    r31 = 2 * (qx * qz - qw * qy)
    r32 = 2 * (qy * qz + qw * qx)
    r33 = 1 - 2 * (qx**2 + qy**2)

    # Create the transformation matrix
    transform_matrix = np.array([[r11, r12, r13, position[0]],
                                  [r21, r22, r23, position[1]],
                                  [r31, r32, r33, position[2]],
                                  [0,   0,   0,   1]])

    return transform_matrix

def se3_to_transformation(se3):
    """
    Convert a pin.SE3 object to a 4x4 transformation matrix.

    :param se3: pin.SE3 object representing the transformation
    :return: 4x4 numpy array representing the transformation matrix
    """
    # Extract rotation matrix (3x3)
    rotation = se3.rotation

    # Extract translation vector (3x1)
    translation = se3.translation

    # Create the transformation matrix
    transformation_matrix = np.eye(4)  # 4x4 identity matrix
    transformation_matrix[:3, :3] = rotation  # Set the rotation
    transformation_matrix[:3, 3] = translation.flatten()  # Set the translation

    return transformation_matrix

def rpy2quat(rpy, order='xyz'):
    r = R.from_euler(order, rpy, degrees=False)
    quat = r.as_quat()
    return quat

def quat2rpy(quat, order='xyz'):
    r = R.from_quat(quat)
    rpy = r.as_euler(order, degrees=False)
    return rpy

def quat2SE3(quat: np.ndarray, position: np.ndarray):
    se3 = pin.SE3(
        pin.Quaternion(quat[0], quat[1], quat[2], quat[3]),
        position
    )
    return se3

def rpy2SE3(rpy: np.ndarray, position: np.ndarray):
    quat = rpy2quat(rpy)
    se3 = quat2SE3(quat, position)
    return se3
