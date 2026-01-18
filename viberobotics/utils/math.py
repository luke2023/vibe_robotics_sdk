import numpy as np

RAD_TO_STEP = 4096 / (2 * np.pi)
STEP_TO_RAD = (2 * np.pi) / 4096

def rad2step(x):
    return 4096 / (2 * np.pi) * (x + np.pi)
def step2rad(x):
    return (2 * np.pi) / 4096 * x - np.pi

def quat_2_rpy(q, scalar_first=False):
    if not scalar_first:
        x, y, z, w = q
    else:
        w, x, y, z = q
    
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp =   2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    rpy = np.array([roll, pitch, yaw], dtype=np.float32)
    return rpy



def rotate_vector_inverse_rpy(roll, pitch, yaw, vector):
    """
    Rotate a vector by the inverse of the given roll, pitch, and yaw angles.

    Parameters:
    roll (float): The roll angle in radians.
    pitch (float): The pitch angle in radians.
    yaw (float): The yaw angle in radians.
    vector (np.ndarray): The 3D vector to be rotated.

    Returns:
    np.ndarray: The rotated 3D vector.
    """
    R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return (R_z @ R_y @ R_x).T @ vector

def rotate_vector_rpy(roll, pitch, yaw, vector):
    """
    Rotate a vector by the given roll, pitch, and yaw angles.

    Parameters:
    roll (float): The roll angle in radians.
    pitch (float): The pitch angle in radians.
    yaw (float): The yaw angle in radians.
    vector (np.ndarray): The 3D vector to be rotated.

    Returns:
    np.ndarray: The rotated 3D vector.
    """
    R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return (R_z @ R_y @ R_x) @ vector

def quat_mult(q1, q2):
    # Quaternion multiplication
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def apply_quat(quat, vec):
    # Apply quaternion rotation to vector
    q = quat
    v = np.array([0] + list(vec))
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    v_rot = quat_mult(quat_mult(q, v), q_conj)
    return v_rot[1:]

def z_rot_quat(angle):
    # Quaternion (w, x, y, z) for a rotation about +Z by `angle`
    half = angle / 2.0
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])


def quat_inv(q):
    w, x, y, z = q
    norm_sq = w * w + x * x + y * y + z * z
    return np.array([w / norm_sq, -x / norm_sq, -y / norm_sq, -z / norm_sq], dtype=np.float32)

def quat_mul_vec(q, v):
    q_v = np.array([0.0, v[0], v[1], v[2]], dtype=np.float32)
    q_conj = quat_inv(q)
    q_res = quat_mult(quat_mult(q, q_v), q_conj)
    return q_res[1:]