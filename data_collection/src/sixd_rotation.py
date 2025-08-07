import numpy as np
from scipy.spatial.transform import Rotation as R


def quaternion_to_matrix(q):
    """
    Convert quaternion (x, y, z, w) to 3x3 rotation matrix.
    """
    rot = R.from_quat([q[0], q[1], q[2], q[3]])  # (x, y, z, w)
    r_mat = rot.as_matrix()
    return r_mat


def matrix_to_quaternion(rot_matrix):
    """
    Convert 3x3 rotation matrix to quaternion (x, y, z, w)
    """
    r = R.from_matrix(rot_matrix)
    quat = r.as_quat()
    return quat


def so3_to_sixd(q):
    """
    Convert 4D quaternion (x,y,z,w) to 6D representation.
    """
    R = quaternion_to_matrix(q)
    return R[:, :2].T.reshape(-1)


def normalize(v):
    return v / (np.linalg.norm(v) + 1e-8)


def sixd_to_so3(sixd):
    """
    Convert 6D vector back to 3x3 rotation matrix using Gram-Schmidt.
    """
    a1 = sixd[:3]
    a2 = sixd[3:]

    b1 = normalize(a1)
    proj = np.dot(b1, a2) * b1
    b2 = normalize(a2 - proj)
    b3 = np.cross(b1, b2)

    R = np.stack([b1, b2, b3], axis=1)

    quat = matrix_to_quaternion(R)
    return quat
