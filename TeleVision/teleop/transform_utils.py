import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_cube_to_world(head_pos, head_quat, cube_pos, cube_quat):
    """
    Transform the cube position and orientation from the head frame to the world frame.

    Args:
        head_pos (list): Position of the head in the world frame [x, y, z].
        head_quat (list): Orientation of the head in the world frame [x, y, z, w].
        cube_pos (list): Position of the cube in the head frame [x, y, z].
        cube_quat (list): Orientation of the cube in the head frame [x, y, z, w].

    Returns:
        world_pos (list): Transformed position in the world frame [x, y, z].
        world_quat (list): Transformed orientation in the world frame [x, y, z, w].
    """
    # Convert head quaternion to rotation matrix
    head_rotation = R.from_quat(head_quat)
    rotation_matrix = head_rotation.as_matrix()

    # Transform the cube position to the world frame
    transformed_pos = np.dot(rotation_matrix, cube_pos) + head_pos

    # Multiply quaternions for the new orientation
    cube_rotation = R.from_quat(cube_quat)
    world_rotation = head_rotation * cube_rotation
    world_quat = world_rotation.as_quat()

    return transformed_pos, world_quat