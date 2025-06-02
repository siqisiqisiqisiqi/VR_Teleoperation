#!/usr/bin/env python3
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig
from pytransform3d import rotations
from pytransform3d.rotations import euler_from_matrix
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from numpy.linalg import norm

from pathlib import Path
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from multiprocessing import shared_memory, Queue, Event


class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (
            self.resolution[0] - self.crop_size_h, self.resolution[1] - 2 * self.crop_size_w)

        self.img_shape = (
            self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(
            create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray(
            (self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        # self.tv = OpenTeleVision(
        #     self.resolution_cropped, self.shm.name, image_queue, toggle_streaming)
        self.tv = OpenTeleVision(
            self.resolution_cropped, self.shm.name, image_queue, toggle_streaming, ngrok=True)
        self.processor = VuerPreprocessor()

        RetargetingConfig.set_default_urdf_dir('../assets')
        with Path(config_file_path).open('r') as f:
            cfg = yaml.safe_load(f)
        left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()

    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process_v2(
            self.tv)

        head_rmat = head_mat[:3, :3]

        left_pose = np.concatenate([left_wrist_mat[:3, 3] + np.array([0.2, 0.1, 0.3]),
                                    rotations.quaternion_from_matrix(left_wrist_mat[:3, :3])[[1, 2, 3, 0]]])
        right_pose = np.concatenate([right_wrist_mat[:3, 3] + np.array([0.2, 0.1, 0.3]),
                                     rotations.quaternion_from_matrix(right_wrist_mat[:3, :3])[[1, 2, 3, 0]]])

        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[
            [4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[
            [4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]

        return head_rmat, left_pose, right_pose, left_qpos, right_qpos


class RosBridge(Node):
    def __init__(self):
        super().__init__('teleop_image_bridge')
        # self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
        #                                                rclpy.Parameter.Type.BOOL,
        #                                                True)])
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None

        # Subscribe to left and right camera Image topics
        self.create_subscription(
            Image,
            '/rgb_left',  # left camera topic
            self._left_callback,
            qos_profile=10
        )
        self.create_subscription(
            Image,
            '/rgb_right',  # right camera topic
            self._right_callback,
            qos_profile=10
        )
        self.create_subscription(
            TFMessage,
            '/tf_right_wrist',  # Topic name
            self._tf_callback,
            10  # QoS profile
        )

        self.publisher_ = self.create_publisher(
            JointState, 'joint_command', 10)
        self.hand_pos_pub = self.create_publisher(Pose, '/hand_pose', 10)
        self.hand_pos_ik_pub = self.create_publisher(Pose, '/hand_pose_ik', 10)

        self.get_logger().info('Subscribed to /rgb_left and /rgb_right')

        self.gripper_index = [0, 2, 4, 6, 8, 10]
        self.pos_error_norm = 10
        self.trans_r = None
        self.quaternion_r = None
        self.head_trans = None
        self.head_quaternion = None

    def _tf_callback(self, msg):
        try:
            # Access the first transform in the message
            transform = msg.transforms[0]

            # Extract the frame IDs
            self.parent_frame = transform.header.frame_id
            child_frame = transform.child_frame_id

            # Extract translation
            translation = transform.transform.translation
            x, y, z = translation.x, translation.y, translation.z
            self.trans_r = np.array([x, y, z])

            # Extract rotation (quaternion)
            rotation = transform.transform.rotation
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
            self.quaternion_r = [qx, qy, qz, qw]

        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

    def _left_callback(self, msg: Image):
        # Convert ROS Image to numpy RGB
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.left_image = cv_img
        except Exception as e:
            self.get_logger().error(f'Left image conversion failed: {e}')

    def _right_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.right_image = cv_img
        except Exception as e:
            self.get_logger().error(f'Right image conversion failed: {e}')

    def publish_hand_pose(self, position, quaternion, IK=False):
        # Create the Pose message
        pose_msg = Pose()

        # Set the position using the list
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]

        # Set the orientation using the quaternion list
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]

        # Publish the message
        if IK == False:
            self.hand_pos_pub.publish(pose_msg)
        else:
            self.hand_pos_ik_pub.publish(pose_msg)
            self.get_logger().info(f'Published hand pose: {pose_msg}')

    def step(self, head_rmat, right_qpos, right_pose):
        msg_data = JointState()
        msg_data.header.stamp = self.get_clock().now().to_msg()

        # head control
        msg_data.name = [
            "head_yaw", "head_pitch", "head_roll"
        ]
        yaw, pitch, roll = euler_from_matrix(
            head_rmat,
            i=2,   # first rotation about Z
            j=1,   # then about Y
            k=0,   # then about X
            extrinsic=False,
            strict_check=True  # optional: will throw if R isn’t a valid rotation
        )
        msg_data.position = [yaw, -1 * pitch, roll]

        # hand control
        msg_data.name.extend([
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ])
        right_gripper_pose = right_qpos[self.gripper_index]

        order = [0, 2, 1, 3, 4, 5]
        values = [float(right_gripper_pose[i]) for i in order]
        msg_data.position.extend(values)
        # publish the data
        self.publisher_.publish(msg_data)

        # arm control
        if self.trans_r is not None and self.quaternion_r is not None:
            # transform and rotation in the head frame
            trans_d = right_pose[:3]
            quat_d = right_pose[3:]  # x,y,z,w

            pos_error = trans_d - self.trans_r
            self.pos_error_norm = norm(pos_error)
            if self.pos_error_norm < 0.2:
                self.publish_hand_pose(
                    trans_d - np.array([0, 0, 1.01]), quat_d, IK=True)
            self.publish_hand_pose(trans_d, quat_d, IK=False)


def main(args=None):
    rclpy.init(args=args)

    # Teleoperation frontend (shared memory + vision processing)
    teleoperator = VuerTeleop('inspire_hand.yml')

    # ROS2 bridge for images
    ros_bridge = RosBridge()

    try:
        # Spin ROS2 in background
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(ros_bridge)

        while rclpy.ok():
            # Process ROS2 callbacks
            executor.spin_once(timeout_sec=0.05)

            # Run teleop inference
            head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
            ros_bridge.step(head_rmat, right_qpos, right_pose)

            # Retrieve latest stereo images
            left_img = ros_bridge.left_image
            right_img = ros_bridge.right_image
            if left_img is None or right_img is None:
                continue  # wait until we have both pubs

            # Stack and write into shared memory for TeleVision consumer
            combined = np.hstack((left_img, right_img))
            np.copyto(teleoperator.img_array, combined)

    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        ros_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
