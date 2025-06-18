#!/usr/bin/env python3
import os
import pickle
import re

import numpy as np
from rclpy.node import Node
import rclpy
import time
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from threading import Lock
from numpy.linalg import norm

bridge = CvBridge()


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # ROS Params
        self.timer_freq = 5  # Hz
        self.save_dir = './my_dataset'
        os.makedirs(self.save_dir, exist_ok=True)

        # Data Buffers
        self.lock = Lock()
        self.current_state = None
        self.current_action = None
        self.cam_left = None
        self.cam_right = None
        self.state = "wait"
        self.running = True

        # Subscriptions
        self.create_subscription(
            TFMessage, '/tf_right_wrist', self.tf_callback, 10)
        self.create_subscription(
            JointState, '/joint_states_isaac', self.joint_callback, 10)
        self.create_subscription(Pose, '/hand_pose_ik', self.pose_callback, 10)
        self.create_subscription(
            JointState, '/joint_command', self.hand_state_callback, 10)
        self.create_subscription(
            Image, '/rgb_left', self.left_image_callback, 10)
        self.create_subscription(
            Image, '/rgb_right', self.right_image_callback, 10)
        self.create_subscription(
            String, '/recording_trigger', self.state_callback, 10)

        # For action
        self.ee_pose = None
        self.gripper_state = 0
        self.hand_joint = [
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ]

        # Timer for recording
        self.episode = []
        self.frame_idx = 0
        existing_files = os.listdir(self.save_dir)
        episode_indices = []

        for fname in existing_files:
            match = re.match(r"episode_(\d+)\.pkl", fname)
            if match:
                episode_indices.append(int(match.group(1)))

        self.episode_idx = max(episode_indices) + 1 if episode_indices else 0

        self.timer = self.create_timer(
            1.0 / self.timer_freq, self.record_callback)

    def state_callback(self, msg):
        self.state = msg.data
        # self.get_logger().info(f"The recording state is {self.state}.")

    def tf_callback(self, msg):
        if not msg.transforms:
            return
        tf = msg.transforms[0].transform
        pos = tf.translation
        ori = tf.rotation
        with self.lock:
            if self.current_state is None:
                self.current_state = [0.0] * 8
            self.current_state[0:3] = [pos.x, pos.y, pos.z]
            self.current_state[3:7] = [ori.x, ori.y, ori.z, ori.w]

    def joint_callback(self, msg):
        with self.lock:
            if self.current_state is None:
                self.current_state = [0.0] * 8
            position = msg.position
            name_list = msg.name
            finger_config = np.zeros(len(self.hand_joint))
            for i, joints in enumerate(self.hand_joint):
                idx = name_list.index(joints)
                finger_config[i] = position[idx]
            self.current_state[7] = 1 if norm(finger_config[:4]) > 0.5 else 0

    def pose_callback(self, msg):
        self.ee_pose = msg

    def hand_state_callback(self, msg):
        with self.lock:
            position = msg.position
            name_list = msg.name
            finger_config = np.zeros(len(self.hand_joint))
            self.get_logger().info("1")
            try:
                for i, joints in enumerate(self.hand_joint):
                    idx = name_list.index(joints)
                    finger_config[i] = position[idx]
                self.get_logger().info("3")
                self.gripper_state = 1 if norm(finger_config[:4]) > 0.5 else 0
                self.get_logger().info(
                    f"command gripper state is {norm(finger_config[:4])}")
            except:
                pass

    def left_image_callback(self, msg):
        self.cam_left = bridge.imgmsg_to_cv2(msg, "bgr8")

    def right_image_callback(self, msg):
        self.cam_right = bridge.imgmsg_to_cv2(msg, "bgr8")

    def get_state(self):
        with self.lock:
            return list(self.current_state) if self.current_state else [0.0] * 8

    def get_action(self):
        if self.ee_pose is None:
            return [0.0] * 7 + [self.gripper_state]
        pos = self.ee_pose.position
        ori = self.ee_pose.orientation
        return [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, self.gripper_state]

    def get_camera_images(self):
        return {
            "cam1": self.cam_left if self.cam_left is not None else np.zeros((480, 640, 3), np.uint8),
            "cam2": self.cam_right if self.cam_right is not None else np.zeros((480, 640, 3), np.uint8)
        }

    def record_callback(self):
        if self.state == "wait":
            time.sleep(0.2)
            return
        state = self.get_state()
        action = self.get_action()
        images = self.get_camera_images()

        record = {
            "observation": {
                "state": state,
                "images": images
            },
            "action": action
        }
        self.episode.append(record)

        # self.get_logger().info(
        #     f"Recording step {self.frame_idx}: action={action}")
        # self.get_logger().info(
        #     f"output gripper state is {self.gripper_state}, current gripper state is {self.current_state[-1]}.")
        self.frame_idx = self.frame_idx + 1

        if self.state == "stop":  # example episode length
            filename = os.path.join(
                self.save_dir, f"episode_{self.episode_idx:03d}.pkl")
            with open(filename, 'wb') as f:
                pickle.dump(self.episode, f)
            self.get_logger().info(
                f"Saved episode {self.episode_idx} with {len(self.episode)} steps")

            # reset
            self.running = False


def main():
    rclpy.init()
    node = DataCollector()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
