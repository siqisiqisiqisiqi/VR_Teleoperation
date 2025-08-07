#!/usr/bin/env python3
import os
import re
import time
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(PARENT_DIR)

import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from threading import Lock
from numpy.linalg import norm
import cv2

from src.save_data import save_data

bridge = CvBridge()


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # ROS Params
        self.timer_freq = 10  # Hz
        self.save_dir = './my_dataset'
        os.makedirs(self.save_dir, exist_ok=True)

        # Data Buffers
        self.lock = Lock()
        self.current_joint = None
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
            JointState, '/joint_command', self.action_callback, 10)
        self.create_subscription(
            Image, '/rgb_global', self.global_image_callback, 10)
        self.create_subscription(
            Image, '/rgb_right', self.right_image_callback, 10)
        self.create_subscription(
            Image, '/rgb_wrist', self.wrist_image_callback, 10)
        self.create_subscription(
            String, '/recording_trigger', self.state_callback, 10)

        # For action
        self.state_action = None
        self.joint_action = None
        self.gripper_state = 0
        self.hand_joint = [
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ]
        self.arm_joint = [
            "right_shoulder_pitch", "right_shoulder_roll",
            "right_elbow_yaw", "right_elbow_pitch",
            "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
        ]

        # Timer for recording
        self.episode = {
            "observation.joint": [],
            "observation.state": [],
            "observation.images": [],
            "action.state": [],
            "action.joint": [],
        }
        self.frame_idx = 0
        existing_files = os.listdir(self.save_dir)
        episode_indices = []

        for folder_name in existing_files:
            match = re.match(r"episode_(\d+)", folder_name)
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
        """joint state subscription
        """
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

            self.current_joint = [0.0] * len(self.arm_joint)
            for i, joints in enumerate(self.arm_joint):
                idx = name_list.index(joints)
                self.current_joint[i] = position[idx]

    def pose_callback(self, msg):
        """keyboard task space action
        """
        self.state_action = msg

    def action_callback(self, msg):
        with self.lock:
            position = msg.position
            name_list = msg.name

            finger_config = np.zeros(len(self.hand_joint))
            try:
                for i, joints in enumerate(self.hand_joint):
                    idx = name_list.index(joints)
                    finger_config[i] = position[idx]
                self.gripper_state = 1 if norm(finger_config[:4]) > 0.5 else 0

                self.joint_action = np.zeros(len(self.arm_joint))
                for i, joints in enumerate(self.arm_joint):
                    idx = name_list.index(joints)
                    self.joint_action[i] = position[idx]
            except:
                pass

    def global_image_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cam_global = cv2.resize(
            img, (640, 360), interpolation=cv2.INTER_AREA)

    def right_image_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cam_right = cv2.resize(
            img, (640, 360), interpolation=cv2.INTER_AREA)

    def wrist_image_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cam_wrist = cv2.resize(
            img, (640, 360), interpolation=cv2.INTER_AREA)

    def get_state(self):
        with self.lock:
            return list(self.current_state) if self.current_state else [0.0] * 8

    def get_joint(self):
        with self.lock:
            return list(self.current_joint) if self.current_joint else [0.0] * len(self.arm_joint)

    def get_state_action(self):
        if self.state_action is None:
            return [0.0] * 7 + [self.gripper_state]
        pos = self.state_action.position
        ori = self.state_action.orientation
        return [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, self.gripper_state]

    def get_joint_action(self):
        if self.joint_action is None:
            return [0.0] * 7
        return list(self.joint_action)

    def get_camera_images(self):
        return {
            "cam1": self.cam_global if self.cam_global is not None else np.zeros((360, 640, 3), np.uint8),
            "cam2": self.cam_right if self.cam_right is not None else np.zeros((360, 640, 3), np.uint8),
            "cam3": self.cam_wrist if self.cam_wrist is not None else np.zeros((360, 640, 3), np.uint8)
        }

    def record_callback(self):
        if self.state == "wait":
            time.sleep(0.2)
            return
        state = self.get_state()
        joint = self.get_joint()
        state_action = self.get_state_action()
        joint_action = self.get_joint_action()
        images = self.get_camera_images()

        self.episode["observation.joint"].append(joint)
        self.episode["observation.state"].append(state)
        self.episode["observation.images"].append(images)
        self.episode["action.state"].append(state_action)
        self.episode["action.joint"].append(joint_action)

        self.frame_idx = self.frame_idx + 1

        if self.state == "stop":  # example episode length

            output_folder = os.path.join(
                self.save_dir, f"episode_{self.episode_idx:03d}")
            os.makedirs(output_folder, exist_ok=True)

            save_data(self.episode, self.episode_idx, output_folder)
            self.get_logger().info(
                f"Saved episode {self.episode_idx} with {self.frame_idx} steps")

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
