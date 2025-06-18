#!/usr/bin/env python3

import numpy as np
from pynput import keyboard

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R

KEY_BINDINGS_POSE = {
    'w': ('pos', 0, 0.001),  # X+
    's': ('pos', 0, -0.001),  # X-
    'a': ('pos', 1, 0.001),  # Y+
    'd': ('pos', 1, -0.001),  # Y-
    'q': ('pos', 2, 0.001),  # Z+
    'e': ('pos', 2, -0.001),  # Z-
    'i': ('rpy', 0, 1.0),   # Roll+
    'k': ('rpy', 0, -1.0),   # Roll-
    'j': ('rpy', 1, 1.0),   # Pitch+
    'l': ('rpy', 1, -1.0),   # Pitch-
    'u': ('rpy', 2, 1.0),   # Yaw+
    'o': ('rpy', 2, -1.0),   # Yaw-
}


class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.pose_received = False
        self.initial_pose = None
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_right_wrist',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        if self.pose_received or not msg.transforms:
            return
        t = msg.transforms[0].transform
        pos = [t.translation.x, t.translation.y, t.translation.z - 1.012]
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        self.initial_pose = pos + list(rpy)
        self.pose_received = True


class KeyboardTeleopNode(Node):
    def __init__(self, initial_pose):
        super().__init__('keyboard_teleop_node')

        self.publisher_ = self.create_publisher(Pose, '/hand_pose_ik', 10)
        # self.hand_pub_ = self.create_publisher(
        #     JointState, '/joint_command', 10)
        self.hand_state_pub_ = self.create_publisher(Bool, '/hand_state', 10)
        self.trigger_pub_ = self.create_publisher(
            String, '/recording_trigger', 10)

        self.pos = np.array(initial_pose[:3])
        self.rpy = np.array(initial_pose[3:])
        self.hand_closed = False
        self.recording = False

        self.timer = self.create_timer(0.2, self.publish_pose)

        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        self.get_logger().info(
            "Keyboard teleop running: use WASDQE, IJKLUO, H (hand), Space (start/stop)")

    def on_press(self, key):
        try:
            if key == keyboard.Key.space:
                self.recording = not self.recording
                msg = String()
                msg.data = 'start' if self.recording else 'stop'
                self.trigger_pub_.publish(msg)
                self.get_logger().info(f"[Trigger] {msg.data.upper()}")
                return
            k = key.char
            # self.get_logger().info(f'Detected the keyboard input {k}.')
            if k in KEY_BINDINGS_POSE:
                mode, idx, delta = KEY_BINDINGS_POSE[k]
                if mode == 'pos':
                    self.pos[idx] += delta
                elif mode == 'rpy':
                    self.rpy[idx] += delta
            elif k == 'h':
                self.hand_closed = not self.hand_closed
                self.publish_hand_state()
                self.publish_hand()
        except AttributeError:
            if key == keyboard.Key.esc:
                self.get_logger().info("Exiting...")
                rclpy.shutdown()

    def publish_pose(self):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.pos
        q = R.from_euler('xyz', self.rpy, degrees=True).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        self.publisher_.publish(pose)

    # def publish_hand(self):
    #     msg = JointState()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.name = [
    #         "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
    #         "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
    #     ]
    #     msg.position = [0.80, 0.75, 0.75, 0.75, 1.20,
    #                     0.30] if self.hand_closed else [0.0] * 4 + [1.2, 0.0]

    #     self.hand_pub_.publish(msg)

    def publish_hand_state(self):
        msg = Bool()
        msg.data = False if self.hand_closed else True
        self.hand_state_pub_.publish(msg)


def wait_for_initial_pose():
    tf_node = TFListenerNode()
    while rclpy.ok() and not tf_node.pose_received:
        rclpy.spin_once(tf_node)
    pose = tf_node.initial_pose
    tf_node.destroy_node()
    return pose


def main():
    rclpy.init()
    initial_pose = wait_for_initial_pose()
    node = KeyboardTeleopNode(initial_pose)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
