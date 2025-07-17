#!/usr/bin/env python3

import time
import numpy as np
import pyspacemouse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R


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


class SpaceMouseTeleop(Node):
    def __init__(self, initial_pose):
        super().__init__('space_mouse_teleop')

        self.publisher_ = self.create_publisher(Pose, '/hand_pose_ik', 10)
        self.hand_state_pub_ = self.create_publisher(Bool, '/hand_state', 10)
        self.trigger_pub_ = self.create_publisher(
            String, '/recording_trigger', 10)

        self.pos = np.array(initial_pose[:3])
        # self.rpy = np.array(initial_pose[3:])
        self.orientation = R.from_euler('xyz', initial_pose[3:], degrees=True)

        self.hand_closed = False
        self.recording = False

        self.last_read_time = time.time()

        # Open SpaceMouse device
        button_arr = [
            pyspacemouse.ButtonCallback(0, self.toggle_hand),
            pyspacemouse.ButtonCallback([14], self.toggle_recording),
        ]
        success = pyspacemouse.open(
            dof_callback=self.update_pose, button_callback_arr=button_arr)
        if not success:
            raise RuntimeError("Failed to open SpaceMouse!")

        self.get_logger().info(
            "SpaceMouse teleop running: move to control pose, LeftBtn to toggle hand, RightBtn to start/stop recording")

        # Timer for publishing pose at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_pose)

    def update_pose(self, state):
        if not state:
            return

        dt = 0.01
        # Translation (scale as needed)
        self.pos += np.array([-1 * state.y, state.x, state.z]) * dt * 0.1

        # Rotation
        delta_rpy_radian = np.array(
            [-1 * state.yaw, -1 * state.pitch, state.roll]) * dt * 0.5
        delta_rot = R.from_euler('xyz', delta_rpy_radian, degrees=False)
        # print(f"rotation is {state.roll}, {state.pitch}, {state.yaw}.")
        self.orientation = self.orientation * delta_rot

    def toggle_hand(self, state, buttons, pressed_buttons):
        self.hand_closed = not self.hand_closed
        self.publish_hand_state()
        self.get_logger().info(
            f"[Gripper] {'Closed' if self.hand_closed else 'Open'}")

    def toggle_recording(self, state, buttons, pressed_buttons):
        self.recording = not self.recording
        msg = String()
        msg.data = 'start' if self.recording else 'stop'
        self.trigger_pub_.publish(msg)
        self.get_logger().info(f"[Recording] {msg.data.upper()}")

    def publish_pose(self):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.pos
        # q = R.from_euler('xyz', self.rpy, degrees=True).as_quat()
        q = self.orientation.as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        self.publisher_.publish(pose)

    def publish_hand_state(self):
        msg = Bool()
        msg.data = not self.hand_closed  # True = open, False = closed
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
    node = SpaceMouseTeleop(initial_pose)

    try:
        while rclpy.ok():
            pyspacemouse.read()
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
