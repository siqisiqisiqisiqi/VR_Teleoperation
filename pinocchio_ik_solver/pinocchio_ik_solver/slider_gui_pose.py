#!/usr/bin/env python3
import sys
import numpy as np

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QCheckBox
from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R


class PoseSliderGUI(QWidget):
    def __init__(self, initial_pose):
        super().__init__()
        self.names = ["x", "y", "z", "roll", "pitch", "yaw", "hand"]
        self.max_vals = [1.0, 1.0, 1.0, 180, 180, 180, 1]
        self.min_vals = [-1.0, -1.0, -1.0, -180, -180, -180, 0]
        self.sliders = []
        self.labels = []
        self.initial_pose = initial_pose + [1]
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.setWindowTitle("Pose Slider (x, y, z, r, p, y) + Hand Control")

        for i, name in enumerate(self.names):
            real_val = self.initial_pose[i]
            label = QLabel(f"{name}: {real_val:.2f}" if name !=
                           "hand" else f"{name}: Closed" if real_val else f"{name}: Open")
            layout.addWidget(label)
            self.labels.append(label)

            if name == "hand":
                checkbox = QCheckBox("Grasp Closed")
                checkbox.setChecked(bool(real_val))
                checkbox.stateChanged.connect(lambda state, label=label: label.setText(
                    f"{name}: {'Closed' if state else 'Open'}"))
                layout.addWidget(checkbox)
                self.sliders.append(checkbox)
            else:
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(0)
                slider.setMaximum(1000)
                norm_val = int(
                    (real_val - self.min_vals[i]) / (self.max_vals[i] - self.min_vals[i]) * 1000)
                slider.setValue(np.clip(norm_val, 0, 1000))

                def make_callback(i=i, label=label):
                    def callback(val):
                        real_val = self.min_vals[i] + (val / 1000.0) * (
                            self.max_vals[i] - self.min_vals[i])
                        label.setText(f"{self.names[i]}: {real_val:.2f}")
                    return callback

                slider.valueChanged.connect(make_callback())
                layout.addWidget(slider)
                self.sliders.append(slider)

        self.setLayout(layout)

    def get_slider_values(self):
        values = []
        for i, name in enumerate(self.names):
            if name == "hand":
                values.append(1 if self.sliders[i].isChecked() else 0)
            else:
                val = self.sliders[i].value()
                real_val = self.min_vals[i] + (val / 1000.0) * \
                    (self.max_vals[i] - self.min_vals[i])
                values.append(real_val)
        return values


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


class PosePublisherNode(Node):
    def __init__(self, gui: PoseSliderGUI):
        super().__init__('pose_slider_publisher')
        self.gui = gui
        self.publisher_ = self.create_publisher(Pose, '/hand_pose_ik', 10)
        self.hand_publisher_ = self.create_publisher(
            JointState, 'joint_command', 10)
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_pose)
        self.timer.start(50)  # 20 Hz

    def publish_pose(self):
        values = self.gui.get_slider_values()
        position = values[:3]
        rpy = values[3:6]
        q = R.from_euler('xyz', rpy, degrees=True).as_quat()
        hand_state = int(round(values[6]))  # ensure 0 or 1

        # Send hand pose
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = position
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = q
        self.publisher_.publish(pose_msg)

        # Send finger command
        joint_names = [
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ]
        if hand_state:
            finger_positions = [0.55, 0.80, 0.6, 0.60, 1.16, 0.13]
        else:
            finger_positions = [0.0, 0.0, 0.0, 0.0, 1.2, 0.0]

        hand_msg = JointState()
        hand_msg.header.stamp = self.get_clock().now().to_msg()
        hand_msg.name = joint_names
        hand_msg.position = finger_positions
        self.hand_publisher_.publish(hand_msg)


def wait_for_initial_pose():
    node = TFListenerNode()
    while rclpy.ok() and not node.pose_received:
        rclpy.spin_once(node)
    initial_pose = node.initial_pose
    node.destroy_node()
    return initial_pose


def main():
    rclpy.init()
    initial_pose = wait_for_initial_pose()

    app = QApplication(sys.argv)
    gui = PoseSliderGUI(initial_pose)
    gui.show()

    pub_node = PosePublisherNode(gui)

    app.exec_()

    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
