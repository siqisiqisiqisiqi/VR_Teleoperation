#!/usr/bin/env python3
import sys
import numpy as np

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R


class PoseSliderGUI(QWidget):
    def __init__(self, initial_pose):
        super().__init__()
        self.names = ["x", "y", "z", "roll", "pitch", "yaw"]
        self.max_vals = [1.0, 1.0, 1.0, 180, 180, 180]
        self.min_vals = [-1.0, -1.0, -1.0, -180, -180, -180]
        self.sliders = []
        self.labels = []
        self.initial_pose = initial_pose
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.setWindowTitle("Pose Slider (x, y, z, r, p, y)")

        for i, name in enumerate(self.names):
            real_val = self.initial_pose[i]
            label = QLabel(f"{name}: {real_val:.2f}")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)

            norm_val = int((real_val - self.min_vals[i]) /
                           (self.max_vals[i] - self.min_vals[i]) * 1000)
            slider.setValue(np.clip(norm_val, 0, 1000))

            def make_callback(i=i, label=label):
                def callback(val):
                    real_val = self.min_vals[i] + (val / 1000.0) * \
                        (self.max_vals[i] - self.min_vals[i])
                    label.setText(f"{self.names[i]}: {real_val:.2f}")
                return callback

            slider.valueChanged.connect(make_callback())
            layout.addWidget(label)
            layout.addWidget(slider)

            self.sliders.append(slider)
            self.labels.append(label)

        self.setLayout(layout)

    def get_slider_values(self):
        values = []
        for i, slider in enumerate(self.sliders):
            val = slider.value()
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
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_pose)
        self.timer.start(50)  # 20 Hz

    def publish_pose(self):
        values = self.gui.get_slider_values()
        position = values[:3]
        rpy = values[3:]
        q = R.from_euler('xyz', rpy, degrees=True).as_quat()

        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = position
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        self.publisher_.publish(msg)


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
