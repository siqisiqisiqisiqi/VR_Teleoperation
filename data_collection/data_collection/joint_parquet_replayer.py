#!/usr/bin/env python3
import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class JointParquetReplayer(Node):
    def __init__(self):
        super().__init__('joint_parquet_replayer')

        self.loop = False
        self.unwrap_angles = False
        self.parquet_path = "./lerobot_dataset/data/chunk-000/episode_000175.parquet"
        self.topic_name = "/joint_command"

        joint_names_param = [
            "right_shoulder_pitch", "right_shoulder_roll",
            "right_elbow_yaw", "right_elbow_pitch",
            "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
        ]

        self.hand_joint = [
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ]
        self.param_joint_names = list(
            joint_names_param) if joint_names_param is not None else []

        if not self.parquet_path:
            raise RuntimeError(
                "Parameter 'parquet_path' is required (path to the .parquet file).")

        # ---------------------
        # Load dataset
        # ---------------------
        try:
            df = pd.read_parquet(self.parquet_path)
        except Exception as e:
            raise RuntimeError(
                f"Failed to read parquet file '{self.parquet_path}': {e}")

        if df.empty:
            raise RuntimeError(f"Parquet file '{self.parquet_path}' is empty.")

        if 'action' not in df.columns:
            raise RuntimeError("No 'action' column found in parquet file.")

        self.trajectory = np.stack(df['action'].values)  # shape [T, N]
        self.T, self.N = self.trajectory.shape
        self.N = self.N - 1

        # Infer or use provided joint names
        if joint_names_param:
            self.joint_names = list(joint_names_param)
            if len(self.joint_names) != self.N:
                raise ValueError(
                    f"Expected {self.N} joint names but got {len(self.joint_names)}")
        else:
            self.joint_names = [f'joint_{i}' for i in range(self.N)]

        self.get_logger().info(
            f"Loaded {self.T} action steps with {self.N} joints.")

        # ROS 2 publisher
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(
            JointState, self.topic_name, qos)

        # Replay state
        self.index = 0
        self.dt = 0.1  # 10 Hz
        self.timer = self.create_timer(self.dt, self.publish_next)

    def publish_next(self):
        if self.index >= self.T:
            if self.loop:
                self.index = 0
                self.get_logger().info("Looping trajectory.")
            else:
                self.get_logger().info("Finished replaying trajectory.")
                self.timer.cancel()
                return
        # joint value
        q = self.trajectory[self.index][:-1]
        # gripper
        hand_state = self.trajectory[self.index][-1]
        print(hand_state)
        hand_config = [0.85, 0.80, 0.80, 0.80, 1.39,
                       0.35] if hand_state > 0.5 else [0.0] * 4 + [1.2, 0.0]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names + self.hand_joint
        msg.position = q.tolist() + hand_config

        self.publisher.publish(msg)
        self.index += 1


def main():
    rclpy.init()
    try:
        node = JointParquetReplayer()
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
