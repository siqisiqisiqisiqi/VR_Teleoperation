#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
                                                       rclpy.Parameter.Type.BOOL,
                                                       True)])
        self.publisher = self.create_publisher(
            JointState, '/joint_command', 10)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states_isaac',
            self.joint_state_callback,
            10
        )
        self.received = False
        self.latest_msg = None

    def joint_state_callback(self, msg):
        self.received = True
        self.latest_msg = msg
        self.get_logger().info('Received first /joint_states message.')

    def publish_initial_joint_command(self):
        time.sleep(1)
        p1 = [0.00, -0.8, 0.5, 1.1, 0.06, 0.16, 0.11]
        p2 = [0.00, -0.75, 0.14, 1.77, 1.09, 0.00, 0.00]
        msg = JointState()
        msg.name = [
            "right_shoulder_pitch", "right_shoulder_roll",
            "right_elbow_yaw", "right_elbow_pitch",
            "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
        ]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = p1
        self.publisher.publish(msg)
        self.get_logger().info('Send the first pos!!!')
        time.sleep(2)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = p2
        self.publisher.publish(msg)
        self.get_logger().info('Send the second pos!!!')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    while not node.received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(
        f'Proceeding with robot startup after receiving: {node.latest_msg.position}')
    node.publish_initial_joint_command()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
