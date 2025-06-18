#!/usr/bin/env python3

import pathlib
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
import pinocchio as pin
import time
from scipy.spatial.transform import Rotation as R


class PoseIKController(Node):
    def __init__(self):
        super().__init__('pose_ik_controller')
        # self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
        #                                                rclpy.Parameter.Type.BOOL,
        #                                                True)])

        self.current_joint_state = [None] * 7
        self.latest_pose_msg = None
        self.hand_state = True
        self.vr_joint = None

        self.target_joint_names = [
            "right_shoulder_pitch", "right_shoulder_roll",
            "right_elbow_yaw", "right_elbow_pitch",
            "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
        ]

        self.hand_joint_names = [
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ]

        self.ordered_joint_names = None

        self.setup_ros()
        self.setup_robot()

        # Create a periodic timer to run the update loop
        self.timer = self.create_timer(0.05, self.update_robot)  # 20Hz

    def setup_ros(self):
        self.get_logger().info('Setting up ROS I/O...')
        self.publisher = self.create_publisher(
            JointState, '/joint_command', 10)
        self.create_subscription(
            JointState, '/joint_states_isaac', self.joint_state_callback, 10)
        self.create_subscription(
            JointState, '/vr_joint', self.vr_joint_callback, 10)
        self.create_subscription(Pose, '/hand_pose_ik', self.pose_callback, 10)
        self.get_logger().info("ros setup completed!")

    def vr_joint_callback(self, msg):
        # receive the joint command from the VR for head and finger
        self.vr_joint = msg

    def joint_state_callback(self, msg):
        position = msg.position
        name_list = msg.name
        name_to_index = {name: i for i, name in enumerate(name_list)}
        for i, name in enumerate(self.ordered_joint_names):
            if name in name_to_index:
                self.current_joint_state[i] = position[name_to_index[name]]
            else:
                print(f"[⚠️] Joint '{name}' not found in joint_states")

    def pose_callback(self, msg):
        self.latest_pose_msg = msg

    def setup_robot(self):
        model_path = pathlib.Path(__file__).parent.parent / "models"
        mesh_dir = model_path
        urdf_model_path = model_path / "t170a_description/urdf/T170A.urdf"
        srdf_model_path = model_path / "t170a_description/srdf/T170A.srdf"

        full_model = pin.buildModelFromUrdf(urdf_model_path)
        joints_to_lock_ids = [jid for jid, joint in enumerate(full_model.joints)
                              if full_model.names[jid] not in self.target_joint_names and full_model.names[jid] != "universe"]

        q0 = pin.neutral(full_model)
        self.model = pin.buildReducedModel(full_model, joints_to_lock_ids, q0)
        self.data = self.model.createData()

        self.ordered_joint_names = [
            name for i, name in enumerate(self.model.names)
            if self.model.joints[i].nq > 0 and name != "universe"
        ]

        self.geom_model = pin.buildGeomFromUrdf(
            self.model, urdf_model_path, pin.GeometryType.COLLISION, mesh_dir
        )
        self.geom_model.addAllCollisionPairs()
        pin.removeCollisionPairs(self.model, self.geom_model, srdf_model_path)
        self.geom_data = pin.GeometryData(self.geom_model)

        self.frame_name = "right_wrist_roll_link"
        self.frame_id = self.model.getFrameId(self.frame_name)
        self.joint_id = self.model.frames[self.frame_id].parentJoint

        pin.loadReferenceConfigurations(self.model, srdf_model_path)
        self.q_home = self.model.referenceConfigurations["right_home"]
        q = self.q_home

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.frame_id)
        self.oMdes_smoothed = self.data.oMf[self.frame_id].copy()
        self.get_logger().info("Inverse kinematic solver setup completed!")

    def interpolate_se3(self, a: pin.SE3, b: pin.SE3, alpha: float) -> pin.SE3:
        quat_a = pin.Quaternion(a.rotation)
        quat_b = pin.Quaternion(b.rotation)
        quat_interp = quat_a.slerp(alpha, quat_b)
        R = quat_interp.matrix()
        t = (1 - alpha) * a.translation + alpha * b.translation
        return pin.SE3(R, t)

    def update_robot(self):
        if self.latest_pose_msg is None or self.current_joint_state[0] is None:
            return

        # Step 1: convert geometry_msgs/Pose to SE3
        pos = self.latest_pose_msg.position
        ori = self.latest_pose_msg.orientation
        translation = np.array([pos.x, pos.y, pos.z])
        quat = pin.Quaternion(ori.w, ori.x, ori.y, ori.z)
        rotation = quat.matrix()
        oMdes = pin.SE3(rotation, translation)

        # self.oMdes_smoothed = self.interpolate_se3(
        #     self.oMdes_smoothed, oMdes, 0.8)
        self.oMdes_smoothed = oMdes

        t0 = time.time()
        # Step 2: update forward kinematics
        q_measured = np.array(self.current_joint_state)
        pin.forwardKinematics(self.model, self.data, q_measured)
        pin.updateFramePlacement(self.model, self.data, self.frame_id)
        pin.computeCollisions(self.model, self.data,
                              self.geom_model, self.geom_data, q_measured, False)

        if any(res.isCollision() for res in self.geom_data.collisionResults):
            self.get_logger().warn("❌ Collision detected. Skipping update.")
            return

        # Step 3: compute error and solve CLIK
        iMd = self.data.oMf[self.frame_id].actInv(self.oMdes_smoothed)
        err = pin.log(iMd).vector

        T = self.data.oMf[self.frame_id]
        rotation_matrix = T.rotation
        # Convert rotation matrix to RPY (in degrees)
        rpy_current = R.from_matrix(
            rotation_matrix).as_euler('xyz', degrees=False)
        current_quat = R.from_euler('xyz', rpy_current).as_quat()
        # print("📌 Pinocchio EE pose:")
        # print("desired  Translation:", translation)
        # print("current  Translation:", T.translation)
        # print(
        #     f"desired quat: x={ori.x:.2f}, y={ori.y:.2f}, z={ori.z:.2f}, w={ori.w:.2f}")
        # print(
        #     f"current quat: x={current_quat[0]:.2f}, y={current_quat[1]:.2f}, z={current_quat[2]:.2f}, w={current_quat[3]:.2f}")
        if np.linalg.norm(err) < 1e-3:
            self.get_logger().info("arrved at the target position!")
            return

        J = pin.computeJointJacobian(
            self.model, self.data, q_measured, self.joint_id)
        J = -pin.Jlog6(iMd.inverse()).dot(J)

        lambda_damp = 1e-6
        J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_damp * np.eye(6))

        v_task = -J_pinv @ err
        dq_null_desired = -0.05 * (q_measured - self.q_home)
        P_null = np.eye(self.model.nv) - J_pinv @ J
        dq_null = P_null @ dq_null_desired

        v = v_task + dq_null
        q_next = pin.integrate(self.model, q_measured, v * 1.0)

        # Step 4: publish joint command
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.ordered_joint_names
        msg.position = list(q_next)

        if self.vr_joint is not None:
            msg.name += self.vr_joint.name
            msg.position += self.vr_joint.position
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseIKController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
