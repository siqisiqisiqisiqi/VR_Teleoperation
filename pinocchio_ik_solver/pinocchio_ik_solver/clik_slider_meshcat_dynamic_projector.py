import pathlib
import copy
import sys

path = str(pathlib.Path(__file__).parent.parent)
sys.path.insert(1, path)

import pickle
import numpy as np
from PyQt5.QtWidgets import QApplication
from scipy.spatial import Delaunay
from PyQt5.QtCore import QTimer

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from utils.twist_slider_gui import PoseSliderWidget
from scipy.spatial.transform import Rotation as R


# switch different arm: left or right
mode = "right"
with open(f"./workspace/{mode}_arm_workspace.pkl", "rb") as f:
    data = pickle.load(f)
    positions = data["positions"]

hull_delaunay = Delaunay(positions)


def is_pose_reachable(position: np.ndarray) -> bool:
    return hull_delaunay.find_simplex(position) >= 0


class PoseIKController:
    def __init__(self, enable_meshcat=True):
        self.enable_meshcat = enable_meshcat
        self.setup_robot()
        self.setup_gui()
        self.setup_timer()

    def setup_robot(self):
        model_path = pathlib.Path(__file__).parent.parent / "models"
        mesh_dir = model_path
        urdf_model_path = model_path / "t170a_description/urdf/T170A.urdf"
        srdf_model_path = model_path / "t170a_description/srdf/T170A.srdf"

        # Load full model and get joints to lock
        full_model, full_collision_model, full_visual_model = pin.buildModelsFromUrdf(
            urdf_model_path, mesh_dir
        )

        self.target_joint_names = [
            f"{mode}_shoulder_pitch", f"{mode}_shoulder_roll",
            f"{mode}_elbow_yaw", f"{mode}_elbow_pitch",
            f"{mode}_wrist_yaw", f"{mode}_wrist_pitch", f"{mode}_wrist_roll"
        ]

        joints_to_lock_ids = [jid for jid, joint in enumerate(full_model.joints)
                              if full_model.names[jid] not in self.target_joint_names and full_model.names[jid] != "universe"]

        q0 = pin.neutral(full_model)
        geom_models = [full_visual_model, full_collision_model]
        self.model, [self.visual_model, self.collision_model] = pin.buildReducedModel(
            full_model, geom_models, joints_to_lock_ids, q0
        )
        self.data = self.model.createData()

        self.geom_model = pin.buildGeomFromUrdf(
            self.model, urdf_model_path, pin.GeometryType.COLLISION, mesh_dir
        )
        self.geom_model.addAllCollisionPairs()
        pin.removeCollisionPairs(self.model, self.geom_model, srdf_model_path)
        self.geom_data = pin.GeometryData(self.geom_model)

        # frame selection
        self.frame_name = f"{mode}_wrist_roll_link"
        self.frame_id = self.model.getFrameId(self.frame_name)
        self.joint_id = self.model.frames[self.frame_id].parentJoint

        pin.loadReferenceConfigurations(self.model, srdf_model_path)
        self.q = self.model.referenceConfigurations[f"{mode}_home"]
        self.q_home = copy.copy(self.q)

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacement(self.model, self.data, self.frame_id)
        self.oMdes_smoothed = self.data.oMf[self.frame_id].copy()

        if self.enable_meshcat:
            self.viz = MeshcatVisualizer(
                self.model, self.collision_model, self.visual_model)
            self.viz.initViewer(open=True)
            self.viz.loadViewerModel()
            self.viz.display(self.q)

    def setup_gui(self):
        oMdes = self.data.oMf[self.frame_id].copy()
        initial_translation = oMdes.translation
        initial_rpy = pin.rpy.matrixToRpy(oMdes.rotation)
        initial_values = list(initial_translation) + list(initial_rpy)

        names = ["x", "y", "z", "roll", "pitch", "yaw"]
        max_vals = [1.0, 1.0, 1.0, np.pi, np.pi, np.pi]

        self.app = QApplication(sys.argv)
        self.window = PoseSliderWidget(initial_values, max_vals, names)
        self.window.show()

    def setup_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot)
        self.timer.start(20)

    def interpolate_se3(self, a: pin.SE3, b: pin.SE3, alpha: float) -> pin.SE3:
        quat_a = pin.Quaternion(a.rotation)
        quat_b = pin.Quaternion(b.rotation)
        quat_interp = quat_a.slerp(alpha, quat_b)
        Rot = quat_interp.matrix()
        t = (1 - alpha) * a.translation + alpha * b.translation
        return pin.SE3(Rot, t)

    def update_robot(self):
        values = self.window.get_pose_values()
        translation = np.array(values[:3])
        rpy = values[3:6]
        rotation = pin.rpy.rpyToMatrix(*rpy)
        oMdes = pin.SE3(rotation, translation)
        desired_quat = R.from_euler('xyz', rpy).as_quat()

        if not is_pose_reachable(translation):
            print("❌ Target is outside the reachable workspace")
            return

        # low pass filter for the input pose
        self.oMdes_smoothed = self.interpolate_se3(
            self.oMdes_smoothed, oMdes, 0.6)

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacement(self.model, self.data, self.frame_id)
        pin.computeCollisions(self.model, self.data,
                              self.geom_model, self.geom_data, self.q, False)

        if any(res.isCollision() for res in self.geom_data.collisionResults):
            print("❌ Collision detected. Step rejected.")
            return

        iMd = self.data.oMf[self.frame_id].actInv(self.oMdes_smoothed)
        err = pin.log(iMd).vector

        T = self.data.oMf[self.frame_id]
        rotation_matrix = T.rotation
        # Convert rotation matrix to RPY (in degrees)
        rpy_current = R.from_matrix(
            rotation_matrix).as_euler('xyz', degrees=False)
        current_quat = R.from_euler('xyz', rpy_current).as_quat()
        # print("📌 Pinocchio EE pose:")
        # print("  Translation:", T.translation)
        # # print(
        # #     f"📌 End-effector RPY (deg): roll={rpy[0]:.2f}, pitch={rpy[1]:.2f}, yaw={rpy[2]:.2f}")
        # print(
        #     f"Desired quat: x={desired_quat[0]:.2f}, y={desired_quat[1]:.2f}, z={desired_quat[2]:.2f}, w={desired_quat[3]:.2f}")
        # print(
        #     f"current quat: x={current_quat[0]:.2f}, y={current_quat[1]:.2f}, z={current_quat[2]:.2f}, w={current_quat[3]:.2f}")

        if np.linalg.norm(err) < 1e-3:
            print("arrved at the target position!")
            return

        J = pin.computeJointJacobian(
            self.model, self.data, self.q, self.joint_id)
        J = -pin.Jlog6(iMd.inverse()).dot(J)

        lambda_damp = 1e-6
        # J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_damp * np.eye(6))
        M = pin.crba(self.model, self.data, self.q)
        M = 0.5 * (M + M.T)
        Minv = np.linalg.inv(M)
        JMJt = J @ Minv @ J.T
        J_sharp = Minv @ J.T @ np.linalg.inv(JMJt +
                                             lambda_damp * np.eye(J.shape[0]))

        v_task = -J_sharp @ err

        k_null = 0.1
        # dq_null_desired = np.zeros(self.model.nv)
        # dq_null_desired[1] = k_null * (self.q_home[1] - self.q[1])
        # dq_null_desired[2] = k_null * (self.q_home[2] - self.q[2])
        dq_null_desired = k_null * (self.q_home - self.q)
        P_null = np.eye(self.model.nv) - J_sharp @ J
        dq_null = P_null @ dq_null_desired

        print(f"‖v_task‖: {np.linalg.norm(v_task):.4f}")
        print(f"dq_null value is : {dq_null}")

        # v = v_task
        v = v_task + dq_null
        self.q[:] = pin.integrate(self.model, self.q, v * 0.02)

        if self.enable_meshcat:
            self.viz.display(self.q)

    def run(self):
        sys.exit(self.app.exec_())


if __name__ == "__main__":
    controller = PoseIKController(enable_meshcat=True)
    controller.run()
