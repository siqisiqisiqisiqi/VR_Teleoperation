import pinocchio as pin
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R


class TaskToJointIKSolver:
    def __init__(self,
                 urdf_path: str,
                 srdf_path: str,
                 ee_frame_name: str,
                 target_joint_names: list[str]):
        # Load full model
        full_model = pin.buildModelFromUrdf(urdf_path)
        q_full = pin.neutral(full_model)

        # Build reduced model
        joints_to_lock = [
            jid for jid, joint in enumerate(full_model.joints)
            if full_model.names[jid] not in target_joint_names and full_model.names[jid] != "universe"
        ]
        self.model = pin.buildReducedModel(full_model, joints_to_lock, q_full)
        self.data = self.model.createData()

        # Frame and joint setup
        self.frame_id = self.model.getFrameId(ee_frame_name)
        self.joint_id = self.model.frames[self.frame_id].parentJoint

    def solve(self, position: np.ndarray, quat_xyzw: np.ndarray,
              q_init: np.ndarray = None, max_iter=100, tol=1e-4) -> np.ndarray:
        """
        Solves IK for a given target position + quaternion (xyzw).
        Returns: joint angles (q) or None if failed.
        """
        R_target = R.from_quat(quat_xyzw).as_matrix()
        T_target = pin.SE3(R_target, position)

        if q_init is None:
            # Default initial guess (neutral or predefined)
            q = np.array([0.14, -0.41, 0.01, 1.74, 0.058, -0.13, 0.0])
        else:
            q = q_init.copy()

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.frame_id)
            err = pin.log6(T_target.inverse() *
                           self.data.oMf[self.frame_id]).vector
            if np.linalg.norm(err) < tol:
                return q
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.frame_id)
            dq = np.linalg.pinv(J) @ err
            q = pin.integrate(self.model, q, -dq)

        print("❌ IK failed to converge.")
        return q  # Failed to converge


if __name__ == "__main__":
    model_root = Path(__file__).parent.parent / "models" / "t170a_description"
    urdf_path = model_root / "urdf/T170A.urdf"
    srdf_path = model_root / "srdf/T170A.srdf"

    target_joint_names = [
        "right_shoulder_pitch", "right_shoulder_roll",
        "right_elbow_yaw", "right_elbow_pitch",
        "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
    ]
    ee_frame_name = "right_wrist_roll_link"

    solver = TaskToJointIKSolver(str(urdf_path), str(
        srdf_path), ee_frame_name, target_joint_names)

    # Sample target pose
    pos = np.array([0.3136, -0.310, 1.25497 - 1.012])
    quat = np.array([-0.01851184, 0.75299371, -
                    0.03591061, -0.65678627])  # x, y, z, w

    q_result = solver.solve(pos, quat)

    if q_result is not None:
        print("✅ IK Solved! Joint values:")
        print(q_result)
    else:
        print("❌ IK failed to converge.")
