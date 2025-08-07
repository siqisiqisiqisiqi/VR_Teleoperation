import pinocchio as pin
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R


target_joint_names = [
            f"right_shoulder_pitch", f"right_shoulder_roll",
            f"right_elbow_yaw", f"right_elbow_pitch",
            f"right_wrist_yaw", f"right_wrist_pitch", f"right_wrist_roll"
        ]

model_path = Path(__file__).parent.parent / "models"
urdf_path = model_path / "t170a_description/urdf/T170A.urdf"
srdf_path = model_path / "t170a_description/srdf/T170A.srdf"

full_model = pin.buildModelFromUrdf(str(urdf_path))
q_full = pin.neutral(full_model)

joints_to_lock_ids = [jid for jid, joint in enumerate(full_model.joints)
                              if full_model.names[jid] not in target_joint_names and full_model.names[jid] != "universe"]

reduced_model = pin.buildReducedModel(full_model, joints_to_lock_ids, q_full)
data = reduced_model.createData()

# === End-effector frame (confirm exact name from your URDF) ===
# <-- Modify this if your frame name is different
ee_frame_name = "right_wrist_roll_link"
frame_id = reduced_model.getFrameId(ee_frame_name)
joint_id = reduced_model.frames[frame_id].parentJoint
# === IK solver ===


def solve_ik_from_pose(model, data, frame_id, T_target, q0=None, max_iter=100, tol=1e-4):
    if q0 is None:
        q = np.array([0.14, -0.41, 0.01, 1.74, 0.058, -0.13, 0.0])
        # q = pin.neutral(model)
    else:
        q = q0.copy()

    for i in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame_id)
        err = pin.log6(T_target.inverse() * data.oMf[frame_id]).vector
        if np.linalg.norm(err) < tol:
            return q
        J = pin.computeFrameJacobian(model, data, q, frame_id)
        dq = np.linalg.pinv(J) @ err
        q = pin.integrate(model, q, -dq)

    return None


# === Target pose (position + quaternion) ===
position = np.array([0.3136, -0.310, 1.25497- 1.012])  # in meters
quat_xyzw = [-0.01851184, 0.75299371, -0.03591061, -0.65678627]  # x,y,z,w format
rotation = R.from_quat(quat_xyzw).as_matrix()
T_target = pin.SE3(rotation, position)

# === Solve IK ===
q_solution = solve_ik_from_pose(reduced_model, data, frame_id, T_target)

# === Output result ===
if q_solution is not None:
    print("✅ IK solution found:")
    print(q_solution)
    # for i, name in enumerate(reduced_model.names):
    #     if i >= reduced_model.njoints:
    #         continue
    #     print(f"  {name}: {q_solution[i]:.4f}")
else:
    print("❌ IK did not converge.")
