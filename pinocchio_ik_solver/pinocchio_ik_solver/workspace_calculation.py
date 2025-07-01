import numpy as np
import pinocchio as pin
from scipy.spatial import ConvexHull, Delaunay
import matplotlib.pyplot as plt
import pathlib
import pickle
import time


# === Load your robot ===
MODEL_DIR = pathlib.Path(__file__).parent.parent / "models"
URDF_PATH = MODEL_DIR / "t170a_description/urdf/T170A.urdf"
SRDF_PATH = MODEL_DIR / "t170a_description/srdf/T170A.srdf"

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch", "right_shoulder_roll",
    "right_elbow_yaw", "right_elbow_pitch",
    "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
]

EE_FRAME_NAME = "right_wrist_roll_link"
N_SAMPLES = 10000  # Number of samples

# === Load Full Model ===
full_model, _, _ = pin.buildModelsFromUrdf(URDF_PATH, MODEL_DIR)
pin.loadReferenceConfigurations(full_model, SRDF_PATH)
q0 = pin.neutral(full_model)

# === Build Reduced Model for Right Arm ===
joints_to_lock = [
    jid for jid, joint in enumerate(full_model.joints)
    if full_model.names[jid] not in RIGHT_ARM_JOINTS and full_model.names[jid] != "universe"
]
reduced_model = pin.buildReducedModel(full_model, joints_to_lock, q0)
reduced_data = reduced_model.createData()

geom_model = pin.buildGeomFromUrdf(
    reduced_model, URDF_PATH, pin.GeometryType.COLLISION, MODEL_DIR
)
geom_model.addAllCollisionPairs()
pin.removeCollisionPairs(reduced_model, geom_model, SRDF_PATH)
geom_data = pin.GeometryData(geom_model)

# === Frame ID for End Effector ===
frame_id = reduced_model.getFrameId(EE_FRAME_NAME)

# === Sample Workspace ===


def sample_workspace(model, data, frame_id, n_samples=10000):
    positions = []
    for _ in range(n_samples):
        q = pin.randomConfiguration(model)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame_id)

        pin.computeCollisions(model, data,
                              geom_model, geom_data, q, False)

        if any(res.isCollision() for res in geom_data.collisionResults):
            continue

        T = data.oMf[frame_id]
        positions.append(T.translation.copy())
    return np.array(positions)


def save_workspace(root, positions, hull):
    with open(f"{root}/right_arm_workspace.pkl", "wb") as f:
        pickle.dump({
            "positions": positions,
            "hull_vertices": hull.vertices  # optional
        }, f)


print("Sampling joint configurations...")
t0 = time.time()
positions = sample_workspace(reduced_model, reduced_data, frame_id, N_SAMPLES)
t1 = time.time()
print(f"Sampling complete, elapsed time {t1-t0}")

# === Compute Convex Hull ===
print("Computing convex hull...")
hull = ConvexHull(positions)
hull_delaunay = Delaunay(positions[hull.vertices])
print("Convex hull complete.")
save_workspace("./workspace", positions, hull)
# === Helper: Check if pose is reachable ===


def is_pose_reachable(position: np.ndarray) -> bool:
    return hull_delaunay.find_simplex(position) >= 0


# === Example Test ===
test_pos = np.array([0.4, -0.2, 0.5])  # Replace with your pose
if is_pose_reachable(test_pos):
    print("✅ Target is reachable")
else:
    print("❌ Target is outside the reachable workspace")

# === Optional: Visualize ===
def plot_workspace(positions):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(positions[:, 0], positions[:, 1],
               positions[:, 2], s=1, alpha=0.3)
    ax.set_title("Reachable Workspace of Right Arm")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    plt.tight_layout()
    plt.show()

plot_workspace(positions)
