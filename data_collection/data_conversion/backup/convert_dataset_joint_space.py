import os
import cv2
import pickle
import sys
from pathlib import Path

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(PARENT_DIR)

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from tqdm import tqdm

from src.tasktojointIKsolver import TaskToJointIKSolver


def write_video(frames, path, fps):
    height, width, _ = frames[0].shape
    writer = cv2.VideoWriter(
        path,
        cv2.VideoWriter_fourcc(*'mp4v'),
        fps,
        (width, height)
    )
    for f in frames:
        writer.write(f)
    writer.release()


# Config
INPUT_DIR = "./my_dataset"
OUTPUT_DIR = "./lerobot_dataset"
FPS = 10
CAMERA_KEYS = ["cam1", "cam2", "cam3"]
CHUNK_NAME = "chunk-000"

# Create output directories
DATA_DIR = os.path.join(OUTPUT_DIR, "data", CHUNK_NAME)
VIDEO_DIRS = {
    cam: os.path.join(OUTPUT_DIR, "videos", CHUNK_NAME,
                      f"observation.images.{cam}")
    for cam in CAMERA_KEYS
}
META_DIR = os.path.join(OUTPUT_DIR, "meta")
os.makedirs(DATA_DIR, exist_ok=True)
for d in VIDEO_DIRS.values():
    os.makedirs(d, exist_ok=True)
os.makedirs(META_DIR, exist_ok=True)

# initialize the tasktojoint converter
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

# Convert each episode
index = 0
pkl_files = sorted([f for f in os.listdir(INPUT_DIR) if f.endswith(".pkl")])
for i, filename in tqdm(enumerate(pkl_files), total=len(pkl_files)):
    with open(os.path.join(INPUT_DIR, filename), 'rb') as f:
        episode = pickle.load(f)

    data = {
        "observation.state": [],
        "action": [],
        "episode_index": [],
        "frame_index": [],
        "timestamp": [],
        "next.reward": [],
        "next.done": [],
        "next.success": [],
        "index": [],
        "task_index": []
    }
    video_buffers = {cam: [] for cam in CAMERA_KEYS}

    q_state = None
    q_action = None

    for t, step in enumerate(episode):
        obs = step['observation']
        act = step['action']

        for cam in CAMERA_KEYS:
            img = obs['images'][cam]
            resiz_img = cv2.resize(
                img, (640, 360), interpolation=cv2.INTER_AREA)
            video_buffers[cam].append(resiz_img)

        # convert the task space to joint space
        pos = np.array(obs['state'][0:3])
        pos[2] = pos[2] - 1.012
        quat = np.array(obs['state'][3:7])  # x, y, z, w
        state = solver.solve(pos, quat, q_state)

        pos = np.array(act[0:3])
        quat = np.array(act[3:7])  # x, y, z, w
        action = solver.solve(pos, quat, q_action)

        q_state = state
        q_action = action

        state_value = np.append(state, obs['state'][-1])
        action_value = np.append(action, act[-1])

        state_value = np.array(state_value, dtype=np.float32)
        action_value = np.array(action_value, dtype=np.float32)

        data["observation.state"].append(state_value)
        data["action"].append(action_value)
        data["episode_index"].append(i)
        data["frame_index"].append(t)
        data["timestamp"].append(t / FPS)
        data["next.reward"].append(0.0)
        data["next.done"].append(t == len(episode) - 1)
        data["next.success"].append(False)
        data["index"].append(index)
        data["task_index"].append(0)
        index = index + 1

    # Save parquet
    table = pa.Table.from_pydict(data)
    pq_path = os.path.join(DATA_DIR, f"episode_{i:06d}.parquet")
    pq.write_table(table, pq_path)

    # Save videos
    for cam in CAMERA_KEYS:
        video_path = os.path.join(VIDEO_DIRS[cam], f"episode_{i:06d}.mp4")
        write_video(video_buffers[cam], video_path, FPS)
    # break
