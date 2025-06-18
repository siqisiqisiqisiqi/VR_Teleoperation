# convert_to_lerobot_format.py
import os
import pickle
import cv2
import json
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from tqdm import tqdm

# Config
INPUT_DIR = "./my_dataset"
OUTPUT_DIR = "./lerobot_dataset"
FPS = 5
CAMERA_KEYS = ["cam1", "cam2"]
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

# Helper to write video


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

    for t, step in enumerate(episode):
        obs = step['observation']
        act = step['action']

        for cam in CAMERA_KEYS:
            img = obs['images'][cam]
            resiz_img = cv2.resize(
                img, (640, 360), interpolation=cv2.INTER_AREA)
            video_buffers[cam].append(resiz_img)

        state = np.array(obs['state'], dtype=np.float32)
        action = np.array(act, dtype=np.float32)

        data["observation.state"].append(state)
        data["action"].append(action)
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
