import os
import json
import numpy as np
import pandas as pd
import pyarrow.parquet as pq
from tqdm import tqdm
from pathlib import Path
import cv2

DATA_PATH = "./lerobot_dataset/data/chunk-000"
VIDEO_PATHS = {
    "observation.images.cam1": "./lerobot_dataset/videos/chunk-000/observation.images.cam1",
    "observation.images.cam2": "./lerobot_dataset/videos/chunk-000/observation.images.cam2",
}
OUT_JSONL = "./lerobot_dataset/meta/episodes_stats.jsonl"


def estimate_num_samples(data_len, min_num=100, max_num=10000, power=0.75):
    return max(min_num, min(int(data_len**power), max_num))


def sample_indices(data_len):
    num_samples = estimate_num_samples(data_len)
    return np.round(np.linspace(0, data_len - 1, num_samples)).astype(int)


def downsample_image(img, target_size=150, max_threshold=300):
    h, w = img.shape[:2]
    if max(h, w) < max_threshold:
        return img
    factor = max(h, w) // target_size
    return img[::factor, ::factor]


def load_sampled_video_frames(video_path, sample_rate=1.0):
    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    indices = sample_indices(total_frames)
    frames = []

    for i in range(total_frames):
        ret, frame = cap.read()
        if not ret:
            break
        if i in indices:
            frame = downsample_image(frame)
            frame = frame.astype(np.float32) / 255.0
            frames.append(np.transpose(frame, (2, 0, 1)))  # CHW
    cap.release()
    return np.stack(frames) if frames else None


def get_stats(array, axis=0, keepdims=False, squeeze=False):
    stats = {
        "min": np.min(array, axis=axis, keepdims=keepdims),
        "max": np.max(array, axis=axis, keepdims=keepdims),
        "mean": np.mean(array, axis=axis, keepdims=keepdims),
        "std": np.std(array, axis=axis, keepdims=keepdims),
        "count": [len(array)]
    }
    if keepdims and squeeze:
        stats = {k: v if k == "count" else v.squeeze(
            0) for k, v in stats.items()}
    return {k: v if k == "count" else v.tolist() for k, v in stats.items()}


def process_episode(parquet_file):
    df = pd.read_parquet(os.path.join(DATA_PATH, parquet_file))
    episode_idx = int(df["episode_index"].iloc[0])
    stats = {}

    # Per-camera video stats
    video_name = parquet_file.replace(".parquet", ".mp4")
    for key, path in VIDEO_PATHS.items():
        frames = load_sampled_video_frames(os.path.join(path, video_name))
        if frames is not None:
            stats[key] = get_stats(frames, axis=(
                0, 2, 3), keepdims=True, squeeze=True)  # stats over CHW

    # State/action stats
    stats["observation.state"] = get_stats(np.vstack(df["observation.state"]))
    stats["action"] = get_stats(np.vstack(df["action"]))

    # Scalar column stats
    for col in ["episode_index", "frame_index", "timestamp", "next.reward",
                "next.done", "next.success", "index", "task_index"]:
        stats[col] = get_stats(df[col].to_numpy(), keepdims=1)

    return {"episode_index": episode_idx, "stats": stats}


# Main
records = []
for file in tqdm(sorted(os.listdir(DATA_PATH))):
    if file.endswith(".parquet"):
        records.append(process_episode(file))

with open(OUT_JSONL, "w") as f:
    for record in records:
        json.dump(record, f)
        f.write("\n")

print("✅ Done generating episodes_stats.jsonl")
