import os
import json
import pyarrow.parquet as pq

DATA_DIR = "./lerobot_dataset/data/chunk-000"
META_DIR = "./lerobot_dataset/meta"
CAMERA_KEYS = ["cam1", "cam2"]
FPS = 5
CHUNK_SIZE = 1000
CODEBASE_VERSION = "v2.1"

# Scan dataset
parquet_files = sorted([f for f in os.listdir(DATA_DIR) if f.endswith(".parquet")])
total_episodes = len(parquet_files)
total_frames = 0
for file in parquet_files:
    table = pq.read_table(os.path.join(DATA_DIR, file))
    total_frames += table.num_rows

# Build info
info = {
    "codebase_version": CODEBASE_VERSION,
    "robot_type": "custom_humanoid",
    "total_episodes": total_episodes,
    "total_frames": total_frames,
    "total_tasks": 1,
    "total_videos": total_episodes,
    "total_chunks": 1,
    "chunks_size": CHUNK_SIZE,
    "fps": FPS,
    "splits": {
        "train": f"0:{total_episodes}"
    },
    "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
    "features": {
        "observation.state": {
            "dtype": "float32",
            "shape": [8],
            "names": {
                "pose": ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]
            }
        },
        "action": {
            "dtype": "float32",
            "shape": [8],
            "names": {
                "pose": ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]
            }
        },
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "next.reward": {"dtype": "float32", "shape": [1], "names": None},
        "next.done": {"dtype": "bool", "shape": [1], "names": None},
        "next.success": {"dtype": "bool", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
    }
}

# Add camera features
for cam in CAMERA_KEYS:
    info["features"][f"observation.images.{cam}"] = {
        "dtype": "video",
        "shape": [360, 640, 3],
        "names": ["height", "width", "channel"],
        "video_info": {
            "video.fps": FPS,
            "video.codec": "mp4v",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False
        }
    }

# Save
with open(os.path.join(META_DIR, "info.json"), 'w') as f:
    json.dump(info, f, indent=2)

print("✅ info.json generated.")
