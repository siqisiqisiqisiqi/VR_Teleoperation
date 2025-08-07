from __future__ import annotations
import os
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import pyarrow as pa
import pyarrow.parquet as pq


# -----------------------------
# Config
# -----------------------------
@dataclass
class InfoConfig:
    dataset_root: Path = Path("./lerobot_dataset")
    data_glob: str = "data/chunk-*/episode_*.parquet"   # supports multiple chunks
    meta_dir: Path = Path("./lerobot_dataset/meta")
    camera_keys: List[str] = field(default_factory=lambda: ["cam1", "cam2", "cam3"])
    fps: int = 10
    chunk_size: int = 1000
    codebase_version: str = "v2.1"
    robot_type: str = "custom_humanoid"
    overwrite: bool = True

    # If you know the shapes, set them. If None, we’ll try to infer.
    obs_state_shape: Optional[int] = None
    action_shape: Optional[int] = None


# -----------------------------
# Generator
# -----------------------------
class DatasetInfoGenerator:
    def __init__(self, cfg: InfoConfig):
        self.cfg = cfg
        self.dataset_root = cfg.dataset_root
        self.meta_dir = cfg.meta_dir
        self.meta_dir.mkdir(parents=True, exist_ok=True)

    # --------- helpers ---------
    def _iter_parquets(self) -> List[Path]:
        paths = sorted(self.dataset_root.glob(self.cfg.data_glob))
        if not paths:
            raise FileNotFoundError(
                f"No Parquet files found with pattern: {self.dataset_root / self.cfg.data_glob}"
            )
        return paths

    def _count_frames(self, parquet_files: List[Path]) -> Tuple[int, int]:
        total_frames = 0
        total_eps = len(parquet_files)
        for p in parquet_files:
            total_frames += pq.read_table(p).num_rows
        return total_eps, total_frames

    def _infer_vector_len(self, parquet_path: Path, column: str) -> Optional[int]:
        """Try to infer vector length from the first row of a list-like column."""
        try:
            # Read only needed column (fast)
            table: pa.Table = pq.read_table(parquet_path, columns=[column])
            if table.num_rows == 0:
                return None
            first = table[column][0].as_py()
            # first could be list[float] or numpy-ish; just try len()
            return len(first) if hasattr(first, "__len__") else None
        except Exception:
            return None

    def _episode_chunk_template(self) -> str:
        # Detect whether multiple chunks exist; keep template stable either way
        return "chunk-{episode_chunk:03d}"

    def _build_features(self, obs_dim: int, action_dim: int) -> Dict:
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": [obs_dim],
                "names": {
                    # You can adapt names to your actual layout
                    "pose": ["x", "y", "z", "r11", "r21", "r31", "r12", "r22", "r32", "gripper"][:obs_dim]
                }
            },
            "action": {
                "dtype": "float32",
                "shape": [action_dim],
                "names": {
                    "pose": ["x", "y", "z", "r11", "r21", "r31", "r12", "r22", "r32", "gripper"][:action_dim]
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

        # Add camera/video features
        for cam in self.cfg.camera_keys:
            features[f"observation.images.{cam}"] = {
                "dtype": "video",
                "shape": [360, 640, 3],
                "names": ["height", "width", "channel"],
                "video_info": {
                    "video.fps": self.cfg.fps,
                    "video.codec": "mp4v",
                    "video.pix_fmt": "yuv420p",
                    "video.is_depth_map": False,
                    "has_audio": False
                }
            }
        return features

    # --------- main build ---------
    def build(self) -> Dict:
        parquet_files = self._iter_parquets()
        total_episodes, total_frames = self._count_frames(parquet_files)

        # Infer dims if not provided
        example = parquet_files[0]
        obs_dim = self.cfg.obs_state_shape or self._infer_vector_len(example, "observation.state") or 10
        act_dim = self.cfg.action_shape or self._infer_vector_len(example, "action") or 10

        info = {
            "codebase_version": self.cfg.codebase_version,
            "robot_type": self.cfg.robot_type,
            "total_episodes": total_episodes,
            "total_frames": total_frames,
            "total_tasks": 1,
            "total_videos": total_episodes,
            "total_chunks": len({p.parent.name for p in parquet_files}),  # number of chunk-* dirs
            "chunks_size": self.cfg.chunk_size,
            "fps": self.cfg.fps,
            "splits": {
                # simple default split: everything is train
                "train": f"0:{total_episodes}"
            },
            "data_path": f"data/{self._episode_chunk_template()}/episode_{'{episode_index:06d}'}" + ".parquet",
            "video_path": f"videos/{self._episode_chunk_template()}/" + "{video_key}/episode_{episode_index:06d}.mp4",
            "features": self._build_features(obs_dim, act_dim),
        }
        return info

    def save(self, info: Dict) -> Path:
        out_path = self.meta_dir / "info.json"
        if out_path.exists() and not self.cfg.overwrite:
            raise FileExistsError(f"{out_path} exists and overwrite=False")
        with out_path.open("w") as f:
            json.dump(info, f, indent=2)
        return out_path

    def run(self) -> Path:
        info = self.build()
        path = self.save(info)
        print(f"✅ info.json written to: {path}")
        print(f"   total_episodes={info['total_episodes']}  total_frames={info['total_frames']}")
        return path


# -----------------------------
# Entry point
# -----------------------------
def main():
    cfg = InfoConfig(
        dataset_root=Path("./lerobot_dataset"),
        meta_dir=Path("./lerobot_dataset/meta"),
        camera_keys=["cam1", "cam2", "cam3"],
        fps=10,
        chunk_size=1000,
        codebase_version="v2.1",
        robot_type="custom_humanoid",
        overwrite=True,
        # Set to None to auto-infer from first parquet, or provide known dims:
        obs_state_shape=None,
        action_shape=None,
    )
    DatasetInfoGenerator(cfg).run()


if __name__ == "__main__":
    main()