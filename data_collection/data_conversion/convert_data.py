import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(PARENT_DIR)

import os
import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from tqdm import tqdm

from src.save_data import copy_existing_videos
from src.sixd_rotation import so3_to_sixd


class DatasetConverter:
    def __init__(self, input_dir="./my_dataset", output_dir="./lerobot_dataset",
                 camera_keys=["cam1", "cam2", "cam3"], chunk_name="chunk-000", fps=10):
        self.input_dir = input_dir
        self.output_dir = output_dir
        self.camera_keys = camera_keys
        self.chunk_name = chunk_name
        self.fps = fps

        self.data_dir = os.path.join(output_dir, "data", chunk_name)
        self.meta_dir = os.path.join(output_dir, "meta")
        self.video_dirs = {
            cam: os.path.join(output_dir, "videos", chunk_name,
                              f"observation.images.{cam}")
            for cam in self.camera_keys
        }

        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.meta_dir, exist_ok=True)
        for d in self.video_dirs.values():
            os.makedirs(d, exist_ok=True)

    def write_video(self, frames, path):
        height, width, _ = frames[0].shape
        writer = cv2.VideoWriter(
            path,
            cv2.VideoWriter_fourcc(*'mp4v'),
            self.fps,
            (width, height)
        )
        for f in frames:
            writer.write(f)
        writer.release()

    def convert(self):
        npz_files = []
        for root, _, files in os.walk(self.input_dir):
            for file in files:
                if file.endswith(".npz"):
                    npz_files.append(os.path.join(root, file))
        npz_files = sorted(npz_files)
        global_index = 0

        for i, file_path in tqdm(enumerate(npz_files), total=len(npz_files), desc="Converting episodes"):
            episode = np.load(file_path, allow_pickle=True)

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
            episode_len = len(episode["observation.state"])

            for t in range(episode_len):
                state = episode["observation.state"][t]
                action = episode["action.state"][t]

                quat = action[3:7]
                sixd_rot = so3_to_sixd(quat)
                new_act = [0] * 10
                new_act[0:3] = action[0:3]
                new_act[3:9] = sixd_rot
                new_act[-1] = action[-1]

                quat2 = state[3:7]
                sixd_rot2 = so3_to_sixd(quat2)
                new_sta = [0] * 10
                new_sta[0:3] = state[0:3]
                new_sta[3:9] = sixd_rot2
                new_sta[-1] = state[-1]

                data["observation.state"].append(new_sta)
                data["action"].append(new_act)
                data["episode_index"].append(i)
                data["frame_index"].append(t)
                data["timestamp"].append(t / self.fps)
                data["next.reward"].append(0.0)
                data["next.done"].append(t == episode_len - 1)
                data["next.success"].append(False)
                data["index"].append(global_index)
                data["task_index"].append(0)
                global_index += 1

            # Save parquet
            table = pa.Table.from_pydict(data)
            pq_path = os.path.join(self.data_dir, f"episode_{i:06d}.parquet")
            pq.write_table(table, pq_path)

            dir_path = os.path.dirname(file_path)
            copy_existing_videos(
                src_folder=dir_path,
                dst_root=self.video_dirs,
                episode_idx=i,
                camera_keys=self.camera_keys
            )


def main():
    converter = DatasetConverter(
        input_dir="./my_dataset",          # directory containing .npz files
        output_dir="./lerobot_dataset",    # output root
        camera_keys=["cam1", "cam2", "cam3"],  # expected camera keys
        chunk_name="chunk-000",                # subfolder name
        fps=10
    )
    converter.convert()


if __name__ == "__main__":
    main()
