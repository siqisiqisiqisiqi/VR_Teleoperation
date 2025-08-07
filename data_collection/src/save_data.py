import os
import cv2
import numpy as np
import shutil

FPS = 10
CAMERA_KEYS = ["cam1", "cam2", "cam3"]


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


def save_data(episode, episode_idx, output_dir):
    data = {}
    video_buffers = {cam: [] for cam in CAMERA_KEYS}
    for t, step in enumerate(episode['observation.images']):

        for cam in CAMERA_KEYS:
            img = step[cam]
            resiz_img = cv2.resize(
                img, (640, 360), interpolation=cv2.INTER_AREA)
            video_buffers[cam].append(resiz_img)

    for key in episode:
        if key != "observation.images":
            data[key] = np.array(episode[key])

    npz_path = os.path.join(output_dir, f"episode_{episode_idx:06d}.npz")
    np.savez_compressed(npz_path, **data)

    for cam in CAMERA_KEYS:
        video_path = os.path.join(
            output_dir, f"episode_{episode_idx:06d}_{cam}.mp4")
        write_video(video_buffers[cam], video_path, FPS)


def copy_existing_videos(src_folder, dst_root, episode_idx, camera_keys):
    for cam in camera_keys:
        src_name = f"episode_{episode_idx:06d}_{cam}.mp4"
        src_path = os.path.join(src_folder, src_name)

        dst_path = os.path.join(dst_root[cam], f"episode_{episode_idx:06d}.mp4")

        if os.path.exists(src_path):
            shutil.copy(src_path, dst_path)
        else:
            print(f"[WARNING] Missing: {src_path}")
