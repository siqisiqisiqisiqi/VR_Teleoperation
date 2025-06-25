# 🦾 ROS to LeRobot Dataset Converter

This repository provides a full pipeline to collect robot data from a ROS 2 system and convert it into the [LeRobot](https://github.com/huggingface/lerobot) dataset format, suitable for training vision-language-action models like SmolVLA or Pi-0.

---

## 📁 Folder Structure

```
├── data_collection/
        └── data_collection.py                        # Collect data using ROS 2 and save to .pkl files
        └──  convert_dataset.py                      # Convert .pkl files to LeRobot-compatible Parquet and video files
        └── generate_episodes_json.py           # Create episodes.jsonl and tasks.jsonl metadata files
        └── generate_episodes_stats_json.py  # Compute per-episode statistics (mean, std, etc.)
        └── generate_info_json.py                   # Generate dataset-level info.json
├── my_dataset/                    # Raw data (output from data_collection.py)
├── lerobot_dataset/               # Final formatted dataset (output of conversion)
```

---

## 📦 Step-by-Step Instructions

### 1. Collect Data from ROS 2

Run the data collector node to save episodes in `.pkl` format.

```bash
python data_collection.py
```

- Input topics:
  - `/tf_right_wrist` — end-effector TF pose
  - `/joint_states_isaac` — hand joint states
  - `/hand_pose_ik` — IK-based hand pose
  - `/joint_command` — gripper command
  - `/rgb_left`, `/rgb_right` — camera images
  - `/recording_trigger` — control recording ("wait" or "stop")

- Output: Pickled episodes in `./my_dataset/episode_###.pkl`

---

### 2. Convert to LeRobot Format

Run the conversion script:

```bash
python convert_dataset.py
```

This will:
- Read `.pkl` files from `./my_dataset`
- Resize and encode images into MP4 videos
- Convert state/action data into Parquet tables
- Save them under `./lerobot_dataset/data/` and `./lerobot_dataset/videos/`

---

### 3. Generate Metadata

#### a) Generate `episodes.jsonl` and `tasks.jsonl`:

```bash
python generate_episodes_json.py
```

#### b) Generate `episodes_stats.jsonl`:

```bash
python generate_episodes_stats_json.py
```

#### c) Generate `info.json` (dataset description):

```bash
python generate_info_json.py
```

---

## 📊 Data Format

- `observation.state`: `[x, y, z, qx, qy, qz, qw, gripper]`
- `action`: same format as state
- `images.cam1` and `images.cam2`: RGB videos (360x640)
- Frame rate: 5 Hz

---

## ✅ Output Example

```
lerobot_dataset/
├── data/
│   └── chunk-000/
│       └── episode_000000.parquet
├── videos/
│   └── chunk-000/
│       ├── observation.images.cam1/episode_000000.mp4
│       └── observation.images.cam2/episode_000000.mp4
└── meta/
    ├── episodes.jsonl
    ├── tasks.jsonl
    ├── episodes_stats.jsonl
    └── info.json
```

---

## 🧠 Notes

- The collected data is suitable for LeRobot’s `SmolVLA` and `DiffusionPolicy` training pipelines.
- You can add new cameras or state dimensions by modifying the relevant `data_collection.py` logic and updating the metadata generator scripts accordingly.