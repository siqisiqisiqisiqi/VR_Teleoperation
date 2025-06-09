import os
import json
import pyarrow.parquet as pq

DATA_DIR = "./lerobot_dataset/data/chunk-000"
META_DIR = "./lerobot_dataset/meta"

TASK0 = "Move the cylinder onto the plate with corresponding color."
# --- 1. Generate episodes.jsonl ---
episodes_jsonl_path = os.path.join(META_DIR, "episodes.jsonl")

with open(episodes_jsonl_path, 'w') as f_out:
    for file in sorted(os.listdir(DATA_DIR)):
        if not file.endswith(".parquet"):
            continue
        episode_index = int(file.split("_")[1].split(".")[0])
        parquet_path = os.path.join(DATA_DIR, file)
        table = pq.read_table(parquet_path)
        length = table.num_rows
        meta = {
            "episode_index": episode_index,
            "tasks": TASK0,
            "length": length,
        }
        f_out.write(json.dumps(meta) + "\n")

print(f"✅ Wrote {episodes_jsonl_path}")

# --- 2. Generate tasks.jsonl ---
tasks_jsonl_path = os.path.join(META_DIR, "tasks.jsonl")
with open(tasks_jsonl_path, 'w') as f:
    json.dump({"task_index": 0, "task": TASK0}, f)
    f.write("\n")

print(f"✅ Wrote {tasks_jsonl_path}")
