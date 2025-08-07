from __future__ import annotations
import json
from pathlib import Path
from dataclasses import dataclass
import pyarrow.parquet as pq


@dataclass
class MetaConfig:
    data_dir: Path = Path("./lerobot_dataset/data/chunk-000")
    meta_dir: Path = Path("./lerobot_dataset/meta")
    task_text: str = "Place the cylinder into the box with right orientation."
    overwrite: bool = True  # overwrite episodes.jsonl and tasks.jsonl if exist


class EpisodesMetaGenerator:
    def __init__(self, cfg: MetaConfig):
        self.cfg = cfg
        self.cfg.meta_dir.mkdir(parents=True, exist_ok=True)

        self.episodes_jsonl = self.cfg.meta_dir / "episodes.jsonl"
        self.tasks_jsonl = self.cfg.meta_dir / "tasks.jsonl"

    def _iter_episode_parquets(self):
        if not self.cfg.data_dir.exists():
            raise FileNotFoundError(f"DATA_DIR not found: {self.cfg.data_dir}")

        # Only files that look like episode_000123.parquet
        for p in sorted(self.cfg.data_dir.glob("episode_*.parquet")):
            yield p

    def _episode_index_from_name(self, parquet_path: Path) -> int:
        # episode_000123.parquet -> 123
        stem = parquet_path.stem  # episode_000123
        idx = int(stem.split("_")[1])
        return idx

    def _episode_length(self, parquet_path: Path) -> int:
        table = pq.read_table(parquet_path)
        return table.num_rows

    def write_episodes_jsonl(self) -> int:
        if self.episodes_jsonl.exists() and not self.cfg.overwrite:
            raise FileExistsError(f"{self.episodes_jsonl} exists and overwrite=False")

        count = 0
        with self.episodes_jsonl.open("w") as f_out:
            for parquet_path in self._iter_episode_parquets():
                ep_idx = self._episode_index_from_name(parquet_path)
                length = self._episode_length(parquet_path)
                meta = {
                    "episode_index": ep_idx,
                    "tasks": self.cfg.task_text,
                    "length": length,
                }
                f_out.write(json.dumps(meta) + "\n")
                count += 1
        return count

    def write_tasks_jsonl(self) -> None:
        if self.tasks_jsonl.exists() and not self.cfg.overwrite:
            raise FileExistsError(f"{self.tasks_jsonl} exists and overwrite=False")
        with self.tasks_jsonl.open("w") as f:
            json.dump({"task_index": 0, "task": self.cfg.task_text}, f)
            f.write("\n")

    def run(self) -> None:
        n = self.write_episodes_jsonl()
        self.write_tasks_jsonl()
        print(f"✅ Wrote {self.episodes_jsonl}  (episodes: {n})")
        print(f"✅ Wrote {self.tasks_jsonl}")


def main():
    cfg = MetaConfig(
        data_dir=Path("./lerobot_dataset/data/chunk-000"),
        meta_dir=Path("./lerobot_dataset/meta"),
        task_text="Place the cylinder into the box with right orientation.",
        overwrite=True,
    )
    EpisodesMetaGenerator(cfg).run()


if __name__ == "__main__":
    main()
