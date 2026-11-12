#!/usr/bin/env python3
"""
从 LeRobot 数据集中随机抽取指定数量的 episodes 创建子集。

用法:
    python random_sample_episodes.py \
        --input_dir /path/to/source_dataset \
        --output_dir /path/to/output_dataset \
        --num_episodes 10 \
        --seed 42
"""

import argparse
import copy
import json
import random
import shutil
from pathlib import Path

import pandas as pd


def load_info(dataset_dir: Path) -> dict:
    """加载数据集的 info.json"""
    info_path = dataset_dir / "meta" / "info.json"
    if not info_path.exists():
        raise FileNotFoundError(f"找不到 info.json: {info_path}")
    with open(info_path, "r") as f:
        return json.load(f)


def load_jsonl(path: Path) -> list[dict]:
    """加载 JSONL 文件"""
    if not path.exists():
        return []
    records = []
    with open(path, "r") as f:
        for line in f:
            if line.strip():
                records.append(json.loads(line))
    return records


def save_jsonl(records: list[dict], path: Path):
    """保存 JSONL 文件"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for record in records:
            f.write(json.dumps(record) + "\n")


def get_chunk_and_local_index(episode_idx: int, chunks_size: int) -> tuple[str, int]:
    """根据 episode_idx 计算所在的 chunk 目录和 chunk 内的本地索引"""
    chunk_num = episode_idx // chunks_size
    local_idx = episode_idx % chunks_size
    return f"chunk-{chunk_num:03d}", local_idx


def find_parquet(dataset_dir: Path, episode_idx: int, chunks_size: int) -> Path | None:
    """查找 episode 对应的 parquet 文件"""
    chunk_dir, _ = get_chunk_and_local_index(episode_idx, chunks_size)
    path = dataset_dir / "data" / chunk_dir / f"episode_{episode_idx:06d}.parquet"
    if path.exists():
        return path
    return None


def find_video(dataset_dir: Path, episode_idx: int, video_key: str, chunks_size: int) -> Path | None:
    """查找 episode 对应的视频文件"""
    chunk_dir, _ = get_chunk_and_local_index(episode_idx, chunks_size)
    path = dataset_dir / "videos" / chunk_dir / video_key / f"episode_{episode_idx:06d}.mp4"
    if path.exists():
        return path
    return None


def random_sample_dataset(
    input_dir: Path,
    output_dir: Path,
    num_episodes: int,
    seed: int | None = None
):
    """从数据集中随机抽取指定数量的 episodes

    Args:
        input_dir: 输入数据集目录
        output_dir: 输出目录
        num_episodes: 要抽取的 episode 数量
        seed: 随机种子（可选）
    """

    # 设置随机种子
    if seed is not None:
        random.seed(seed)
        print(f"使用随机种子: {seed}")

    # 1. 加载数据集信息
    print(f"\n[1/4] 加载数据集: {input_dir}")
    info = load_info(input_dir)
    total_episodes = info["total_episodes"]
    chunks_size = info.get("chunks_size", 1000)

    if num_episodes > total_episodes:
        raise ValueError(
            f"请求的 episodes 数量 ({num_episodes}) 超过数据集总数 ({total_episodes})"
        )

    print(f"  总 episodes: {total_episodes}")
    print(f"  抽取数量: {num_episodes}")

    # 2. 随机选择 episode 索引
    print("\n[2/4] 随机选择 episodes...")
    all_indices = list(range(total_episodes))
    selected_indices = sorted(random.sample(all_indices, num_episodes))
    print(f"  选中的 episode 索引: {selected_indices}")

    # 3. 创建输出目录
    print(f"\n[3/4] 创建输出目录: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (output_dir / "meta").mkdir(parents=True, exist_ok=True)
    (output_dir / "videos" / "chunk-000").mkdir(parents=True, exist_ok=True)

    # 4. 复制选中的 episodes
    print("\n[4/4] 复制数据...")

    episodes = load_jsonl(input_dir / "meta" / "episodes.jsonl")
    episodes_stats = load_jsonl(input_dir / "meta" / "episodes_stats.jsonl")
    tasks = load_jsonl(input_dir / "meta" / "tasks.jsonl")

    new_episodes = []
    new_episodes_stats = []
    total_frames = 0
    total_videos = 0

    # task 映射
    task_text_to_idx: dict[str, int] = {}
    new_tasks = []
    local_task_map = {t["task_index"]: t["task"] for t in tasks}

    for new_idx, old_idx in enumerate(selected_indices):
        print(f"  处理 episode {old_idx} -> {new_idx}")

        # 复制 parquet
        src_parquet = find_parquet(input_dir, old_idx, chunks_size)
        if src_parquet is None:
            print(f"    ⚠ 找不到 parquet: episode_{old_idx:06d}")
            continue

        dst_parquet = output_dir / "data" / "chunk-000" / f"episode_{new_idx:06d}.parquet"
        df = pd.read_parquet(src_parquet)

        # 更新 episode_index 和 index
        df["episode_index"] = new_idx
        df["index"] = df["frame_index"] + total_frames

        # 更新 task_index
        if "task_index" in df.columns:
            old_task_indices = df["task_index"].unique()
            for old_ti in old_task_indices:
                task_text = local_task_map.get(int(old_ti), f"unknown_task_{old_ti}")
                if task_text not in task_text_to_idx:
                    new_ti = len(task_text_to_idx)
                    task_text_to_idx[task_text] = new_ti
                    new_tasks.append({"task_index": new_ti, "task": task_text})
            df["task_index"] = df["task_index"].map(
                lambda x: task_text_to_idx.get(local_task_map.get(int(x), f"unknown_task_{x}"), 0)
            )

        df.to_parquet(dst_parquet)
        ep_frames = len(df)

        # 复制视频
        video_keys = [k for k in info["features"].keys() if info["features"][k].get("dtype") == "video"]
        for video_key in video_keys:
            src_video = find_video(input_dir, old_idx, video_key, chunks_size)
            if src_video is not None:
                dst_video_dir = output_dir / "videos" / "chunk-000" / video_key
                dst_video_dir.mkdir(parents=True, exist_ok=True)
                dst_video = dst_video_dir / f"episode_{new_idx:06d}.mp4"
                shutil.copy2(src_video, dst_video)
                total_videos += 1

        # 更新元数据
        if old_idx < len(episodes):
            ep_meta = copy.deepcopy(episodes[old_idx])
            ep_meta["episode_index"] = new_idx
            new_episodes.append(ep_meta)

        if old_idx < len(episodes_stats):
            ep_stats = copy.deepcopy(episodes_stats[old_idx])
            ep_stats["episode_index"] = new_idx
            # 更新 stats 内部的 episode_index 和 index
            if "stats" in ep_stats:
                stats = ep_stats["stats"]
                if "episode_index" in stats:
                    stats["episode_index"] = {
                        "min": [new_idx],
                        "max": [new_idx],
                        "mean": [float(new_idx)],
                        "std": [0.0],
                        "count": stats["episode_index"].get("count", [ep_frames]),
                    }
                if "index" in stats:
                    old_index = stats["index"]
                    count = old_index.get("count", [ep_frames])
                    n = count[0] if isinstance(count, list) else count
                    new_min = total_frames
                    new_max = total_frames + n - 1
                    new_mean = (new_min + new_max) / 2.0
                    stats["index"] = {
                        "min": [new_min],
                        "max": [new_max],
                        "mean": [new_mean],
                        "std": old_index.get("std", [0.0]),
                        "count": count,
                    }
            new_episodes_stats.append(ep_stats)

        total_frames += ep_frames

    # 保存元数据
    save_jsonl(new_episodes, output_dir / "meta" / "episodes.jsonl")
    save_jsonl(new_episodes_stats, output_dir / "meta" / "episodes_stats.jsonl")
    save_jsonl(new_tasks, output_dir / "meta" / "tasks.jsonl")

    # 创建 info.json
    new_info = copy.deepcopy(info)
    new_info["total_episodes"] = len(selected_indices)
    new_info["total_frames"] = total_frames
    new_info["total_videos"] = total_videos
    new_info["total_tasks"] = len(new_tasks)
    new_info["total_chunks"] = 1
    new_info["splits"] = {"train": f"0:{len(selected_indices)}"}

    with open(output_dir / "meta" / "info.json", "w") as f:
        json.dump(new_info, f, indent=4)

    print(f"\n{'='*60}")
    print(f"✓ 随机抽取完成!")
    print(f"  输出目录: {output_dir}")
    print(f"  抽取 episodes: {len(selected_indices)}")
    print(f"  总 frames: {total_frames}")
    print(f"  总 videos: {total_videos}")
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(
        description="从 LeRobot 数据集中随机抽取指定数量的 episodes",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "--input_dir", "-i",
        type=Path,
        required=True,
        help="输入数据集目录"
    )
    parser.add_argument(
        "--output_dir", "-o",
        type=Path,
        required=True,
        help="输出目录"
    )
    parser.add_argument(
        "--num_episodes", "-n",
        type=int,
        required=True,
        help="要抽取的 episode 数量"
    )
    parser.add_argument(
        "--seed", "-s",
        type=int,
        default=None,
        help="随机种子（可选，用于可重复性）"
    )

    args = parser.parse_args()

    # 验证输入目录存在
    if not args.input_dir.exists():
        raise FileNotFoundError(f"输入目录不存在: {args.input_dir}")

    # 检查输出目录
    if args.output_dir.exists():
        response = input(f"输出目录 {args.output_dir} 已存在，是否覆盖? [y/N]: ")
        if response.lower() != "y":
            print("已取消")
            return
        shutil.rmtree(args.output_dir)

    random_sample_dataset(
        args.input_dir,
        args.output_dir,
        args.num_episodes,
        args.seed
    )


if __name__ == "__main__":
    main()
