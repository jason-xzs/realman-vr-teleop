#!/usr/bin/env python3
"""
合并多个 LeRobot 格式数据集的脚本。

用法:
    python merge_lerobot_datasets.py \
        --input_dirs /path/to/dataset_01 /path/to/dataset_02 \
        --output_dir /path/to/merged_output \
        --max_episodes 50

注意:
    - 所有输入数据集必须具有相同的 features 结构（相机、状态维度等）
    - 原始数据集不会被修改
    - episode_index 会被自动重新编号
    - task 按文本内容去重，task_index 会被正确重映射
    - episodes_stats 内部的 episode_index/index 统计量会同步更新
"""

import argparse
import copy
import json
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
    """查找 episode 对应的 parquet 文件，支持多 chunk"""
    chunk_dir, _ = get_chunk_and_local_index(episode_idx, chunks_size)
    path = dataset_dir / "data" / chunk_dir / f"episode_{episode_idx:06d}.parquet"
    if path.exists():
        return path
    return None


def find_video(dataset_dir: Path, episode_idx: int, video_key: str, chunks_size: int) -> Path | None:
    """查找 episode 对应的视频文件，支持多 chunk"""
    chunk_dir, _ = get_chunk_and_local_index(episode_idx, chunks_size)
    path = dataset_dir / "videos" / chunk_dir / video_key / f"episode_{episode_idx:06d}.mp4"
    if path.exists():
        return path
    return None


def update_episode_stats(ep_stats: dict, new_ep_idx: int, frame_offset: int, ep_frames: int) -> dict:
    """更新 episodes_stats 内部的 episode_index 和 index 统计量"""
    result = copy.deepcopy(ep_stats)
    result["episode_index"] = new_ep_idx

    stats = result.get("stats", {})

    # 更新 stats 内部的 episode_index 统计量
    if "episode_index" in stats:
        stats["episode_index"] = {
            "min": [new_ep_idx],
            "max": [new_ep_idx],
            "mean": [float(new_ep_idx)],
            "std": [0.0],
            "count": stats["episode_index"].get("count", [ep_frames]),
        }

    # 更新 stats 内部的 index 统计量
    if "index" in stats:
        old_index = stats["index"]
        count = old_index.get("count", [ep_frames])
        n = count[0] if isinstance(count, list) else count
        new_min = frame_offset
        new_max = frame_offset + n - 1
        new_mean = (new_min + new_max) / 2.0
        # std 不变（帧内相对分布不变）
        stats["index"] = {
            "min": [new_min],
            "max": [new_max],
            "mean": [new_mean],
            "std": old_index.get("std", [0.0]),
            "count": count,
        }

    return result


def validate_datasets(infos: list[dict], dataset_dirs: list[Path]):
    """验证所有数据集是否兼容"""
    base_info = infos[0]
    base_features = set(base_info["features"].keys())

    for i, info in enumerate(infos[1:], 1):
        # 检查 fps 是否一致
        if info["fps"] != base_info["fps"]:
            raise ValueError(
                f"FPS 不一致: {dataset_dirs[0]} 是 {base_info['fps']}, "
                f"{dataset_dirs[i]} 是 {info['fps']}"
            )

        # 检查 features 是否一致
        current_features = set(info["features"].keys())
        if current_features != base_features:
            missing = base_features - current_features
            extra = current_features - base_features
            raise ValueError(
                f"Features 不一致: {dataset_dirs[i]}\n"
                f"  缺少: {missing}\n"
                f"  多余: {extra}"
            )

    print(f"✓ 所有 {len(infos)} 个数据集验证通过，features 和 fps 一致")


def merge_datasets(input_dirs: list[Path], output_dir: Path, max_episodes: int | None = None):
    """合并多个数据集

    Args:
        input_dirs: 输入数据集目录列表
        output_dir: 输出目录
        max_episodes: 最大 episode 数量限制，超过则截断
    """

    # 1. 加载并验证所有数据集
    print("\n[1/5] 加载并验证数据集...")
    infos = []
    for d in input_dirs:
        info = load_info(d)
        if info["total_episodes"] == 0:
            print(f"  ⚠ 跳过空数据集: {d}")
            continue
        infos.append((d, info))

    if len(infos) == 0:
        raise ValueError("没有找到有效的数据集（所有数据集都是空的）")

    dataset_dirs = [d for d, _ in infos]
    info_dicts = [info for _, info in infos]
    validate_datasets(info_dicts, dataset_dirs)

    # 统计总 episode 数
    all_episodes_count = sum(info["total_episodes"] for info in info_dicts)
    if max_episodes is not None:
        print(f"  总 episodes: {all_episodes_count}，限制为: {max_episodes}")
        if max_episodes > all_episodes_count:
            print(f"  ⚠ max_episodes ({max_episodes}) 大于总数 ({all_episodes_count})，将使用全部")
            max_episodes = None
    else:
        print(f"  总 episodes: {all_episodes_count}，不限制数量")

    # 2. 创建输出目录
    print(f"\n[2/5] 创建输出目录: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (output_dir / "meta").mkdir(parents=True, exist_ok=True)
    (output_dir / "videos" / "chunk-000").mkdir(parents=True, exist_ok=True)

    # 3. 合并数据
    print("\n[3/5] 合并 parquet 数据和视频...")

    chunks_size = info_dicts[0].get("chunks_size", 1000)
    total_episodes = 0
    total_frames = 0
    total_videos = 0
    merged_episodes = []  # episodes.jsonl
    merged_episodes_stats = []  # episodes_stats.jsonl

    # task 按文本去重，重新分配 task_index
    task_text_to_idx: dict[str, int] = {}  # {"pick and place": 0, ...}
    merged_tasks = []  # tasks.jsonl

    reached_limit = False

    for dataset_dir, info in infos:
        if reached_limit:
            break

        num_episodes = info["total_episodes"]
        src_chunks_size = info.get("chunks_size", 1000)
        print(f"  处理: {dataset_dir.name} ({num_episodes} episodes)")

        # 加载该数据集的 episodes 元数据
        episodes = load_jsonl(dataset_dir / "meta" / "episodes.jsonl")
        episodes_stats = load_jsonl(dataset_dir / "meta" / "episodes_stats.jsonl")
        tasks = load_jsonl(dataset_dir / "meta" / "tasks.jsonl")

        # 建立该数据集的 old_task_index → task_text 映射
        local_task_map = {t["task_index"]: t["task"] for t in tasks}

        for local_ep_idx in range(num_episodes):
            # 检查是否达到上限
            if max_episodes is not None and total_episodes >= max_episodes:
                reached_limit = True
                print(f"    ✓ 已达到 max_episodes={max_episodes}，停止")
                break

            new_ep_idx = total_episodes

            # 查找 parquet 文件（支持多 chunk）
            src_parquet = find_parquet(dataset_dir, local_ep_idx, src_chunks_size)
            # 输出始终写到 chunk-000（总量 <1000 时）
            dst_parquet = output_dir / "data" / "chunk-000" / f"episode_{new_ep_idx:06d}.parquet"

            if src_parquet is not None:
                # 读取并修改 episode_index
                df = pd.read_parquet(src_parquet)
                df["episode_index"] = new_ep_idx
                # 更新全局 index（先归零再加偏移，防止源数据 index 不从0开始）
                df["index"] = df["frame_index"] + total_frames

                # 更新 task_index 映射
                if "task_index" in df.columns:
                    old_task_indices = df["task_index"].unique()
                    for old_ti in old_task_indices:
                        task_text = local_task_map.get(int(old_ti), f"unknown_task_{old_ti}")
                        if task_text not in task_text_to_idx:
                            new_ti = len(task_text_to_idx)
                            task_text_to_idx[task_text] = new_ti
                            merged_tasks.append({"task_index": new_ti, "task": task_text})
                    # 重映射 task_index
                    df["task_index"] = df["task_index"].map(
                        lambda x: task_text_to_idx.get(local_task_map.get(int(x), f"unknown_task_{x}"), 0)
                    )

                df.to_parquet(dst_parquet)
                ep_frames = len(df)
            else:
                print(f"    ⚠ 找不到 parquet: episode_{local_ep_idx:06d}")
                continue

            # 复制视频文件
            video_keys = [k for k in info["features"].keys() if info["features"][k].get("dtype") == "video"]
            for video_key in video_keys:
                src_video = find_video(dataset_dir, local_ep_idx, video_key, src_chunks_size)
                if src_video is not None:
                    dst_video_dir = output_dir / "videos" / "chunk-000" / video_key
                    dst_video_dir.mkdir(parents=True, exist_ok=True)
                    dst_video = dst_video_dir / f"episode_{new_ep_idx:06d}.mp4"
                    shutil.copy2(src_video, dst_video)
                    total_videos += 1

            # 更新 episodes 元数据
            if local_ep_idx < len(episodes):
                ep_meta = copy.deepcopy(episodes[local_ep_idx])
                ep_meta["episode_index"] = new_ep_idx
                merged_episodes.append(ep_meta)

            # 更新 episodes_stats 元数据（含内部 episode_index/index 修正）
            if local_ep_idx < len(episodes_stats):
                ep_stats = update_episode_stats(
                    episodes_stats[local_ep_idx], new_ep_idx, total_frames, ep_frames
                )
                merged_episodes_stats.append(ep_stats)

            total_frames += ep_frames
            total_episodes += 1

    # 4. 保存元数据
    print("\n[4/5] 保存元数据...")

    # 保存 episodes.jsonl
    save_jsonl(merged_episodes, output_dir / "meta" / "episodes.jsonl")
    print(f"  ✓ episodes.jsonl ({len(merged_episodes)} episodes)")

    # 保存 episodes_stats.jsonl
    save_jsonl(merged_episodes_stats, output_dir / "meta" / "episodes_stats.jsonl")
    print(f"  ✓ episodes_stats.jsonl")

    # 保存 tasks.jsonl
    save_jsonl(merged_tasks, output_dir / "meta" / "tasks.jsonl")
    print(f"  ✓ tasks.jsonl ({len(merged_tasks)} tasks)")

    # 5. 创建合并后的 info.json
    print("\n[5/5] 创建 info.json...")
    merged_info = copy.deepcopy(info_dicts[0])
    merged_info["total_episodes"] = total_episodes
    merged_info["total_frames"] = total_frames
    merged_info["total_videos"] = total_videos
    merged_info["total_tasks"] = len(merged_tasks)
    merged_info["total_chunks"] = 1
    merged_info["splits"] = {"train": f"0:{total_episodes}"}

    with open(output_dir / "meta" / "info.json", "w") as f:
        json.dump(merged_info, f, indent=4)

    print(f"\n{'='*60}")
    print(f"✓ 合并完成!")
    print(f"  输出目录: {output_dir}")
    print(f"  总 episodes: {total_episodes}")
    print(f"  总 frames: {total_frames}")
    print(f"  总 videos: {total_videos}")
    print(f"  总 tasks: {len(merged_tasks)}")
    print(f"{'='*60}")

    return output_dir


def main():
    parser = argparse.ArgumentParser(
        description="合并多个 LeRobot 格式数据集",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "--input_dirs", "-i",
        nargs="+",
        type=Path,
        required=True,
        help="要合并的数据集目录列表"
    )
    parser.add_argument(
        "--output_dir", "-o",
        type=Path,
        required=True,
        help="合并后的数据集输出目录"
    )
    parser.add_argument(
        "--max_episodes", "-n",
        type=int,
        default=None,
        help="最大 episode 数量限制，超过则从末尾截断（默认不限制）"
    )

    args = parser.parse_args()

    # 验证输入目录存在
    for d in args.input_dirs:
        if not d.exists():
            raise FileNotFoundError(f"输入目录不存在: {d}")

    # 检查输出目录
    if args.output_dir.exists():
        response = input(f"输出目录 {args.output_dir} 已存在，是否覆盖? [y/N]: ")
        if response.lower() != "y":
            print("已取消")
            return
        shutil.rmtree(args.output_dir)

    merge_datasets(args.input_dirs, args.output_dir, args.max_episodes)


if __name__ == "__main__":
    main()
