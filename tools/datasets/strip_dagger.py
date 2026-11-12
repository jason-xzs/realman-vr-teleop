#!/usr/bin/env python3
"""从 dagger_realman_0212-3 中剥离 control_source 和 policy_action，生成兼容版本"""

import json
import shutil
import pandas as pd
from pathlib import Path

SRC = Path("/data/data/lijiahang/dagger_realman_0212-3")
DST = Path("/data/data/lijiahang/dagger_realman_0212-3_stripped")
REMOVE_KEYS = {"control_source", "policy_action"}

if DST.exists():
    print("ERROR: " + str(DST) + " already exists, aborting")
    exit(1)

# 1. 加载 info
with open(SRC / "meta" / "info.json") as f:
    info = json.load(f)

total_ep = info["total_episodes"]
total_fr = info["total_frames"]
print("Source: %d episodes, %d frames" % (total_ep, total_fr))

# 2. 从 features 中移除多余字段
new_features = {k: v for k, v in info["features"].items() if k not in REMOVE_KEYS}
info["features"] = new_features

video_keys = [k for k, v in new_features.items() if v.get("dtype") == "video"]
print("Keeping features: " + str(list(new_features.keys())))

# 3. 创建输出目录
DST.mkdir(parents=True)
(DST / "meta").mkdir()
(DST / "data" / "chunk-000").mkdir(parents=True)
for vk in video_keys:
    (DST / "videos" / "chunk-000" / vk).mkdir(parents=True)

# 4. 保存 info.json
with open(DST / "meta" / "info.json", "w") as f:
    json.dump(info, f, indent=4)
print("Saved info.json")

# 5. 复制 meta 文件
for meta_file in ["episodes.jsonl", "episodes_stats.jsonl", "tasks.jsonl"]:
    src_f = SRC / "meta" / meta_file
    if src_f.exists():
        shutil.copy2(src_f, DST / "meta" / meta_file)
        print("Copied " + meta_file)

# 6. 处理 parquet —— 去掉多余列
num_ep = info["total_episodes"]
for ep_idx in range(num_ep):
    src_pq = SRC / "data" / "chunk-000" / ("episode_%06d.parquet" % ep_idx)
    dst_pq = DST / "data" / "chunk-000" / ("episode_%06d.parquet" % ep_idx)
    df = pd.read_parquet(src_pq)
    drop_cols = [c for c in df.columns if c in REMOVE_KEYS]
    if drop_cols:
        df = df.drop(columns=drop_cols)
    df.to_parquet(dst_pq)

print("Processed %d parquet files" % num_ep)

# 7. 复制视频文件
for vk in video_keys:
    src_dir = SRC / "videos" / "chunk-000" / vk
    dst_dir = DST / "videos" / "chunk-000" / vk
    if src_dir.exists():
        for f in sorted(src_dir.glob("*.mp4")):
            shutil.copy2(f, dst_dir / f.name)
        print("Copied videos for " + vk)

print("")
print("Done! New dataset at " + str(DST))
print("Episodes: %d" % info["total_episodes"])
