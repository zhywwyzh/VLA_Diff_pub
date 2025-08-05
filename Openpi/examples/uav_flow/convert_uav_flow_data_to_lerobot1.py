import os
import shutil
import gc
import json
import logging
import traceback
import sys
import io

import pandas as pd
import numpy as np
from PIL import Image
import psutil
import tyro
from pathlib import Path

from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME, LeRobotDataset

# Configuration
target_repo = "uav_flow1"
FPS = 5
TARGET_H = 256
TARGET_W = 256
MAX_EPISODES = None  # 可设置测试时处理的最大 episode 数
PARQUET_COLUMNS = ['id', 'frame_idx', 'image', 'log']


def main(data_dir: str, *, push_to_hub: bool = False):
    # 日志配置
    logger = logging.getLogger("uav_sync")
    logger.setLevel(logging.DEBUG)
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
    logger.addHandler(handler)

    try:
        out_dir = HF_LEROBOT_HOME / target_repo
        if out_dir.exists():
            logger.info(f"Removing existing dataset at {out_dir}")
            shutil.rmtree(out_dir)

        # 创建 LeRobotDataset 同步写入
        dataset = LeRobotDataset.create(
            repo_id=target_repo,
            robot_type="uav",
            fps=FPS,
            features={
                "image": {"dtype": "image", "shape": (TARGET_H, TARGET_W, 3)},
                "wrist_image": {"dtype": "image", "shape": (TARGET_H, TARGET_W, 3)},
                "state": {"dtype": "float32", "shape": (6,)},
                "actions": {"dtype": "float32", "shape": (6,)},
            },
            image_writer_threads=0,
            image_writer_processes=0,
        )

        # 找到所有 Parquet 文件
        files = sorted(Path(data_dir).rglob("*.parquet"))
        if not files:
            logger.error("未找到任何 Parquet 文件，请检查 data_dir")
            return
        logger.info(f"发现 {len(files)} 个 Parquet 文件，开始处理...")

        episodes_count = 0
        processed_ids = set()

        def flush_episode(rows):
            nonlocal episodes_count
            if len(rows) < 2:
                return
            eid = rows[0].id
            # 解析日志
            try:
                raw = rows[0].log
                common = json.loads(raw) if isinstance(raw, str) else raw
                pre_logs = common.get('preprocessed_logs')
                task = common.get('instruction_unified', "")
                if not isinstance(pre_logs, list) or len(pre_logs) != len(rows):
                    raise ValueError("preprocessed_logs 长度不匹配")
            except Exception as e:
                logger.error(f"Episode {eid} 日志解析失败: {e}")
                return

            # 处理首帧
            try:
                img0 = rows[0].image
                b0 = img0.get('bytes') if isinstance(img0, dict) else img0
                pil0 = Image.open(io.BytesIO(b0)).convert("RGB").resize((TARGET_W, TARGET_H))
                first_arr = np.array(pil0, dtype=np.uint8)
            except Exception as e:
                logger.error(f"Episode {eid} 首帧图像加载失败: {e}")
                return

            # 写帧
            for i in range(len(rows) - 1):
                try:
                    entry = rows[i].image
                    b = entry.get('bytes') if isinstance(entry, dict) else entry
                    pil = Image.open(io.BytesIO(b)).convert("RGB").resize((TARGET_W, TARGET_H))
                    wrist_arr = np.array(pil, dtype=np.uint8)
                    state = np.array(pre_logs[i], dtype=np.float32)
                    action = np.array(pre_logs[i+1], dtype=np.float32)
                    if state.shape != (6,) or action.shape != (6,):
                        continue
                    dataset.add_frame({
                        "image": first_arr,
                        "wrist_image": wrist_arr,
                        "state": state,
                        "actions": action,
                        "task": task,
                    })
                except Exception as e:
                    logger.warning(f"Episode {eid} 步骤 {i} 写入失败: {e}")

            dataset.save_episode()
            episodes_count += 1
            processed_ids.add(eid)
            logger.info(f"Episode {eid} 同步保存成功 (总计 {episodes_count})")
            # 释放内存
            gc.collect()

        # 遍历文件
        for file in files:
            logger.info(f"正在处理文件: {file}")
            before = psutil.Process().memory_info().rss / (1024*1024)
            df = pd.read_parquet(file, columns=PARQUET_COLUMNS)
            if df.empty:
                df = None; gc.collect(); continue

            current_id = None
            buffer = []
            for row in df.itertuples(index=False):
                if current_id is None or row.id != current_id:
                    # flush 上一个
                    flush_episode(buffer)
                    buffer = [row]
                    current_id = row.id
                else:
                    buffer.append(row)
                if MAX_EPISODES and episodes_count >= MAX_EPISODES:
                    break
            # flush 尾部
            flush_episode(buffer)
            df = None
            buffer = None
            gc.collect()
            after = psutil.Process().memory_info().rss / (1024*1024)
            logger.info(f"文件完成, 内存: 前 {before:.1f}MB, 后 {after:.1f}MB")
            if MAX_EPISODES and episodes_count >= MAX_EPISODES:
                break

        logger.info(f"全部转换完成，共保存 {episodes_count} 个 episode")

        if push_to_hub and episodes_count:
            logger.info("推送到 Hugging Face Hub...")
            dataset.push_to_hub(
                repo_id=target_repo,
                tags=["uav-flow1", "uav"],
                private=False,
                push_videos=True,
                license="apache-2.0",
            )

    except Exception:
        logger.error("发生致命错误", exc_info=True)
        Path("error_logs/crash.txt").write_text(traceback.format_exc())
        raise
    finally:
        logging.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
