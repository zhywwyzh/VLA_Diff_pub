"""
优化版本：按任务采样创建不同规模的UAV-Flow数据集

该脚本从分类好的UAV-Flow数据中，维护每个task的计数器，
按遇到顺序分配到不同数据集：第1次→1ep，第2次→2ep，第4次→4ep，第8次→8ep

特点：
- 增量处理，不一次性加载所有数据
- 早期停止，收集够了就停止
- 优化内存管理

Usage:
python create_sampled_datasets_optimized.py --data_dir /path/to/uav_flow_raw_categorized --output_dir /path/to/output
"""

import os
import shutil
import gc
import json
import logging
import sys
import io
from collections import defaultdict
from pathlib import Path

import pandas as pd
import numpy as np
from PIL import Image
import psutil
import argparse

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

# Configuration
FPS = 5
TARGET_H = 256
TARGET_W = 256
TARGET_COUNTS = [1, 2, 4, 8]  # 第几次遇到时收集该episode
PARQUET_COLUMNS = ['id', 'frame_idx', 'image', 'log']
DEFAULT_OUTPUT_DIR = "/data/vla/sampled_uav_datasets"

def setup_logger():
    """设置日志"""
    logger = logging.getLogger("uav_sampler_opt")
    logger.setLevel(logging.INFO)
    if logger.hasHandlers():
        logger.handlers.clear()
    
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logger.addHandler(handler)
    return logger

def process_episode(rows: list, task_instruction: str, episode_id: str, logger):
    """处理单个episode，返回处理后的帧数据列表"""
    
    if len(rows) < 2:
        return None
    
    try:
        # 解析日志
        raw_log = rows[0]['log']
        common_log = json.loads(raw_log) if isinstance(raw_log, str) else raw_log
        preprocessed_logs = common_log.get('preprocessed_logs')
        
        if not isinstance(preprocessed_logs, list) or len(preprocessed_logs) != len(rows):
            return None
            
    except Exception:
        return None
    
    try:
        # 处理首帧图像
        first_image_data = rows[0]['image']
        first_image_bytes = first_image_data.get('bytes') if isinstance(first_image_data, dict) else first_image_data
        first_pil = Image.open(io.BytesIO(first_image_bytes)).convert("RGB").resize((TARGET_W, TARGET_H))
        first_array = np.array(first_pil, dtype=np.uint8)
        
    except Exception:
        return None
    
    # 处理所有帧
    frames = []
    for i in range(len(rows) - 1):
        try:
            # 当前帧图像
            current_image_data = rows[i]['image']
            current_image_bytes = current_image_data.get('bytes') if isinstance(current_image_data, dict) else current_image_data
            current_pil = Image.open(io.BytesIO(current_image_bytes)).convert("RGB").resize((TARGET_W, TARGET_H))
            wrist_array = np.array(current_pil, dtype=np.uint8)
            
            # 状态和动作
            state = np.array(preprocessed_logs[i], dtype=np.float32)
            action = np.array(preprocessed_logs[i+1], dtype=np.float32)
            
            if state.shape != (6,) or action.shape != (6,):
                continue
            
            frame = {
                "image": first_array,
                "wrist_image": wrist_array,
                "state": state,
                "actions": action,
                "task": task_instruction,
            }
            frames.append(frame)
            
        except Exception:
            continue
    
    return frames if len(frames) > 0 else None

def create_datasets(output_dir: str, logger):
    """创建所有数据集对象"""
    output_dir = Path(output_dir)
    
    # 如果输出目录已存在，先清理整个目录
    if output_dir.exists():
        logger.info(f"清理已存在的输出目录: {output_dir}")
        try:
            shutil.rmtree(output_dir)
            logger.info(f"成功删除目录: {output_dir}")
        except Exception as e:
            logger.error(f"删除目录失败: {e}")
            raise
    
    # 确认目录不存在
    if output_dir.exists():
        logger.error(f"目录删除后仍然存在: {output_dir}")
        raise RuntimeError(f"无法删除目录: {output_dir}")
    
    logger.info(f"输出目录已清理，准备创建数据集到: {output_dir}")
    
    datasets = {}
    
    for count in TARGET_COUNTS:
        dataset_name = f"uav_flow_{count}ep_per_task"
        dataset_path = output_dir / dataset_name
        
        # 确保每个数据集的子目录不存在
        if dataset_path.exists():
            logger.info(f"清理已存在的数据集目录: {dataset_path}")
            shutil.rmtree(dataset_path)
        
        # 创建数据集
        try:
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                root=output_dir,
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
        except FileExistsError as e:
            # 如果仍然遇到目录存在错误，尝试再次清理
            logger.warning(f"数据集创建失败，可能存在目录冲突: {e}")
            if output_dir.exists():
                logger.info(f"再次清理输出目录: {output_dir}")
                shutil.rmtree(output_dir)
            # 重试创建
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                root=output_dir,
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
        
        datasets[count] = {
            'dataset': dataset,
            'episode_count': 0,
            'frame_count': 0
        }
        
        logger.info(f"创建数据集: {dataset_name}")
    
    return datasets

def collect_and_process_episodes(data_dir: str, datasets: dict, logger):
    """增量收集和处理episodes"""
    logger.info("开始增量收集和处理episodes...")
    
    # 找到所有文件
    files = sorted(Path(data_dir).rglob("*.parquet"))
    logger.info(f"找到 {len(files)} 个parquet文件")
    
    # 维护每个task的计数器
    task_counters = defaultdict(int)
    
    # 统计信息
    total_processed = 0
    total_collected = {count: 0 for count in TARGET_COUNTS}
    
    for file_idx, file_path in enumerate(files):
        logger.info(f"处理文件 {file_idx+1}/{len(files)}: {file_path.name}")
        
        try:
            # 读取文件
            df = pd.read_parquet(file_path, columns=PARQUET_COLUMNS)
            if df.empty:
                continue
            
            # 按episode处理
            for episode_id, episode_df in df.groupby('id'):
                rows = episode_df.sort_values('frame_idx').to_dict('records')
                total_processed += 1
                
                if len(rows) < 2:
                    continue
                
                # 提取task信息
                try:
                    raw_log = rows[0]['log']
                    common_log = json.loads(raw_log) if isinstance(raw_log, str) else raw_log
                    task_instruction = common_log.get('instruction_unified', '')
                    
                    if not task_instruction:
                        continue
                        
                except Exception:
                    continue
                
                # 更新计数器
                task_counters[task_instruction] += 1
                current_count = task_counters[task_instruction]
                
                # 检查是否需要收集该episode
                if current_count in TARGET_COUNTS:
                    logger.info(f"收集任务 '{task_instruction[:30]}...' 的第 {current_count} 个episode: {episode_id}")
                    
                    # 处理episode
                    frames = process_episode(rows, task_instruction, str(episode_id), logger)
                    if frames is None:
                        logger.warning(f"Episode {episode_id} 处理失败，跳过")
                        continue
                    
                    # 保存到对应数据集
                    dataset_info = datasets[current_count]
                    dataset = dataset_info['dataset']
                    
                    dataset.clear_episode_buffer()
                    for frame in frames:
                        dataset.add_frame(frame)
                    dataset.save_episode()
                    
                    # 更新统计
                    dataset_info['episode_count'] += 1
                    dataset_info['frame_count'] += len(frames)
                    total_collected[current_count] += 1
                    
                    logger.info(f"保存到数据集 {current_count}ep: episode {episode_id} ({len(frames)} 帧)")
            
            # 清理内存
            del df
            gc.collect()
            
            # 定期显示进度
            if (file_idx + 1) % 10 == 0:
                logger.info(f"已处理 {file_idx+1}/{len(files)} 个文件，总episodes: {total_processed}")
                logger.info(f"已收集: {total_collected}")
                
                # 内存使用情况
                process = psutil.Process()
                mem_usage = process.memory_info().rss / (1024 * 1024)
                logger.info(f"内存使用: {mem_usage:.1f}MB")
            
        except Exception as e:
            logger.warning(f"处理文件 {file_path} 失败: {e}")
            continue
    
    # 最终统计
    logger.info(f"\n=== 收集完成 ===")
    logger.info(f"总处理episodes: {total_processed}")
    unique_tasks = len(task_counters)
    logger.info(f"发现unique tasks: {unique_tasks}")
    
    for count in TARGET_COUNTS:
        info = datasets[count]
        logger.info(f"数据集 {count}ep: {info['episode_count']} episodes, {info['frame_count']} frames")

def main(data_dir: str, *, output_dir: str = DEFAULT_OUTPUT_DIR, push_to_hub: bool = False):
    """主函数"""
    logger = setup_logger()
    
    try:
        logger.info(f"数据目录: {data_dir}")
        logger.info(f"输出目录: {output_dir}")
        
        # 创建所有数据集
        datasets = create_datasets(output_dir, logger)
        
        # 增量收集和处理episodes
        collect_and_process_episodes(data_dir, datasets, logger)
        
        # 可选：推送到Hub
        if push_to_hub:
            for count in TARGET_COUNTS:
                dataset_name = f"uav_flow_{count}ep_per_task"
                dataset = datasets[count]['dataset']
                episode_count = datasets[count]['episode_count']
                
                if episode_count > 0:
                    logger.info(f"推送数据集 {dataset_name} 到 Hugging Face Hub...")
                    dataset.push_to_hub(
                        repo_id=dataset_name,
                        tags=["uav-flow", "uav", "sampled-dataset"],
                        private=False,
                        push_videos=True,
                        license="apache-2.0",
                    )
                    logger.info(f"数据集 {dataset_name} 已推送到Hub")
        
        logger.info(f"\n{'='*50}")
        logger.info("所有数据集创建完成！")
        logger.info(f"数据集保存在: {output_dir}")
        
        # 显示最终结果
        for count in TARGET_COUNTS:
            info = datasets[count]
            dataset_path = Path(output_dir) / f"uav_flow_{count}ep_per_task"
            logger.info(f"  {dataset_path}: {info['episode_count']} episodes")
        
        logger.info(f"{'='*50}")
        
    except Exception as e:
        logger.error("发生致命错误", exc_info=True)
        raise
    finally:
        logging.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="创建按任务采样的UAV-Flow数据集")
    parser.add_argument("--data_dir", required=True, help="原始数据目录路径")
    parser.add_argument("--output_dir", default=DEFAULT_OUTPUT_DIR, help="输出目录路径")
    parser.add_argument("--push_to_hub", action="store_true", help="是否推送到Hugging Face Hub")
    
    args = parser.parse_args()
    main(args.data_dir, output_dir=args.output_dir, push_to_hub=args.push_to_hub)