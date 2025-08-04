"""
按任务采样创建不同规模的UAV-Flow数据集

该脚本从分类好的UAV-Flow数据中，维护每个task的计数器，
按遇到顺序分配到不同数据集：第1次→1ep，第2次→2ep，第4次→4ep，第8次→8ep

Usage:
python create_sampled_datasets.py --data_dir /path/to/uav_flow_raw_categorized --output_dir /path/to/output

输出4个数据集：
- uav_flow_1ep_per_task (每个任务的第1个episode)  
- uav_flow_2ep_per_task (每个任务的第2个episode)
- uav_flow_4ep_per_task (每个任务的第4个episode)
- uav_flow_8ep_per_task (每个任务的第8个episode)
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
import tyro

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

# Configuration
FPS = 5
TARGET_H = 256
TARGET_W = 256
TARGET_COUNTS = [1, 2, 4, 8]  # 第几次遇到时收集该episode
PARQUET_COLUMNS = ['id', 'frame_idx', 'image', 'log']
DEFAULT_OUTPUT_DIR = "/data/vla/sampled_uav_datasets"  # 新的默认保存路径


def setup_logger():
    """设置日志"""
    logger = logging.getLogger("uav_sampler")
    logger.setLevel(logging.INFO)
    if logger.hasHandlers():
        logger.handlers.clear()
    
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logger.addHandler(handler)
    return logger


def collect_all_episodes(data_dir: str, logger):
    """收集所有episode数据"""
    logger.info("开始收集所有episode数据...")
    
    # 找到所有 Parquet 文件
    files = sorted(Path(data_dir).rglob("*.parquet"))
    if not files:
        logger.error(f"在'{data_dir}'中未找到匹配'*.parquet'的Parquet文件。请检查路径。")
        return None
    
    logger.info(f"发现 {len(files)} 个 Parquet 文件")
    
    all_episodes = []
    
    for file_idx, file_path in enumerate(files):
        logger.info(f"处理文件 {file_idx+1}/{len(files)}: {file_path.name}")
        
        try:
            # 读取parquet文件
            df = pd.read_parquet(file_path, columns=PARQUET_COLUMNS)
            if df.empty:
                continue
                
            # 按episode ID分组
            for episode_id, episode_df in df.groupby('id'):
                rows = episode_df.sort_values('frame_idx').to_dict('records')
                
                if len(rows) < 2:
                    continue
                
                # 提取任务信息
                try:
                    raw_log = rows[0]['log']
                    common_log = json.loads(raw_log) if isinstance(raw_log, str) else raw_log
                    task_instruction = common_log.get('instruction_unified', '')
                    
                    if not task_instruction:
                        logger.warning(f"Episode {episode_id} 缺少instruction_unified，跳过")
                        continue
                        
                    episode_data = {
                        'episode_id': str(episode_id),
                        'task_instruction': task_instruction,
                        'file_path': str(file_path),
                        'data': rows
                    }
                    all_episodes.append(episode_data)
                    
                except Exception as e:
                    logger.warning(f"Episode {episode_id} 解析失败: {e}")
                    continue
                
        except Exception as e:
            logger.warning(f"处理文件 {file_path} 时出错: {e}")
            continue
            
        # 清理内存
        del df
        gc.collect()
    
    logger.info(f"总共收集到 {len(all_episodes)} 个有效episodes")
    return all_episodes


def categorize_episodes_by_count(all_episodes: list, logger):
    """按task计数器分配episodes到不同数据集"""
    logger.info("开始按task计数器分配episodes...")
    
    # 维护每个task的计数器
    task_counters = defaultdict(int)
    
    # 为每个目标计数创建episode列表
    categorized = {count: [] for count in TARGET_COUNTS}
    
    # 统计信息
    task_stats = defaultdict(int)
    
    for episode in all_episodes:
        task = episode['task_instruction']
        task_counters[task] += 1
        current_count = task_counters[task]
        task_stats[task] = current_count
        
        # 检查是否需要收集该episode
        if current_count in TARGET_COUNTS:
            categorized[current_count].append(episode)
            logger.debug(f"收集任务 '{task}' 的第 {current_count} 个episode: {episode['episode_id']}")
    
    # 统计信息
    logger.info("分配完成！统计信息:")
    for count in TARGET_COUNTS:
        logger.info(f"  第{count}次遇到的episodes: {len(categorized[count])} 个")
    
    logger.info("各任务统计:")
    for task, total_count in sorted(task_stats.items()):
        logger.info(f"  '{task}': 总共 {total_count} 个episodes")
    
    return categorized


def process_episode(episode_data: dict, logger):
    """处理单个episode，返回处理后的帧数据列表"""
    episode_id = episode_data['episode_id']
    task_instruction = episode_data['task_instruction']
    rows = episode_data['data']
    
    if len(rows) < 2:
        logger.warning(f"Episode {episode_id} 数据不足(<2帧)，跳过")
        return None
    
    try:
        # 解析日志
        raw_log = rows[0]['log']
        common_log = json.loads(raw_log) if isinstance(raw_log, str) else raw_log
        preprocessed_logs = common_log.get('preprocessed_logs')
        
        if not isinstance(preprocessed_logs, list) or len(preprocessed_logs) != len(rows):
            logger.warning(f"Episode {episode_id} preprocessed_logs长度不匹配，跳过")
            return None
            
    except Exception as e:
        logger.warning(f"Episode {episode_id} 日志解析失败: {e}")
        return None
    
    try:
        # 处理首帧图像
        first_image_data = rows[0]['image']
        first_image_bytes = first_image_data.get('bytes') if isinstance(first_image_data, dict) else first_image_data
        first_pil = Image.open(io.BytesIO(first_image_bytes)).convert("RGB").resize((TARGET_W, TARGET_H))
        first_array = np.array(first_pil, dtype=np.uint8)
        
    except Exception as e:
        logger.warning(f"Episode {episode_id} 首帧图像处理失败: {e}")
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
                logger.warning(f"Episode {episode_id} 步骤 {i} 状态/动作维度错误，跳过该步骤")
                continue
            
            frame = {
                "image": first_array,
                "wrist_image": wrist_array,
                "state": state,
                "actions": action,
                "task": task_instruction,
            }
            frames.append(frame)
            
        except Exception as e:
            logger.warning(f"Episode {episode_id} 步骤 {i} 处理失败: {e}")
            continue
    
    if len(frames) == 0:
        logger.warning(f"Episode {episode_id} 没有有效帧，跳过")
        return None
        
    return frames


def create_dataset(episodes: list, dataset_name: str, output_dir: str, logger):
    """创建LeRobot数据集"""
    logger.info(f"开始创建数据集: {dataset_name}")
    
    if not episodes:
        logger.warning(f"数据集 {dataset_name} 没有可用的episodes，跳过创建")
        return None
    
    # 确保输出目录存在
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 数据集保存路径
    dataset_path = output_dir / dataset_name
    if dataset_path.exists():
        logger.info(f"清理已存在的数据集: {dataset_path}")
        shutil.rmtree(dataset_path)
    
    # 创建新数据集，指定本地路径
    dataset = LeRobotDataset.create(
        repo_id=dataset_name,
        root=output_dir,  # 指定自定义的保存目录
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
    
    total_episodes = 0
    total_frames = 0
    
    # 处理每个episode
    for episode_data in episodes:
        episode_id = episode_data['episode_id']
        task = episode_data['task_instruction']
        
        # 处理episode
        frames = process_episode(episode_data, logger)
        if frames is None:
            continue
        
        # 添加所有帧到数据集
        dataset.clear_episode_buffer()
        for frame in frames:
            dataset.add_frame(frame)
        
        # 保存episode
        dataset.save_episode()
        total_episodes += 1
        total_frames += len(frames)
        
        logger.info(f"保存episode {episode_id} (任务: {task}, {len(frames)} 帧)")
        
        # 定期清理内存
        if total_episodes % 10 == 0:
            gc.collect()
    
    logger.info(f"数据集 {dataset_name} 创建完成:")
    logger.info(f"  总episodes: {total_episodes}")
    logger.info(f"  总帧数: {total_frames}")
    logger.info(f"  保存路径: {dataset_path}")
    
    return dataset


def main(data_dir: str, *, output_dir: str = DEFAULT_OUTPUT_DIR, push_to_hub: bool = False):
    """主函数"""
    logger = setup_logger()
    
    try:
        logger.info(f"输出目录: {output_dir}")
        
        # 1. 收集所有episodes
        all_episodes = collect_all_episodes(data_dir, logger)
        if all_episodes is None:
            logger.error("未找到任何有效的episode数据。退出。")
            return
        
        # 2. 按计数器分配episodes到不同数据集
        categorized = categorize_episodes_by_count(all_episodes, logger)
        
        # 3. 为每个目标计数创建数据集
        for count in TARGET_COUNTS:
            logger.info(f"\n{'='*50}")
            logger.info(f"创建数据集：第 {count} 次遇到的episodes")
            logger.info(f"{'='*50}")
            
            episodes_for_this_count = categorized[count]
            if not episodes_for_this_count:
                logger.warning(f"第 {count} 次遇到的episodes为空，跳过数据集创建")
                continue
            
            # 创建数据集名称
            dataset_name = f"uav_flow_{count}ep_per_task"
            
            # 创建和保存数据集
            try:
                dataset = create_dataset(episodes_for_this_count, dataset_name, output_dir, logger)
                
                if dataset is None:
                    logger.warning(f"数据集 {dataset_name} 创建失败，跳过")
                    continue
                
                # 可选：推送到Hub
                if push_to_hub:
                    logger.info(f"推送数据集 {dataset_name} 到 Hugging Face Hub...")
                    dataset.push_to_hub(
                        repo_id=dataset_name,
                        tags=["uav-flow", "uav", "sampled-dataset"],
                        private=False,
                        push_videos=True,
                        license="apache-2.0",
                    )
                    logger.info(f"数据集 {dataset_name} 已推送到Hub")
                
            except Exception as e:
                logger.error(f"创建数据集 {dataset_name} 时出错: {e}", exc_info=True)
                continue
            
            # 清理内存
            del dataset
            gc.collect()
        
        logger.info(f"\n{'='*50}")
        logger.info("所有数据集创建完成！")
        logger.info(f"数据集保存在: {output_dir}")
        logger.info(f"{'='*50}")
        
        # 最终清理
        del all_episodes
        del categorized
        gc.collect()
        
    except Exception as e:
        logger.error("发生致命错误", exc_info=True)
        raise
    finally:
        logging.shutdown()


if __name__ == "__main__":
    tyro.cli(main)