"""
优化版本：按任务采样创建不同规模的UAV-Flow数据集

该脚本从分类好的UAV-Flow数据中，为每个task收集固定数量的episodes，
创建4个不同规模的数据集：
- 1ep数据集：每个task包含1个episode
- 2ep数据集：每个task包含2个episode  
- 4ep数据集：每个task包含4个episode
- 8ep数据集：每个task包含8个episode

特点：
- 增量处理，不一次性加载所有数据
- 为每个task确保相同数量的episodes
- 优化内存管理

Usage:
python create_sampled_datasets_optimized.py --data_dir /data/vla/uav_flow_raw_categorized --output_dir /path/to/output
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
TARGET_COUNTS = [8]  # 每个task收集的episodes数量
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
            
    except Exception as e:
        logger.debug(f"Episode {episode_id}: 日志解析失败: {e}")
        return None
    
    try:
        # 处理首帧图像
        first_image_data = rows[0]['image']
        first_image_bytes = first_image_data.get('bytes') if isinstance(first_image_data, dict) else first_image_data
        if not isinstance(first_image_bytes, bytes) or not first_image_bytes:
            return None
        
        first_pil = Image.open(io.BytesIO(first_image_bytes)).convert("RGB").resize((TARGET_W, TARGET_H))
        first_array = np.array(first_pil, dtype=np.uint8)
        
        # 及时清理PIL对象
        first_pil.close()
        del first_pil
        
    except Exception as e:
        logger.debug(f"Episode {episode_id}: 首帧图像处理失败: {e}")
        return None
    
    # 处理所有帧
    frames = []
    for i in range(len(rows) - 1):
        try:
            # 当前帧图像
            current_image_data = rows[i]['image']
            current_image_bytes = current_image_data.get('bytes') if isinstance(current_image_data, dict) else current_image_data
            if not isinstance(current_image_bytes, bytes) or not current_image_bytes:
                continue
                
            current_pil = Image.open(io.BytesIO(current_image_bytes)).convert("RGB").resize((TARGET_W, TARGET_H))
            wrist_array = np.array(current_pil, dtype=np.uint8)
            
            # 及时清理PIL对象
            current_pil.close()
            del current_pil
            
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
            
        except Exception as e:
            logger.debug(f"Episode {episode_id}, Step {i}: 帧处理失败: {e}")
            # 确保清理可能的PIL对象
            try:
                if 'current_pil' in locals():
                    current_pil.close()
                    del current_pil
            except:
                pass
            continue
    
    return frames if len(frames) > 0 else None

def decide_sampled_episodes(data_dir: str, logger):
    """扫描数据文件，为每个task收集指定数量的episodes（使用计数器和动态移除机制）"""
    logger.info("开始扫描数据，为每个target count收集episodes...")
    
    # 找到所有文件
    files = sorted(Path(data_dir).rglob("*.parquet"))
    logger.info(f"找到 {len(files)} 个parquet文件")
    
    # 为每个target count维护独立的收集状态
    sampled_episodes_by_count = {}
    task_counters_by_count = {}  # count -> {task: current_count}
    collected_episodes_by_count = {}  # count -> {task: [episodes]}
    
    for count in TARGET_COUNTS:
        sampled_episodes_by_count[count] = []
        task_counters_by_count[count] = defaultdict(int)
        collected_episodes_by_count[count] = defaultdict(list)
    
    # 统计信息
    total_processed = 0
    unique_tasks_seen = set()
    
    logger.info("开始全局扫描，使用计数器收集episodes...")
    for file_idx, file_path in enumerate(files):
        if (file_idx + 1) % 10 == 0:
            logger.info(f"扫描文件 {file_idx+1}/{len(files)}: {file_path.name}")
        
        try:
            # 读取文件（只读取必要的列）
            df = pd.read_parquet(file_path, columns=['id', 'frame_idx', 'log'])
            if df.empty:
                continue
            
            # 按episode处理
            for episode_id, episode_df in df.groupby('id'):
                total_processed += 1
                
                if len(episode_df) < 2:
                    continue
                
                # 只取第一行来提取task信息
                first_row = episode_df.iloc[0]
                
                # 提取task信息
                try:
                    raw_log = first_row['log']
                    common_log = json.loads(raw_log) if isinstance(raw_log, str) else raw_log
                    task_instruction = common_log.get('instruction_unified', '')
                    
                    if not task_instruction:
                        continue
                        
                except Exception:
                    continue
                
                unique_tasks_seen.add(task_instruction)
                
                # 为每个target count检查是否需要收集这个episode
                for count in TARGET_COUNTS:
                    current_count = task_counters_by_count[count][task_instruction]
                    
                    if current_count < count:  # 还需要更多episodes
                        # 记录episode信息
                        episode_info = {
                            'file_path': str(file_path),
                            'episode_id': episode_id,
                            'task_instruction': task_instruction,
                            'frame_count': len(episode_df),
                            'target_count': count
                        }
                        
                        collected_episodes_by_count[count][task_instruction].append(episode_info)
                        task_counters_by_count[count][task_instruction] += 1
                        
                        # 如果这个task已经收集够了，移到最终列表
                        if task_counters_by_count[count][task_instruction] == count:
                            episodes_for_this_task = collected_episodes_by_count[count][task_instruction]
                            sampled_episodes_by_count[count].extend(episodes_for_this_task)
                            
                            # 清理已完成的task以节省内存
                            del collected_episodes_by_count[count][task_instruction]
                            
                            logger.debug(f"Task '{task_instruction}' 已收集满 {count} episodes")
            
            # 清理内存
            del df
            gc.collect()
            
        except Exception as e:
            logger.warning(f"扫描文件 {file_path} 失败: {e}")
            continue
    
    # 处理未收集满的tasks（将部分收集的episodes也加入）
    for count in TARGET_COUNTS:
        incomplete_tasks = 0
        for task_instruction, episodes in collected_episodes_by_count[count].items():
            if len(episodes) > 0:
                incomplete_tasks += 1
                logger.debug(f"Task '{task_instruction}' 在 {count}ep数据集中只收集到 {len(episodes)} episodes (目标{count}个)")
                # 包含不完整的tasks，使用它们现有的所有episodes
                sampled_episodes_by_count[count].extend(episodes)
        
        # 统计结果
        total_episodes = len(sampled_episodes_by_count[count])
        
        # 计算完整和不完整的tasks数量
        task_episode_counts = {}
        for episode_info in sampled_episodes_by_count[count]:
            task = episode_info['task_instruction']
            task_episode_counts[task] = task_episode_counts.get(task, 0) + 1
        
        complete_tasks = sum(1 for task_count in task_episode_counts.values() if task_count == count)
        total_tasks = len(task_episode_counts)
        
        logger.info(f"  {count}ep数据集收集完成:")
        logger.info(f"    总tasks: {total_tasks} tasks")
        logger.info(f"    完整tasks: {complete_tasks} tasks (有{count}个episodes)")
        logger.info(f"    不完整tasks: {incomplete_tasks} tasks (少于{count}个episodes)")
        logger.info(f"    总episodes: {total_episodes}")
        
        # 验证
        if total_tasks != complete_tasks + incomplete_tasks:
            logger.warning(f"    警告：tasks统计不匹配！")
    
    # 生成task统计jsonl文件
    logger.info(f"\n=== 生成task统计文件 ===")
    
    # 统计每个task在每个数据集中的episode数量
    task_stats = {}
    for task in unique_tasks_seen:
        task_stats[task] = {}
        for count in TARGET_COUNTS:
            # 计算这个task在这个count数据集中有多少episodes
            task_episodes_in_dataset = [ep for ep in sampled_episodes_by_count[count] 
                                      if ep['task_instruction'] == task]
            task_stats[task][f"{count}ep_dataset"] = len(task_episodes_in_dataset)
    
    # 保存到jsonl文件
    jsonl_path = "/data/vla/sampled_uav_datasets/task_episode_stats.jsonl"
    try:
        os.makedirs(os.path.dirname(jsonl_path), exist_ok=True)
        with open(jsonl_path, 'w', encoding='utf-8') as f:
            for task, stats in task_stats.items():
                record = {
                    "task": task,
                    "total_episodes_available": sum(stats.values()) if TARGET_COUNTS else 0,
                    **stats
                }
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        
        logger.info(f"Task统计文件已保存到: {jsonl_path}")
        logger.info(f"包含 {len(task_stats)} 个tasks的统计信息")
        
    except Exception as e:
        logger.warning(f"保存task统计文件失败: {e}")
    
    # 最终统计
    logger.info(f"\n=== 全局收集完成 ===")
    logger.info(f"总扫描episodes: {total_processed}")
    logger.info(f"发现unique tasks: {len(unique_tasks_seen)}")
    
    return sampled_episodes_by_count

def create_final_datasets_sequentially(data_dir: str, sampled_episodes_by_count, output_dir: str, logger):
    """按顺序创建最终数据集，避免同时维护多个数据集"""
    logger.info("开始按顺序创建最终数据集...")
    
    output_dir = Path(output_dir)
    
    # 确保输出目录存在
    output_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"确保输出目录存在: {output_dir}")
    
    # 提前清理所有可能存在的count目录，避免冲突
    for count in TARGET_COUNTS:
        count_output_dir = output_dir / f"{count}ep_dataset"
        if count_output_dir.exists():
            logger.info(f"预清理count目录: {count_output_dir}")
            shutil.rmtree(count_output_dir)
    
    final_results = {}
    
    # 重新处理原始数据，但只收集特定count的episodes
    for count in TARGET_COUNTS:
        if count not in sampled_episodes_by_count or len(sampled_episodes_by_count[count]) == 0:
            logger.info(f"跳过数据集 {count}ep (没有收集到数据)")
            final_results[count] = {'episode_count': 0, 'frame_count': 0}
            continue
        
        logger.info(f"\n--- 创建数据集: uav_flow_{count}ep_per_task ---")
        
        # 为每个数据集创建独立的文件夹
        dataset_name = f"uav_flow_{count}ep_per_task"
        count_output_dir = output_dir / f"{count}ep_dataset"
        dataset_path = count_output_dir / dataset_name
        
        # 不需要预先创建目录，让LeRobot自己创建
        logger.info(f"数据集将保存到: {count_output_dir}")
        logger.info(f"目录状态检查 - count_output_dir存在: {count_output_dir.exists()}")
        
        # 创建当前count的数据集
        logger.info(f"开始创建LeRobot数据集，repo_id={dataset_name}, root={count_output_dir}")
        dataset = LeRobotDataset.create(
            repo_id=dataset_name,
            root=count_output_dir,  # 使用独立的输出目录
            robot_type="uav",
            fps=FPS,
            features={
                "image": {"dtype": "image", "shape": (TARGET_H, TARGET_W, 3)},
                "wrist_image": {"dtype": "image", "shape": (TARGET_H, TARGET_W, 3)},
                "state": {"dtype": "float32", "shape": (6,)},
                "actions": {"dtype": "float32", "shape": (6,)},
            },
            image_writer_threads=6,  # 单个数据集时可以用更多资源
            image_writer_processes=3,
        )
        
        # 获取这个count需要的episodes信息
        target_episodes = sampled_episodes_by_count[count]
        target_tasks = {ep['task_instruction'] for ep in target_episodes}
        
        logger.info(f"需要重新处理 {len(target_episodes)} 个episodes，涉及 {len(target_tasks)} 个tasks")
        
        # 处理当前count的episodes
        episode_count, frame_count = process_episodes_for_count(
            dataset, target_episodes, count, logger
        )
        
        final_results[count] = {
            'dataset': dataset,
            'episode_count': episode_count,
            'frame_count': frame_count,
            'dataset_path': count_output_dir / dataset_name
        }
        
        logger.info(f"完成数据集 {dataset_name}: {episode_count} episodes, {frame_count} frames")
        
        # 释放当前数据集的内存
        del dataset
        gc.collect()
    
    # 重新加载数据集对象用于返回
    final_datasets = {}
    for count in TARGET_COUNTS:
        if final_results[count]['episode_count'] > 0:
            dataset_name = f"uav_flow_{count}ep_per_task"
            # 这里可以重新加载数据集，但为了节省内存，我们只返回统计信息
            final_datasets[count] = final_results[count]
    
    return final_datasets

def process_episodes_for_count(dataset, target_episodes, target_count: int, logger):
    """处理指定count的episodes并保存到数据集"""
    episode_count = 0
    frame_count = 0
    
    # 按文件分组，减少文件读取次数
    episodes_by_file = defaultdict(list)
    for ep in target_episodes:
        episodes_by_file[ep['file_path']].append(ep)
    
    logger.info(f"需要处理 {len(episodes_by_file)} 个文件中的 {len(target_episodes)} 个episodes")
    
    for file_path, file_episodes in episodes_by_file.items():
        logger.info(f"处理文件 {Path(file_path).name} 中的 {len(file_episodes)} 个episodes")
        
        try:
            # 读取整个文件
            df = pd.read_parquet(file_path, columns=PARQUET_COLUMNS)
            if df.empty:
                continue
            
            # 只处理目标episodes
            target_episode_ids = {ep['episode_id'] for ep in file_episodes}
            
            for episode_id, episode_df in df.groupby('id'):
                if episode_id not in target_episode_ids:
                    continue
                
                rows = episode_df.sort_values('frame_idx').to_dict('records')
                
                if len(rows) < 2:
                    continue
                
                # 从episodes信息中获取task
                episode_info = next((ep for ep in file_episodes if ep['episode_id'] == episode_id), None)
                if episode_info is None:
                    continue
                    
                task_instruction = episode_info['task_instruction']
                
                # 处理并保存episode
                frames = process_episode(rows, task_instruction, str(episode_id), logger)
                if frames is None:
                    logger.warning(f"Episode {episode_id} 处理失败，跳过")
                    continue
                
                dataset.clear_episode_buffer()
                for frame in frames:
                    dataset.add_frame(frame)
                dataset.save_episode()
                
                episode_count += 1
                frame_count += len(frames)
                
                if episode_count % 10 == 0:
                    logger.info(f"已处理 {episode_count}/{len(target_episodes)} episodes")
            
            del df
            gc.collect()
            
        except Exception as e:
            logger.warning(f"处理文件 {file_path} 失败: {e}")
            continue
    
    return episode_count, frame_count

def main(data_dir: str, *, output_dir: str = DEFAULT_OUTPUT_DIR, push_to_hub: bool = False):
    """主函数"""
    logger = setup_logger()
    
    try:
        logger.info(f"数据目录: {data_dir}")
        logger.info(f"输出目录: {output_dir}")
        
        # 步骤1: 扫描数据，决定采样策略
        logger.info("\n=== 步骤1: 扫描数据并决定采样策略 ===")
        sampled_episodes = decide_sampled_episodes(data_dir, logger)
        
        # 步骤2: 按顺序创建最终的分割数据集
        logger.info("\n=== 步骤2: 按顺序创建最终分割数据集 ===")
        final_datasets = create_final_datasets_sequentially(data_dir, sampled_episodes, output_dir, logger)
        
        # 可选：推送到Hub
        if push_to_hub:
            logger.info("\n=== 推送到Hugging Face Hub ===")
            for count in TARGET_COUNTS:
                if count not in final_datasets:
                    continue
                    
                dataset_name = f"uav_flow_{count}ep_per_task"
                dataset = final_datasets[count]['dataset']
                episode_count = final_datasets[count]['episode_count']
                
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
        logger.info(f"数据集根目录: {output_dir}")
        
        # 显示最终结果
        for count in TARGET_COUNTS:
            if count not in final_datasets:
                logger.info(f"  uav_flow_{count}ep_per_task: 0 episodes (未收集到数据)")
            else:
                info = final_datasets[count]
                # 使用保存的具体数据集路径
                if 'dataset_path' in info:
                    dataset_path = info['dataset_path']
                else:
                    # 如果没有保存路径信息，则构造路径
                    dataset_path = Path(output_dir) / f"{count}ep_dataset" / f"uav_flow_{count}ep_per_task"
                logger.info(f"  {dataset_path}: {info['episode_count']} episodes, {info['frame_count']} frames")
        
        # 最终内存使用报告
        try:
            process = psutil.Process()
            final_memory = process.memory_info().rss / (1024 * 1024)
            logger.info(f"最终内存使用: {final_memory:.1f}MB")
        except:
            pass
        
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