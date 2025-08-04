"""
从UAV-Flow数据集中按任务抽取小数据集的脚本。

该脚本读取原始Parquet文件，按任务分组，然后为每个任务抽取指定数量的episode，
生成4个不同大小的数据集：每任务1条、2条、4条、8条数据。

用法:
uv run examples/uav_flow/convert_uav_flow_data_to_lerobot_small.py --data_dir /path/to/your/uav_parquet_files_folder

注意: 需要安装pandas, pyarrow, 和Pillow:
`uv pip install pandas pyarrow Pillow`

生成的数据集将保存到$HF_LEROBOT_HOME目录下。
"""

import shutil
import os
import glob
import pandas as pd
import numpy as np
from PIL import Image
import io 
import json 
import logging 
import gc
import psutil
import traceback
import sys
import random
from collections import defaultdict
from typing import Dict, List, Tuple

from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import tyro

# 配置参数
BASE_REPO_NAME = "uav_flow_small"  # 基础数据集名称
TARGET_IMAGE_HEIGHT = 256
TARGET_IMAGE_WIDTH = 256
FPS = 5
RANDOM_SEED = 42  # 设置随机种子以确保结果可重现

# 定义要生成的数据集配置：每个任务的episode数量
DATASET_CONFIGS = [
    {"episodes_per_task": 1, "suffix": "_1ep"},
    {"episodes_per_task": 2, "suffix": "_2ep"}, 
    {"episodes_per_task": 4, "suffix": "_4ep"},
    {"episodes_per_task": 8, "suffix": "_8ep"}
]


def collect_all_episodes_by_task(data_dir: str, logger: logging.Logger) -> Dict[str, List[Tuple[str, str, pd.DataFrame]]]:
    """
    扫描所有parquet文件，收集所有episode并按task分组。
    
    返回: {task_instruction: [(episode_id, file_path, episode_dataframe), ...]}
    """
    logger.info("开始收集所有episode并按任务分组...")
    
    # 找到所有parquet文件
    file_pattern = "*.parquet"
    parquet_files = sorted(glob.glob(os.path.join(data_dir, file_pattern)))
    if not parquet_files:
        logger.error(f"在'{data_dir}'中未找到匹配'{file_pattern}'的Parquet文件。请检查路径。")
        return {}
    
    logger.info(f"找到{len(parquet_files)}个Parquet文件进行处理。")
    
    episodes_by_task = defaultdict(list)
    total_episodes_found = 0
    
    for pq_file_idx, pq_file in enumerate(parquet_files):
        logger.info(f"扫描文件 {pq_file_idx + 1}/{len(parquet_files)}: {pq_file}...")
        
        try:
            current_df = pd.read_parquet(pq_file)
            logger.debug(f"文件 {pq_file} 加载完成，包含 {len(current_df)} 行数据。")
        except Exception as e:
            logger.error(f"读取Parquet文件 {pq_file} 时出错: {e}。跳过此文件。")
            continue
            
        if current_df.empty:
            logger.warning(f"文件 {pq_file} 为空。跳过。")
            continue
            
        # 按id和frame_idx排序
        current_df.sort_values(by=['id', 'frame_idx'], inplace=True)
        
        # 按'id'分组处理每个episode
        episode_groups = list(current_df.groupby('id'))
        
        for episode_id, episode_df in episode_groups:
            episode_objects = episode_df.to_dict('records')
            num_objects_in_episode = len(episode_objects)
            
            if num_objects_in_episode < 2:
                logger.debug(f"Episode '{episode_id}' 只有 {num_objects_in_episode} 个对象，少于要求的2个。跳过。")
                continue
                
            try:
                # 提取任务指令
                raw_common_log = episode_objects[0]['log']
                common_log = json.loads(raw_common_log) if isinstance(raw_common_log, str) else raw_common_log
                if not isinstance(common_log, dict):
                    logger.debug(f"Episode '{episode_id}': 'log'字段不是字典或有效的JSON字符串。跳过。")
                    continue
                
                task_instruction = common_log.get('instruction_unified', "")
                if not task_instruction.strip():
                    logger.debug(f"Episode '{episode_id}': 任务指令为空。跳过。")
                    continue
                    
                preprocessed_logs_list = common_log.get('preprocessed_logs')
                if preprocessed_logs_list is None:
                    logger.debug(f"Episode '{episode_id}': 'log'数据中缺少'preprocessed_logs'。跳过。")
                    continue
                
                if len(preprocessed_logs_list) != num_objects_in_episode:
                    logger.debug(f"Episode '{episode_id}': 日志列表长度与对象数量不匹配。跳过。")
                    continue
                    
                # 验证第一张图片
                raw_first_image_data = episode_objects[0]['image']
                first_image_bytes = raw_first_image_data.get('bytes') if isinstance(raw_first_image_data, dict) else raw_first_image_data
                if not isinstance(first_image_bytes, bytes) or not first_image_bytes:
                    logger.debug(f"Episode '{episode_id}': 第一张图片数据无效。跳过。")
                    continue
                    
                # 如果所有验证都通过，将episode添加到对应任务组
                episodes_by_task[task_instruction].append((str(episode_id), pq_file, episode_df.copy()))
                total_episodes_found += 1
                
            except (KeyError, json.JSONDecodeError, Exception) as e:
                logger.debug(f"Episode '{episode_id}': 处理'log'数据时出错: {e}。跳过episode。")
                continue
        
        # 清理内存
        del episode_groups
        del current_df
        gc.collect()
    
    logger.info(f"总共找到 {total_episodes_found} 个有效episode，分布在 {len(episodes_by_task)} 个不同任务中。")
    
    # 打印每个任务的episode数量统计
    for task, episodes in episodes_by_task.items():
        logger.info(f"任务 '{task[:50]}...' 有 {len(episodes)} 个episode")
    
    return dict(episodes_by_task)


def select_episodes_for_task(episodes: List[Tuple], episodes_per_task: int, logger: logging.Logger) -> List[Tuple]:
    """
    为给定任务选择指定数量的episode。
    如果可用episode数少于所需数量，返回所有可用的episode。
    """
    available_count = len(episodes)
    if available_count <= episodes_per_task:
        logger.debug(f"任务可用episode数 ({available_count}) <= 所需数量 ({episodes_per_task})，返回所有episode")
        return episodes
    else:
        # 随机选择指定数量的episode
        selected = random.sample(episodes, episodes_per_task)
        logger.debug(f"从 {available_count} 个episode中随机选择了 {episodes_per_task} 个")
        return selected


def process_episode_for_dataset(episode_id: str, episode_df: pd.DataFrame, dataset: LeRobotDataset, logger: logging.Logger) -> bool:
    """
    处理单个episode并添加到数据集中。
    返回是否成功添加了frames。
    """
    episode_objects = episode_df.to_dict('records')
    num_objects_in_episode = len(episode_objects)
    
    try:
        # 提取任务指令和预处理日志
        raw_common_log = episode_objects[0]['log']
        common_log = json.loads(raw_common_log) if isinstance(raw_common_log, str) else raw_common_log
        task_instruction = common_log.get('instruction_unified', "")
        preprocessed_logs_list = common_log.get('preprocessed_logs')
        
        # 处理第一张图片
        raw_first_image_data = episode_objects[0]['image']
        first_image_bytes = raw_first_image_data.get('bytes') if isinstance(raw_first_image_data, dict) else raw_first_image_data
        pil_first_image = Image.open(io.BytesIO(first_image_bytes)).convert("RGB").resize((TARGET_IMAGE_WIDTH, TARGET_IMAGE_HEIGHT))
        first_image_arr = np.array(pil_first_image, dtype=np.uint8)
        
    except Exception as e:
        logger.error(f"Episode '{episode_id}': 预处理失败: {e}")
        return False
    
    frames_added = 0
    for step_idx in range(num_objects_in_episode - 1):
        try:
            # 处理当前步骤的图片
            raw_current_image_data = episode_objects[step_idx]['image']
            current_image_bytes = raw_current_image_data.get('bytes') if isinstance(raw_current_image_data, dict) else raw_current_image_data
            if not isinstance(current_image_bytes, bytes) or not current_image_bytes:
                logger.warning(f"Episode '{episode_id}', Step {step_idx}: 手腕图片数据无效。跳过步骤。")
                continue
            pil_wrist_image = Image.open(io.BytesIO(current_image_bytes)).convert("RGB").resize((TARGET_IMAGE_WIDTH, TARGET_IMAGE_HEIGHT))
            wrist_image_arr = np.array(pil_wrist_image, dtype=np.uint8)
            
            # 处理状态和动作数据
            state_data = np.array(preprocessed_logs_list[step_idx], dtype=np.float32)
            action_data = np.array(preprocessed_logs_list[step_idx + 1], dtype=np.float32)
            if state_data.shape != (6,) or action_data.shape != (6,):
                logger.warning(f"Episode '{episode_id}', Step {step_idx}: 状态/动作形状不正确。跳过步骤。")
                continue
            
            frame_payload = {
                "image": first_image_arr,
                "wrist_image": wrist_image_arr,
                "state": state_data,
                "actions": action_data,
                "task": task_instruction,
            }
            dataset.add_frame(frame_payload)
            frames_added += 1
            
        except Exception as e:
            logger.warning(f"Episode '{episode_id}', Step {step_idx}: 处理步骤时出错: {e}。跳过步骤。")
            continue
    
    if frames_added > 0:
        dataset.save_episode()
        logger.debug(f"成功保存episode '{episode_id}'，包含 {frames_added} 个步骤。")
        return True
    else:
        logger.warning(f"Episode '{episode_id}' 没有添加任何frame，因此未保存。")
        return False


def create_dataset_with_config(episodes_by_task: Dict[str, List[Tuple]], config: Dict, logger: logging.Logger) -> bool:
    """
    根据配置创建一个数据集。
    """
    episodes_per_task = config["episodes_per_task"]
    suffix = config["suffix"]
    repo_name = BASE_REPO_NAME + suffix
    
    logger.info(f"开始创建数据集 '{repo_name}' (每任务 {episodes_per_task} 个episode)...")
    
    # 清理现有数据集
    output_path = HF_LEROBOT_HOME / repo_name
    if output_path.exists():
        logger.info(f"清理现有数据集: {output_path}")
        shutil.rmtree(output_path)
    
    # 创建LeRobot数据集
    try:
        dataset = LeRobotDataset.create(
            repo_id=repo_name,
            robot_type="uav",
            fps=FPS,
            features={
                "image": {
                    "dtype": "image",
                    "shape": (TARGET_IMAGE_HEIGHT, TARGET_IMAGE_WIDTH, 3),
                    "names": ["height", "width", "channel"],
                },
                "wrist_image": {
                    "dtype": "image", 
                    "shape": (TARGET_IMAGE_HEIGHT, TARGET_IMAGE_WIDTH, 3),
                    "names": ["height", "width", "channel"],
                },
                "state": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": ["state"],
                },
                "actions": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": ["actions"],
                },
            },
            image_writer_threads=10,
            image_writer_processes=5,
        )
    except Exception as e:
        logger.error(f"创建数据集 '{repo_name}' 失败: {e}")
        return False
    
    total_episodes_processed = 0
    total_tasks_processed = 0
    
    # 为每个任务选择并处理episode
    for task_instruction, available_episodes in episodes_by_task.items():
        selected_episodes = select_episodes_for_task(available_episodes, episodes_per_task, logger)
        
        task_episodes_processed = 0
        for episode_id, file_path, episode_df in selected_episodes:
            if process_episode_for_dataset(episode_id, episode_df, dataset, logger):
                task_episodes_processed += 1
                total_episodes_processed += 1
        
        if task_episodes_processed > 0:
            total_tasks_processed += 1
            logger.debug(f"任务 '{task_instruction[:50]}...' 成功处理了 {task_episodes_processed} 个episode")
        
        # 定期垃圾回收
        if total_episodes_processed % 50 == 0:
            gc.collect()
    
    logger.info(f"数据集 '{repo_name}' 创建完成:")
    logger.info(f"  - 处理了 {total_tasks_processed} 个任务")
    logger.info(f"  - 总共处理了 {total_episodes_processed} 个episode")
    
    return total_episodes_processed > 0


def main(data_dir: str):
    # 设置随机种子
    random.seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)
    
    # 设置日志
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    
    if logger.hasHandlers():
        logger.handlers.clear()
    
    # 创建错误日志文件处理器
    error_log_path = "/data/vla/Openpi/examples/uav_flow/error_small.log"
    try:
        os.makedirs(os.path.dirname(error_log_path), exist_ok=True)
        file_handler = logging.FileHandler(error_log_path, mode='w')
    except OSError as e:
        print(f"警告: 无法为错误日志文件创建目录 {error_log_path}: {e}。错误将不会保存到文件。")
        file_handler = None
    
    if file_handler:
        file_handler.setLevel(logging.WARNING)
        file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s')
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
    
    # 创建控制台日志处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    try:
        # 第一步：收集所有episode并按任务分组
        episodes_by_task = collect_all_episodes_by_task(data_dir, logger)
        
        if not episodes_by_task:
            logger.error("未找到任何有效的episode数据。退出。")
            return
        
        logger.info(f"按任务分组完成，找到 {len(episodes_by_task)} 个不同的任务。")
        
        # 第二步：为每个配置创建数据集
        successful_datasets = 0
        for config in DATASET_CONFIGS:
            logger.info(f"\n{'='*50}")
            if create_dataset_with_config(episodes_by_task, config, logger):
                successful_datasets += 1
            else:
                logger.error(f"创建数据集失败: {config}")
            
            # 强制垃圾回收
            gc.collect()
        
        logger.info(f"\n{'='*50}")
        logger.info(f"所有数据集创建完成！成功创建了 {successful_datasets}/{len(DATASET_CONFIGS)} 个数据集。")
        
        if successful_datasets == 0:
            logger.error("所有数据集创建都失败了。")
        
    except Exception as e:
        logger.error("主函数中发生未处理的异常:", exc_info=True)
        # 写入详细的异常信息到单独文件用于调试
        crash_details_path = os.path.join(os.path.dirname(error_log_path), "crash_details_small.txt")
        with open(crash_details_path, "w") as crash_file:
            crash_file.write(f"异常: {str(e)}\n\n")
            crash_file.write(f"堆栈跟踪:\n{traceback.format_exc()}\n\n")
            crash_file.write(f"Python版本: {sys.version}\n")
            crash_file.write(f"崩溃时内存使用: RSS={psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024):.2f}MB\n")
        raise
    finally:
        logging.shutdown()


if __name__ == "__main__":
    tyro.cli(main)