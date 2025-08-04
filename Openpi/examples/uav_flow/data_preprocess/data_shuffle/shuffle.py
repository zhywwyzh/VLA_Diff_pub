import os
import pandas as pd
import numpy as np
from pathlib import Path
from tqdm import tqdm
from collections import defaultdict

def save_and_clear_buffer(buffer, base_path, category_name, file_counters):
    """Helper function to save buffered data to a parquet file and clear the buffer."""
    if not buffer:
        return
    
    output_df = pd.concat(buffer, ignore_index=True)
    split_name = base_path.name # 'train' or 'test'
    
    output_filename = f"{category_name}-{file_counters[split_name]:05d}.parquet"
    output_filepath = base_path / output_filename
    
    # 使用 pyarrow 引擎并设置较大的行组大小，以避免下游读取时出现 "chunked array" 的问题
    output_df.to_parquet(output_filepath, index=False, engine='pyarrow', row_group_size=len(output_df))
    
    file_counters[split_name] += 1
    buffer.clear()

def process_uav_dataset(source_dir, dest_dir, target_category=None, default_test_split_ratio=0.1, episodes_per_file=100):
    """
    处理整个无人机轨迹数据集，将其分割为训练集和测试集，并优化内存使用。
    可以指定 target_category 来只处理单个类别。
    """
    source_path = Path(source_dir)
    dest_path = Path(dest_dir)

    if not source_path.is_dir():
        print(f"错误：源目录 '{source_dir}' 不存在。")
        return

    if target_category:
        # 如果指定了目标类别，只处理该类别
        category_path = source_path / target_category
        if not category_path.is_dir():
            print(f"错误：指定的目标类别目录 '{category_path}' 不存在。")
            return
        categories = [category_path]
        print(f"将只处理指定的类别: {target_category}")
    else:
        # 否则，处理所有类别
        categories = [d for d in source_path.iterdir() if d.is_dir() and not d.name.startswith('.')]
        print("将处理所有类别。")


    for category_path in tqdm(categories, desc="处理所有类别"):
        category_name = category_path.name
        tqdm.write(f"--- 开始处理类别: {category_name} ---")

        parquet_files = sorted(list(category_path.glob('*.parquet')))
        if not parquet_files:
            tqdm.write(f"在 '{category_name}' 中未找到Parquet文件，跳过。")
            continue

        # 步骤1: 低内存占用方式收集所有 episode IDs
        tqdm.write("正在收集所有 episode IDs...")
        all_episode_ids = set()
        for f in tqdm(parquet_files, desc=f"扫描 {category_name} IDs", leave=False):
            ids_in_file = pd.read_parquet(f, columns=['id'])['id'].unique()
            all_episode_ids.update(ids_in_file)
        
        all_episode_ids = np.array(list(all_episode_ids))
        np.random.shuffle(all_episode_ids)

        # 步骤2: 划分训练集和测试集 IDs
        if category_name in ['move', 'pass']:
            test_split_ratio = 0.05
        else:
            test_split_ratio = default_test_split_ratio

        num_test_episodes = int(len(all_episode_ids) * test_split_ratio)
        test_episode_ids = set(all_episode_ids[:num_test_episodes])
        train_episode_ids = set(all_episode_ids[num_test_episodes:])
        
        tqdm.write(f"总 episodes: {len(all_episode_ids)}")
        tqdm.write(f"训练集 episodes: {len(train_episode_ids)}")
        tqdm.write(f"测试集 episodes: {len(test_episode_ids)} (比例: {test_split_ratio:.0%})")

        # 步骤3: 分块处理和保存
        train_output_path = dest_path / category_name / 'train'
        test_output_path = dest_path / category_name / 'test'
        train_output_path.mkdir(parents=True, exist_ok=True)
        test_output_path.mkdir(parents=True, exist_ok=True)

        buffers = {'train': [], 'test': []}
        buffered_episode_counts = {'train': 0, 'test': 0}
        file_counters = {'train': 0, 'test': 0}

        tqdm.write("正在分块处理并保存文件...")
        for f in tqdm(parquet_files, desc=f"处理 {category_name} 文件", leave=False):
            df_chunk = pd.read_parquet(f)
            unique_ids_in_chunk = df_chunk['id'].unique()

            for episode_id in unique_ids_in_chunk:
                episode_df = df_chunk[df_chunk['id'] == episode_id]
                
                if episode_id in train_episode_ids:
                    split = 'train'
                elif episode_id in test_episode_ids:
                    split = 'test'
                else:
                    continue

                buffers[split].append(episode_df)
                buffered_episode_counts[split] += 1

                if buffered_episode_counts[split] >= episodes_per_file:
                    save_and_clear_buffer(buffers[split], locals()[f"{split}_output_path"], category_name, file_counters)
                    buffered_episode_counts[split] = 0
        
        # 保存剩余的缓冲区数据
        save_and_clear_buffer(buffers['train'], train_output_path, category_name, file_counters)
        save_and_clear_buffer(buffers['test'], test_output_path, category_name, file_counters)
        
        tqdm.write(f"--- 完成处理类别: {category_name} ---\n")

if __name__ == '__main__':
    # 定义源数据和目标数据路径
    source_base_dir = 'vla/uav_flow_raw_categorized'
    dest_base_dir = 'vla/uav_flow_raw_shuffled'
    
    # --- 控制变量 ---
    # 设置此变量以仅处理特定类别 (例如 'surround')。
    # 如果要处理所有类别，请将其设置为 None。
    CATEGORY_TO_PROCESS = 'surround'

    print(f"源目录: {source_base_dir}")
    print(f"目标目录: {dest_base_dir}")
    
    # 在运行前，建议先手动删除目标目录中对应的子目录
    if CATEGORY_TO_PROCESS:
        print(f"警告: 准备重新生成 '{dest_base_dir}/{CATEGORY_TO_PROCESS}'。请确保已备份或可以安全覆盖。")

    process_uav_dataset(
        source_base_dir, 
        dest_base_dir, 
        target_category=CATEGORY_TO_PROCESS
    )
    
    print("所有数据处理完成！")