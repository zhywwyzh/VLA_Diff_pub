import os
import json
import numpy as np
import glob
import shutil
from tqdm import tqdm

def calculate_std(min_val, max_val):
    """计算从 min_val 到 max_val 的等差数列的标准差"""
    if min_val > max_val:
        return 0.0
    arr = np.arange(min_val, max_val + 1)
    return np.std(arr)

def merge_meta_and_data():
    """
    合并所有子类别下 train 文件夹中的 meta 数据和 data parquet 文件。
    """
    source_base_dir = "/data/vla/uav_flow_test_lxx/parts/"
    target_base_dir = "/data/vla/uav_flow_test_lxx/merged/train/uav_flow"
    target_meta_dir = os.path.join(target_base_dir, "meta")
    target_data_dir = os.path.join(target_base_dir, "data")

    # 确保目标目录存在
    os.makedirs(target_meta_dir, exist_ok=True)
    os.makedirs(target_data_dir, exist_ok=True)

    # 初始化全局追踪变量
    final_info = {
        "codebase_version": "v2.1",
        "robot_type": "uav",
        "total_episodes": 0,
        "total_frames": 0,
        "total_tasks": 0,
        "total_videos": 0,
        "total_chunks": 0,
        "chunks_size": 1000, # 更新为新的 chunk 大小
        "fps": 5,
        "splits": {},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {}
    }
    
    final_tasks = []
    final_episodes = []
    final_episodes_stats = []

    current_task_index_offset = 0
    current_episode_index_offset = 0
    last_global_frame_max = -1

    # 查找所有要处理的 meta 文件夹
    # 定义类别和 train 文件夹的排序规则
    category_order = [
        "approach", "ascend_descend", "land", "move", "pass", 
        "retreat", "rotate", "shift", "surround", "turn"
    ]
    category_order_map = {name: i for i, name in enumerate(category_order)}

    all_meta_dirs = glob.glob(os.path.join(source_base_dir, "*", "train*", "uav_flow", "meta"))

    def sort_key(path):
        parts = path.split(os.sep)
        # 假设路径结构是 .../uav_flow_lerobot_3w_categorized/category/trainX/uav_flow/meta
        category = parts[-4]
        train_dir = parts[-3]
        
        category_index = category_order_map.get(category, len(category_order))
        try:
            # 从 "train1", "train2" 等中提取数字
            train_number = int(train_dir.replace('train', ''))
        except (ValueError, IndexError):
            train_number = -1
        return category_index, train_number

    # 按类别和 train 文件夹排序，以确保处理顺序稳定
    meta_dirs = sorted(all_meta_dirs, key=sort_key)

    if not meta_dirs:
        print("错误：未找到任何 meta 文件夹。请检查源目录结构。")
        return

    print(f"找到 {len(meta_dirs)} 个 meta 文件夹进行合并。")

    # 【修改】: 移除第一个大循环，将 Parquet 处理逻辑合并到下面的元数据循环中
    # --- 元数据与 Parquet 文件合并逻辑 ---
    with tqdm(total=len(meta_dirs), desc="合并元数据和 Parquet 文件") as pbar:
        for meta_dir in meta_dirs:
            pbar.set_postfix_str(os.path.basename(os.path.dirname(os.path.dirname(meta_dir))))

            info_path = os.path.join(meta_dir, "info.json")
            tasks_path = os.path.join(meta_dir, "tasks.jsonl")
            episodes_path = os.path.join(meta_dir, "episodes.jsonl")
            episodes_stats_path = os.path.join(meta_dir, "episodes_stats.jsonl")

            # 检查所有文件是否存在
            if not all(os.path.exists(p) for p in [info_path, tasks_path, episodes_path, episodes_stats_path]):
                print(f"警告：跳过 {meta_dir}，因为缺少一个或多个 meta 文件。")
                pbar.update(1)
                continue

            # 1. 读取和处理 info.json
            with open(info_path, 'r') as f:
                current_info = json.load(f)
            
            final_info["total_episodes"] += current_info.get("total_episodes", 0)
            final_info["total_frames"] += current_info.get("total_frames", 0)
            final_info["total_tasks"] += current_info.get("total_tasks", 0)
            if not final_info["features"]: # 仅在第一次时复制 features
                final_info["features"] = current_info.get("features", {})

            # 2. 处理 tasks.jsonl
            local_task_index_map = {}
            with open(tasks_path, 'r') as f:
                for line in f:
                    task_obj = json.loads(line)
                    original_task_index = task_obj["task_index"]
                    new_task_index = original_task_index + current_task_index_offset
                    local_task_index_map[original_task_index] = new_task_index
                    task_obj["task_index"] = new_task_index
                    final_tasks.append(task_obj)
            
            # 3. 处理 episodes.jsonl 和 episodes_stats.jsonl
            local_episode_task_map = {}
            with open(episodes_path, 'r') as f_ep, open(tasks_path, 'r') as f_tasks:
                episodes_lines = f_ep.readlines()
                tasks_data = [json.loads(line) for line in f_tasks]
                task_desc_to_index = {t['task']: t['task_index'] for t in tasks_data}

            for ep_line in episodes_lines:
                ep_obj = json.loads(ep_line)
                original_ep_index = ep_obj['episode_index']
                task_description = ep_obj['tasks'][0]
                original_task_idx = task_desc_to_index.get(task_description)
                if original_task_idx is not None:
                    local_episode_task_map[original_ep_index] = original_task_idx
                
                # 【修改】: 在这里处理对应的 Parquet 文件
                source_data_dir = os.path.join(meta_dir, "..", "data")
                source_parquet_files = glob.glob(os.path.join(source_data_dir, "chunk-*", f"episode_{original_ep_index:06d}.parquet"))
                
                if not source_parquet_files:
                    print(f"\n警告：找不到 episode {original_ep_index} 对应的 Parquet 文件，跳过。")
                    continue
                
                source_parquet_path = source_parquet_files[0]
                new_episode_index = original_ep_index + current_episode_index_offset
                
                new_chunk_index = new_episode_index // 1000
                target_chunk_dir = os.path.join(target_data_dir, f"chunk-{new_chunk_index:03d}")
                os.makedirs(target_chunk_dir, exist_ok=True)
                
                target_parquet_path = os.path.join(target_chunk_dir, f"episode_{new_episode_index:06d}.parquet")
                shutil.copy2(source_parquet_path, target_parquet_path)

                ep_obj["episode_index"] = new_episode_index
                final_episodes.append(ep_obj)

            # 最后处理 episodes_stats.jsonl
            with open(episodes_stats_path, 'r') as f:
                for line in f:
                    stats_obj = json.loads(line)
                    original_episode_index = stats_obj["episode_index"]
                    
                    new_episode_index = original_episode_index + current_episode_index_offset
                    stats_obj["episode_index"] = new_episode_index
                    stats_obj["stats"]["episode_index"]["min"] = [new_episode_index]
                    stats_obj["stats"]["episode_index"]["max"] = [new_episode_index]
                    stats_obj["stats"]["episode_index"]["mean"] = [float(new_episode_index)]

                    count = stats_obj["stats"]["index"]["count"][0]
                    new_min_index = last_global_frame_max + 1
                    new_max_index = new_min_index + count - 1
                    stats_obj["stats"]["index"]["min"] = [new_min_index]
                    stats_obj["stats"]["index"]["max"] = [new_max_index]
                    stats_obj["stats"]["index"]["mean"] = [round((new_min_index + new_max_index) / 2.0, 1)]
                    stats_obj["stats"]["index"]["std"] = [calculate_std(new_min_index, new_max_index)]
                    last_global_frame_max = new_max_index

                    original_task_idx = local_episode_task_map.get(original_episode_index)
                    if original_task_idx is not None:
                        new_task_idx = local_task_index_map.get(original_task_idx)
                        if new_task_idx is not None:
                            stats_obj["stats"]["task_index"]["min"] = [new_task_idx]
                            stats_obj["stats"]["task_index"]["max"] = [new_task_idx]
                            stats_obj["stats"]["task_index"]["mean"] = [float(new_task_idx)]
                    
                    final_episodes_stats.append(stats_obj)

            # 更新偏移量以备下一个文件夹使用
            if final_tasks:
                current_task_index_offset = final_tasks[-1]["task_index"] + 1
            if final_episodes:
                current_episode_index_offset = final_episodes[-1]["episode_index"] + 1
            
            pbar.update(1)

    # 写入最终合并的元数据文件
    final_info["total_chunks"] = (final_info["total_episodes"] + 999) // 1000
    final_info_path = os.path.join(target_meta_dir, "info.json")
    with open(final_info_path, 'w') as f:
        json.dump(final_info, f, indent=4)

    final_tasks_path = os.path.join(target_meta_dir, "tasks.jsonl")
    with open(final_tasks_path, 'w') as f:
        for task in final_tasks:
            f.write(json.dumps(task) + '\n')

    final_episodes_path = os.path.join(target_meta_dir, "episodes.jsonl")
    with open(final_episodes_path, 'w') as f:
        for episode in final_episodes:
            f.write(json.dumps(episode) + '\n')

    final_episodes_stats_path = os.path.join(target_meta_dir, "episodes_stats.jsonl")
    with open(final_episodes_stats_path, 'w') as f:
        for stats in final_episodes_stats:
            f.write(json.dumps(stats) + '\n')

    print("\n元数据合并完成！")
    print(f"所有文件已保存至: {target_base_dir}")
    print(f"总任务数: {final_info['total_tasks']}")
    print(f"总 Episodes 数: {final_info['total_episodes']}")
    print(f"总帧数: {final_info['total_frames']}")

if __name__ == '__main__':
    merge_meta_and_data()