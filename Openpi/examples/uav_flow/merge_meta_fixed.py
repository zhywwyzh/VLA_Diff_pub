import os
import json
import numpy as np
import glob
import shutil
from tqdm import tqdm

def calculate_std(values):
    """计算实际数值列表的标准差，而不是假设等差数列"""
    if len(values) <= 1:
        return 0.0
    return float(np.std(values))

def merge_meta_and_data():
    """
    合并所有子类别下 train 文件夹中的 meta 数据和 data parquet 文件。
    修复版本：解决任务索引映射、帧索引连续性、偏移量更新等问题。
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
        "chunks_size": 1000,
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
    global_frame_index = 0  # 改为连续的全局帧索引计数器

    # 查找所有要处理的 meta 文件夹
    category_order = [
        "approach", "ascend_descend", "land", "move", "pass", 
        "retreat", "rotate", "shift", "surround", "turn"
    ]
    category_order_map = {name: i for i, name in enumerate(category_order)}

    all_meta_dirs = glob.glob(os.path.join(source_base_dir, "*", "train*", "uav_flow", "meta"))

    def sort_key(path):
        parts = path.split(os.sep)
        category = parts[-4]
        train_dir = parts[-3]
        
        category_index = category_order_map.get(category, len(category_order))
        try:
            train_number = int(train_dir.replace('train', ''))
        except (ValueError, IndexError):
            train_number = -1
        return category_index, train_number

    meta_dirs = sorted(all_meta_dirs, key=sort_key)

    if not meta_dirs:
        print("错误：未找到任何 meta 文件夹。请检查源目录结构。")
        return

    print(f"找到 {len(meta_dirs)} 个 meta 文件夹进行合并。")

    # 用于数据一致性检查
    used_episode_indices = set()
    used_task_indices = set()

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
            if not final_info["features"]:
                final_info["features"] = current_info.get("features", {})

            # 2. 处理 tasks.jsonl - 先读取原始数据建立映射
            original_tasks_data = []
            with open(tasks_path, 'r') as f:
                for line in f:
                    original_tasks_data.append(json.loads(line))
            
            # 建立原始任务描述到原始索引的映射
            original_task_desc_to_index = {t['task']: t['task_index'] for t in original_tasks_data}
            
            # 建立原始索引到新索引的映射
            local_task_index_map = {}
            for task_obj in original_tasks_data:
                original_task_index = task_obj["task_index"]
                new_task_index = original_task_index + current_task_index_offset
                
                # 检查任务索引唯一性
                if new_task_index in used_task_indices:
                    print(f"警告：任务索引 {new_task_index} 重复，跳过当前meta目录")
                    break
                
                used_task_indices.add(new_task_index)
                local_task_index_map[original_task_index] = new_task_index
                task_obj["task_index"] = new_task_index
                final_tasks.append(task_obj.copy())
            else:
                # 只有在没有break的情况下才继续处理episodes
                
                # 3. 处理 episodes.jsonl
                local_episode_task_map = {}
                episodes_data = []
                
                with open(episodes_path, 'r') as f:
                    for line in f:
                        ep_obj = json.loads(line)
                        episodes_data.append(ep_obj)
                        
                        original_ep_index = ep_obj['episode_index']
                        task_description = ep_obj['tasks'][0]
                        
                        # 使用原始任务描述到原始索引的映射
                        original_task_idx = original_task_desc_to_index.get(task_description)
                        if original_task_idx is not None:
                            local_episode_task_map[original_ep_index] = original_task_idx

                # 处理每个episode的数据文件和索引
                successfully_processed_episodes = []
                for ep_obj in episodes_data:
                    original_ep_index = ep_obj['episode_index']
                    new_episode_index = original_ep_index + current_episode_index_offset
                    
                    # 检查episode索引唯一性
                    if new_episode_index in used_episode_indices:
                        print(f"警告：Episode索引 {new_episode_index} 重复，跳过")
                        continue
                    
                    # 查找并复制对应的 Parquet 文件
                    source_data_dir = os.path.join(meta_dir, "..", "data")
                    source_parquet_files = glob.glob(os.path.join(source_data_dir, "chunk-*", f"episode_{original_ep_index:06d}.parquet"))
                    
                    if not source_parquet_files:
                        print(f"警告：找不到 episode {original_ep_index} 对应的 Parquet 文件，跳过。")
                        continue
                    
                    source_parquet_path = source_parquet_files[0]
                    new_chunk_index = new_episode_index // 1000
                    target_chunk_dir = os.path.join(target_data_dir, f"chunk-{new_chunk_index:03d}")
                    os.makedirs(target_chunk_dir, exist_ok=True)
                    
                    target_parquet_path = os.path.join(target_chunk_dir, f"episode_{new_episode_index:06d}.parquet")
                    shutil.copy2(source_parquet_path, target_parquet_path)

                    used_episode_indices.add(new_episode_index)
                    ep_obj["episode_index"] = new_episode_index
                    final_episodes.append(ep_obj.copy())
                    successfully_processed_episodes.append((original_ep_index, new_episode_index))

                # 4. 处理 episodes_stats.jsonl
                with open(episodes_stats_path, 'r') as f:
                    for line in f:
                        stats_obj = json.loads(line)
                        original_episode_index = stats_obj["episode_index"]
                        
                        # 只处理成功处理的episodes
                        episode_mapping = None
                        for orig_idx, new_idx in successfully_processed_episodes:
                            if orig_idx == original_episode_index:
                                episode_mapping = (orig_idx, new_idx)
                                break
                        
                        if episode_mapping is None:
                            continue
                            
                        original_ep_idx, new_episode_index = episode_mapping
                        
                        # 更新episode索引相关统计
                        stats_obj["episode_index"] = new_episode_index
                        stats_obj["stats"]["episode_index"]["min"] = [new_episode_index]
                        stats_obj["stats"]["episode_index"]["max"] = [new_episode_index]
                        stats_obj["stats"]["episode_index"]["mean"] = [float(new_episode_index)]

                        # 更新全局帧索引 - 使用连续的全局计数器
                        count = stats_obj["stats"]["index"]["count"][0]
                        frame_indices = list(range(global_frame_index, global_frame_index + count))
                        
                        stats_obj["stats"]["index"]["min"] = [min(frame_indices)]
                        stats_obj["stats"]["index"]["max"] = [max(frame_indices)]
                        stats_obj["stats"]["index"]["mean"] = [round(np.mean(frame_indices), 1)]
                        stats_obj["stats"]["index"]["std"] = [calculate_std(frame_indices)]
                        
                        global_frame_index += count  # 更新全局帧索引计数器

                        # 更新任务索引统计
                        original_task_idx = local_episode_task_map.get(original_ep_idx)
                        if original_task_idx is not None:
                            new_task_idx = local_task_index_map.get(original_task_idx)
                            if new_task_idx is not None:
                                stats_obj["stats"]["task_index"]["min"] = [new_task_idx]
                                stats_obj["stats"]["task_index"]["max"] = [new_task_idx]
                                stats_obj["stats"]["task_index"]["mean"] = [float(new_task_idx)]
                        
                        final_episodes_stats.append(stats_obj)

                # 更新偏移量 - 基于实际处理的数据
                if final_tasks:
                    current_task_index_offset = max(t["task_index"] for t in final_tasks) + 1
                if final_episodes:
                    current_episode_index_offset = max(e["episode_index"] for e in final_episodes) + 1
            
            pbar.update(1)

    # 数据一致性检查
    print(f"\n数据一致性检查：")
    print(f"处理的任务数: {len(final_tasks)}")
    print(f"处理的episodes数: {len(final_episodes)}")
    print(f"处理的统计记录数: {len(final_episodes_stats)}")
    print(f"全局帧索引范围: 0 到 {global_frame_index - 1}")

    # 写入最终合并的元数据文件
    final_info["total_chunks"] = (len(final_episodes) + 999) // 1000
    final_info["total_episodes"] = len(final_episodes)
    final_info["total_tasks"] = len(final_tasks)
    final_info["total_frames"] = global_frame_index
    
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
