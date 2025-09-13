#!/usr/bin/env python3
"""
测试修改后的采样逻辑（不依赖LeRobot）
"""

import os
import json
from pathlib import Path
from collections import defaultdict

import pandas as pd


def test_sampling_logic(data_dir: str, target_counts=[2]):
    """测试采样逻辑"""
    print("开始测试采样逻辑...")
    
    # 找到所有parquet文件
    files = sorted(Path(data_dir).rglob("*.parquet"))
    print(f"找到 {len(files)} 个parquet文件")
    
    # 为每个target count维护独立的收集状态
    sampled_episodes_by_count = {}
    task_counters_by_count = {}  # count -> {task: current_count}
    collected_episodes_by_count = {}  # count -> {task: [episodes]}
    
    for count in target_counts:
        sampled_episodes_by_count[count] = []
        task_counters_by_count[count] = defaultdict(int)
        collected_episodes_by_count[count] = defaultdict(list)
    
    # 统计信息
    total_processed = 0
    unique_tasks_seen = set()
    
    print("开始全局扫描，使用计数器收集episodes...")
    for file_idx, file_path in enumerate(files[:5]):  # 只测试前5个文件
        print(f"扫描文件 {file_idx+1}/5: {file_path.name}")
        
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
                for count in target_counts:
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
                            
                            print(f"    Task '{task_instruction}' 已收集满 {count} episodes")
            
            del df
            
        except Exception as e:
            print(f"  警告：处理文件 {file_path} 失败: {e}")
            continue
    
    # 处理未收集满的tasks（将部分收集的episodes也加入）
    for count in target_counts:
        incomplete_tasks = 0
        for task_instruction, episodes in collected_episodes_by_count[count].items():
            if len(episodes) > 0:
                incomplete_tasks += 1
                print(f"    Task '{task_instruction}' 在 {count}ep数据集中只收集到 {len(episodes)} episodes (目标{count}个)")
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
        
        print(f"\n  {count}ep数据集收集完成:")
        print(f"    总tasks: {total_tasks} tasks")
        print(f"    完整tasks: {complete_tasks} tasks (有{count}个episodes)")
        print(f"    不完整tasks: {incomplete_tasks} tasks (少于{count}个episodes)")
        print(f"    总episodes: {total_episodes}")
        
        # 显示每个task的episode数量
        print(f"    详细分布:")
        for task, episode_count in sorted(task_episode_counts.items(), key=lambda x: x[1], reverse=True)[:10]:
            print(f"      '{task}': {episode_count} episodes")
    
    # 生成task统计
    print(f"\n=== Task统计 ===")
    print(f"总扫描episodes: {total_processed}")
    print(f"发现unique tasks: {len(unique_tasks_seen)}")
    
    # 生成task统计jsonl文件
    task_stats = {}
    for task in unique_tasks_seen:
        task_stats[task] = {}
        for count in target_counts:
            # 计算这个task在这个count数据集中有多少episodes
            task_episodes_in_dataset = [ep for ep in sampled_episodes_by_count[count] 
                                      if ep['task_instruction'] == task]
            task_stats[task][f"{count}ep_dataset"] = len(task_episodes_in_dataset)
    
    # 保存到jsonl文件
    jsonl_path = "/data/vla/VLA_Diff/Openpi/examples/uav_flow/test_task_stats.jsonl"
    try:
        with open(jsonl_path, 'w', encoding='utf-8') as f:
            for task, stats in task_stats.items():
                record = {
                    "task": task,
                    "total_episodes_available": sum(stats.values()) if target_counts else 0,
                    **stats
                }
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        
        print(f"\nTask统计文件已保存到: {jsonl_path}")
        print(f"包含 {len(task_stats)} 个tasks的统计信息")
        
    except Exception as e:
        print(f"保存task统计文件失败: {e}")


if __name__ == "__main__":
    test_sampling_logic("/data/vla/uav_flow_raw_categorized", target_counts=[2])