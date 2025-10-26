#!/usr/bin/env python3
"""
统计所有task的episode数量

这个程序会扫描所有parquet文件，统计每个task有多少个episodes，
并计算总的episode数量。

Usage:
python count_all_task_episodes.py --data_dir /data/vla/uav_flow_raw_categorized
"""

import os
import json
import argparse
from pathlib import Path
from collections import defaultdict

import pandas as pd


def count_task_episodes(data_dir: str):
    """统计所有task的episode数量"""
    print("开始统计所有task的episode数量...")
    
    # 找到所有parquet文件
    files = sorted(Path(data_dir).rglob("*.parquet"))
    print(f"找到 {len(files)} 个parquet文件")
    
    # 统计每个task的episodes
    task_episode_counts = defaultdict(int)
    total_episodes = 0
    valid_episodes = 0
    
    for file_idx, file_path in enumerate(files):
        if (file_idx + 1) % 20 == 0:
            print(f"处理文件 {file_idx+1}/{len(files)}: {file_path.name}")
        
        try:
            # 读取文件
            df = pd.read_parquet(file_path, columns=['id', 'frame_idx', 'log'])
            if df.empty:
                continue
            
            # 按episode处理
            for episode_id, episode_df in df.groupby('id'):
                total_episodes += 1
                
                if len(episode_df) < 2:
                    continue
                
                valid_episodes += 1
                
                # 提取task信息
                try:
                    first_row = episode_df.iloc[0]
                    raw_log = first_row['log']
                    common_log = json.loads(raw_log) if isinstance(raw_log, str) else raw_log
                    task_instruction = common_log.get('instruction_unified', '')
                    
                    if task_instruction:
                        task_episode_counts[task_instruction] += 1
                        
                except Exception as e:
                    continue
            
            del df  # 清理内存
            
        except Exception as e:
            print(f"  错误：处理文件 {file_path} 失败: {e}")
            continue
    
    print(f"\n=== 统计结果 ===")
    print(f"总文件数: {len(files)}")
    print(f"总episodes: {total_episodes}")
    print(f"有效episodes (≥2帧): {valid_episodes}")
    print(f"发现unique tasks: {len(task_episode_counts)}")
    
    # 计算总的有效episode数量
    total_task_episodes = sum(task_episode_counts.values())
    print(f"所有task的episodes总数: {total_task_episodes}")
    
    # 按episode数量排序显示
    sorted_tasks = sorted(task_episode_counts.items(), key=lambda x: x[1], reverse=True)
    
    print(f"\n=== 前20个episode最多的tasks ===")
    for i, (task, count) in enumerate(sorted_tasks[:20]):
        print(f"{i+1:2d}. {count:4d} episodes - '{task}'")
    
    print(f"\n=== 后20个episode最少的tasks ===")
    for i, (task, count) in enumerate(sorted_tasks[-20:]):
        print(f"{i+1:2d}. {count:4d} episodes - '{task}'")
    
    # 统计episode数量分布
    episode_distribution = defaultdict(int)
    for task, count in task_episode_counts.items():
        episode_distribution[count] += 1
    
    print(f"\n=== Episode数量分布 ===")
    print(f"{'Episodes':<10} {'Tasks':<8} {'累计Episodes':<12}")
    print("-" * 35)
    
    cumulative_episodes = 0
    for episode_count in sorted(episode_distribution.keys(), reverse=True):
        task_count = episode_distribution[episode_count]
        episodes_for_this_count = episode_count * task_count
        cumulative_episodes += episodes_for_this_count
        
        print(f"{episode_count:<10} {task_count:<8} {episodes_for_this_count:<12}")
        
        # 只显示前15行
        if len([x for x in episode_distribution.keys() if x >= episode_count]) >= 15:
            remaining_counts = len([x for x in episode_distribution.keys() if x < episode_count])
            if remaining_counts > 0:
                remaining_episodes = total_task_episodes - cumulative_episodes
                print(f"...        {remaining_counts:<8} {remaining_episodes:<12}")
            break
    
    print(f"\n验证：累计episodes = {cumulative_episodes}, 应该等于总episodes = {total_task_episodes}")
    
    # 保存详细结果到文件
    output_file = "/data/vla/VLA_Diff/Openpi/examples/uav_flow/all_task_episode_counts.txt"
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("=== 所有Task的Episode统计 ===\n\n")
            f.write(f"总文件数: {len(files)}\n")
            f.write(f"总episodes: {total_episodes}\n")
            f.write(f"有效episodes: {valid_episodes}\n")
            f.write(f"Unique tasks: {len(task_episode_counts)}\n")
            f.write(f"所有task的episodes总数: {total_task_episodes}\n\n")
            
            f.write("所有Tasks的Episode数量 (按数量降序):\n")
            f.write("-" * 80 + "\n")
            for i, (task, count) in enumerate(sorted_tasks):
                f.write(f"{i+1:4d}. {count:4d} episodes - '{task}'\n")
            
            f.write(f"\nEpisode数量分布:\n")
            for episode_count in sorted(episode_distribution.keys(), reverse=True):
                task_count = episode_distribution[episode_count]
                f.write(f"{episode_count} episodes: {task_count} tasks\n")
        
        print(f"\n详细结果已保存到: {output_file}")
        
    except Exception as e:
        print(f"保存结果文件失败: {e}")
    
    # 保存为jsonl格式
    jsonl_file = "/data/vla/VLA_Diff/Openpi/examples/uav_flow/all_task_episode_counts.jsonl"
    try:
        with open(jsonl_file, 'w', encoding='utf-8') as f:
            for task, count in sorted_tasks:
                record = {
                    "task": task,
                    "episode_count": count
                }
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        
        print(f"JSONL格式结果已保存到: {jsonl_file}")
        
    except Exception as e:
        print(f"保存JSONL文件失败: {e}")
    
    return task_episode_counts, total_task_episodes


def main():
    parser = argparse.ArgumentParser(description="统计所有task的episode数量")
    parser.add_argument("--data_dir", 
                       default="/data/vla/uav_flow_raw_categorized",
                       help="数据目录路径")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.data_dir):
        print(f"错误：数据目录不存在: {args.data_dir}")
        return
    
    task_counts, total_episodes = count_task_episodes(args.data_dir)
    
    print(f"\n=== 最终总结 ===")
    print(f"发现 {len(task_counts)} 个unique tasks")
    print(f"总共 {total_episodes} 个有效episodes")
    print(f"平均每个task有 {total_episodes/len(task_counts):.1f} 个episodes")


if __name__ == "__main__":
    main()