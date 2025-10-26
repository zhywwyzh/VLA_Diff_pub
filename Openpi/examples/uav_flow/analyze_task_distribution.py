#!/usr/bin/env python3
"""
统计UAV-Flow数据中所有task的episode分布

这个脚本会扫描所有parquet文件，统计每个task有多少个episodes，
帮助理解为什么采样结果与预期不符。

Usage:
python analyze_task_distribution.py --data_dir /data/vla/uav_flow_raw_categorized
"""

import os
import json
import argparse
from pathlib import Path
from collections import defaultdict, Counter

import pandas as pd


def analyze_task_distribution(data_dir: str):
    """分析task的episode分布"""
    print("开始分析task分布...")
    
    # 找到所有parquet文件
    files = sorted(Path(data_dir).rglob("*.parquet"))
    print(f"找到 {len(files)} 个parquet文件")
    
    # 统计每个task的episodes
    task_episodes = defaultdict(list)  # task -> [episode_info1, episode_info2, ...]
    total_episodes_processed = 0
    files_processed = 0
    
    for file_idx, file_path in enumerate(files):
        if (file_idx + 1) % 10 == 0:
            print(f"处理文件 {file_idx+1}/{len(files)}: {file_path.name}")
        
        try:
            # 读取文件
            df = pd.read_parquet(file_path, columns=['id', 'frame_idx', 'log'])
            if df.empty:
                continue
            
            files_processed += 1
            
            # 按episode处理
            for episode_id, episode_df in df.groupby('id'):
                total_episodes_processed += 1
                
                if len(episode_df) < 2:
                    continue
                
                # 提取task信息
                try:
                    first_row = episode_df.iloc[0]
                    raw_log = first_row['log']
                    common_log = json.loads(raw_log) if isinstance(raw_log, str) else raw_log
                    task_instruction = common_log.get('instruction_unified', '')
                    
                    if not task_instruction:
                        continue
                        
                except Exception as e:
                    print(f"  警告：Episode {episode_id} 日志解析失败: {e}")
                    continue
                
                # 记录episode信息
                episode_info = {
                    'file_path': str(file_path.name),
                    'episode_id': episode_id,
                    'frame_count': len(episode_df)
                }
                task_episodes[task_instruction].append(episode_info)
            
            del df  # 清理内存
            
        except Exception as e:
            print(f"  错误：处理文件 {file_path} 失败: {e}")
            continue
    
    print(f"\n=== 统计完成 ===")
    print(f"处理的文件: {files_processed}/{len(files)}")
    print(f"总episodes: {total_episodes_processed}")
    print(f"发现unique tasks: {len(task_episodes)}")
    
    # 生成详细统计
    print(f"\n=== Task Episode分布 ===")
    
    # 按episode数量排序
    task_counts = [(task, len(episodes)) for task, episodes in task_episodes.items()]
    task_counts.sort(key=lambda x: x[1], reverse=True)
    
    print(f"前20个episode最多的tasks:")
    for i, (task, count) in enumerate(task_counts[:20]):
        print(f"  {i+1:2d}. {count:3d} episodes - '{task}'")
    
    print(f"\n后20个episode最少的tasks:")
    for i, (task, count) in enumerate(task_counts[-20:]):
        print(f"  {i+1:2d}. {count:3d} episodes - '{task}'")
    
    # 按episode数量分组统计
    episode_count_distribution = Counter(count for _, count in task_counts)
    
    print(f"\n=== Episode数量分布统计 ===")
    print(f"{'Episodes':<10} {'Tasks':<8} {'累计Tasks':<10} {'百分比':<8}")
    print("-" * 40)
    
    cumulative_tasks = 0
    total_tasks = len(task_episodes)
    
    for episode_count in sorted(episode_count_distribution.keys(), reverse=True):
        task_count = episode_count_distribution[episode_count]
        cumulative_tasks += task_count
        percentage = (cumulative_tasks / total_tasks) * 100
        
        print(f"{episode_count:<10} {task_count:<8} {cumulative_tasks:<10} {percentage:<8.1f}%")
        
        # 只显示前15行，避免输出太长
        if len([x for x in episode_count_distribution.keys() if x >= episode_count]) >= 15:
            remaining = len([x for x in episode_count_distribution.keys() if x < episode_count])
            if remaining > 0:
                print(f"... 还有 {remaining} 个不同的episode数量级别")
            break
    
    # 针对常见的target counts进行分析
    target_counts = [1, 2, 4, 8]
    print(f"\n=== 不同数据集规模的可用Tasks分析 ===")
    
    for count in target_counts:
        eligible_tasks = [task for task, episodes in task_episodes.items() if len(episodes) >= count]
        eligible_count = len(eligible_tasks)
        total_episodes = eligible_count * count
        
        print(f"{count}ep数据集:")
        print(f"  可用tasks: {eligible_count}/{total_tasks} ({eligible_count/total_tasks*100:.1f}%)")
        print(f"  总episodes: {total_episodes}")
        
        if count <= 4:  # 只显示前几个的详细信息
            insufficient_tasks = [task for task, episodes in task_episodes.items() if len(episodes) < count]
            print(f"  不足{count}个episodes的tasks: {len(insufficient_tasks)}")
    
    # 保存详细结果到文件
    output_file = "/data/vla/VLA_Diff/Openpi/examples/uav_flow/task_distribution_analysis.txt"
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("=== UAV-Flow Task Episode分布详细分析 ===\n\n")
            f.write(f"总文件数: {len(files)}\n")
            f.write(f"处理的文件: {files_processed}\n")
            f.write(f"总episodes: {total_episodes_processed}\n")
            f.write(f"Unique tasks: {len(task_episodes)}\n\n")
            
            f.write("所有Tasks的Episode数量 (按数量降序):\n")
            f.write("-" * 80 + "\n")
            for i, (task, count) in enumerate(task_counts):
                f.write(f"{i+1:4d}. {count:3d} episodes - '{task}'\n")
            
            f.write(f"\n按Episode数量分组:\n")
            for episode_count in sorted(episode_count_distribution.keys(), reverse=True):
                task_count = episode_count_distribution[episode_count]
                f.write(f"{episode_count} episodes: {task_count} tasks\n")
        
        print(f"\n详细分析结果已保存到: {output_file}")
        
    except Exception as e:
        print(f"保存结果文件失败: {e}")
    
    return task_episodes, task_counts, episode_count_distribution


def main():
    parser = argparse.ArgumentParser(description="分析UAV-Flow数据中task的episode分布")
    parser.add_argument("--data_dir", required=True, help="数据目录路径")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.data_dir):
        print(f"错误：数据目录不存在: {args.data_dir}")
        return
    
    task_episodes, task_counts, episode_count_distribution = analyze_task_distribution(args.data_dir)
    
    print(f"\n=== 快速总结 ===")
    print(f"如果你想要每个task都有至少2个episodes的2ep数据集：")
    eligible_for_2ep = len([task for task, episodes in task_episodes.items() if len(episodes) >= 2])
    print(f"  可用tasks: {eligible_for_2ep}")
    print(f"  预期episodes: {eligible_for_2ep} × 2 = {eligible_for_2ep * 2}")
    
    print(f"\n如果你想要每个task都有至少8个episodes的8ep数据集：")
    eligible_for_8ep = len([task for task, episodes in task_episodes.items() if len(episodes) >= 8])
    print(f"  可用tasks: {eligible_for_8ep}")
    print(f"  预期episodes: {eligible_for_8ep} × 8 = {eligible_for_8ep * 8}")


if __name__ == "__main__":
    main()