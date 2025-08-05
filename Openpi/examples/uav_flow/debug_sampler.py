"""
调试版本：检查数据收集过程中的问题
"""

import os
import json
import logging
import sys
from pathlib import Path
import pandas as pd
import gc

# Configuration
PARQUET_COLUMNS = ['id', 'frame_idx', 'image', 'log']
MAX_FILES_TO_TEST = 5  # 只测试前5个文件

def setup_logger():
    """设置日志"""
    logger = logging.getLogger("debug")
    logger.setLevel(logging.INFO)
    if logger.hasHandlers():
        logger.handlers.clear()
    
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logger.addHandler(handler)
    return logger

def debug_data_collection(data_dir: str):
    """调试数据收集过程"""
    logger = setup_logger()
    
    logger.info(f"开始调试数据收集过程，目录: {data_dir}")
    
    # 找到所有 Parquet 文件
    files = sorted(Path(data_dir).rglob("*.parquet"))
    if not files:
        logger.error(f"在'{data_dir}'中未找到任何parquet文件")
        return
    
    logger.info(f"找到 {len(files)} 个parquet文件")
    
    # 只测试前几个文件
    test_files = files[:MAX_FILES_TO_TEST]
    logger.info(f"测试前 {len(test_files)} 个文件:")
    for f in test_files:
        logger.info(f"  - {f}")
    
    total_episodes = 0
    valid_episodes = 0
    task_samples = {}
    
    for file_idx, file_path in enumerate(test_files):
        logger.info(f"\n处理文件 {file_idx+1}/{len(test_files)}: {file_path.name}")
        
        try:
            # 读取parquet文件
            df = pd.read_parquet(file_path, columns=PARQUET_COLUMNS)
            logger.info(f"  文件大小: {len(df)} 行")
            
            if df.empty:
                logger.warning("  文件为空，跳过")
                continue
            
            # 检查unique episodes
            unique_ids = df['id'].unique()
            logger.info(f"  包含 {len(unique_ids)} 个unique episodes")
            
            # 处理前几个episodes作为样本
            sample_ids = unique_ids[:3] if len(unique_ids) >= 3 else unique_ids
            
            for episode_id in sample_ids:
                total_episodes += 1
                episode_df = df[df['id'] == episode_id]
                rows = episode_df.sort_values('frame_idx').to_dict('records')
                
                logger.info(f"    Episode {episode_id}: {len(rows)} 帧")
                
                if len(rows) < 2:
                    logger.warning(f"    Episode {episode_id}: 帧数不足(<2)，跳过")
                    continue
                
                # 检查log字段
                try:
                    raw_log = rows[0]['log']
                    if isinstance(raw_log, str):
                        common_log = json.loads(raw_log)
                    else:
                        common_log = raw_log
                        
                    if not isinstance(common_log, dict):
                        logger.warning(f"    Episode {episode_id}: log不是dict类型")
                        continue
                        
                    # 检查关键字段
                    instruction = common_log.get('instruction_unified', '')
                    preprocessed_logs = common_log.get('preprocessed_logs')
                    
                    logger.info(f"    Episode {episode_id}:")
                    logger.info(f"      instruction_unified: '{instruction[:50]}...' ({'存在' if instruction else '缺失'})")
                    logger.info(f"      preprocessed_logs: {'存在' if preprocessed_logs else '缺失'} (长度: {len(preprocessed_logs) if preprocessed_logs else 0})")
                    
                    if instruction and preprocessed_logs:
                        valid_episodes += 1
                        # 记录task样本
                        if instruction not in task_samples:
                            task_samples[instruction] = 0
                        task_samples[instruction] += 1
                        logger.info(f"    ✓ Episode {episode_id}: 有效")
                    else:
                        logger.warning(f"    ✗ Episode {episode_id}: 缺少关键字段")
                        
                except Exception as e:
                    logger.warning(f"    Episode {episode_id}: 解析log失败: {e}")
                    continue
            
            # 清理内存
            del df
            gc.collect()
            
        except Exception as e:
            logger.error(f"  处理文件失败: {e}")
            continue
    
    # 统计结果
    logger.info(f"\n=== 调试结果 ===")
    logger.info(f"总episodes检查: {total_episodes}")
    logger.info(f"有效episodes: {valid_episodes}")
    logger.info(f"有效率: {valid_episodes/total_episodes*100:.1f}%" if total_episodes > 0 else "无数据")
    
    logger.info(f"\n发现的tasks:")
    for task, count in sorted(task_samples.items()):
        logger.info(f"  '{task}': {count} 个episodes")
    
    if valid_episodes == 0:
        logger.error("❌ 没有找到任何有效的episodes！")
        logger.error("可能的问题:")
        logger.error("1. instruction_unified字段缺失或为空")
        logger.error("2. preprocessed_logs字段缺失")
        logger.error("3. log字段格式错误")
    else:
        logger.info(f"✅ 找到了 {valid_episodes} 个有效episodes，数据格式正常")

if __name__ == "__main__":
    debug_data_collection("/data/vla/uav_flow_raw_categorized")