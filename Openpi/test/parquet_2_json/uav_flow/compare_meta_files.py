import json
import os
from pathlib import Path
import sys

def pretty_print_dict(d):
    """以易于阅读的格式打印字典。"""
    return json.dumps(d, indent=2, ensure_ascii=False)

def compare_info_json(file1, file2):
    """比较两个 info.json 文件，重点关注 'features' 键。"""
    print(f"--- 比较: {Path(file1).name} ---")
    if not file1.exists() or not file2.exists():
        print(f"错误: 至少一个文件不存在。请检查路径:\n- {file1}\n- {file2}")
        print("-" * 50 + "\n")
        return

    try:
        with open(file1, 'r', encoding='utf-8') as f:
            data1 = json.load(f)
        with open(file2, 'r', encoding='utf-8') as f:
            data2 = json.load(f)
    except Exception as e:
        print(f"错误: 读取或解析JSON文件时出错 - {e}")
        print("-" * 50 + "\n")
        return

    if data1 == data2:
        print("✅ info.json 文件完全相同。")
    else:
        print("❌ info.json 文件存在差异。")
        
        # 比较 'features' 部分，这最可能是问题所在
        features1 = data1.get('features', {})
        features2 = data2.get('features', {})
        
        if features1 == features2:
            print("\n✅ 'features' 部分相同。差异在其他地方。")
            # 您可以在此处添加对其他顶级键的比较
        else:
            print("\n❌ 'features' 部分存在差异。这很可能是导致错误的原因。")
            print("\n--- 来源 1 的 'features' ---")
            print(pretty_print_dict(features1))
            print("\n--- 来源 2 的 'features' ---")
            print(pretty_print_dict(features2))

    print("-" * 50 + "\n")


def compare_jsonl_files(file1, file2):
    """比较两个 .jsonl 文件的行数和第一行结构。"""
    print(f"--- 比较: {Path(file1).name} ---")
    if not file1.exists() or not file2.exists():
        print(f"错误: 至少一个文件不存在。请检查路径:\n- {file1}\n- {file2}")
        print("-" * 50 + "\n")
        return

    try:
        with open(file1, 'r', encoding='utf-8') as f:
            lines1 = f.readlines()
        with open(file2, 'r', encoding='utf-8') as f:
            lines2 = f.readlines()
    except Exception as e:
        print(f"错误: 读取文件时出错 - {e}")
        print("-" * 50 + "\n")
        return

    # 比较行数
    if len(lines1) == len(lines2):
        print(f"✅ 行数相同: {len(lines1)}")
    else:
        print(f"❌ 行数不同: 来源1有 {len(lines1)} 行, 来源2有 {len(lines2)} 行。")

    # 比较第一行的结构
    if not lines1 or not lines2:
        print("至少一个文件为空，无法比较结构。")
        print("-" * 50 + "\n")
        return
        
    try:
        keys1 = set(json.loads(lines1[0]).keys())
        keys2 = set(json.loads(lines2[0]).keys())
    except Exception as e:
        print(f"错误: 解析第一行JSON时出错 - {e}")
        print("-" * 50 + "\n")
        return

    if keys1 == keys2:
        print(f"✅ 第一行的键结构相同: {sorted(list(keys1))}")
    else:
        print("❌ 第一行的键结构不同:")
        if keys1 - keys2:
            print(f"  - 仅在来源1: {sorted(list(keys1 - keys2))}")
        if keys2 - keys1:
            print(f"  - 仅在来源2: {sorted(list(keys2 - keys1))}")
        
    print("-" * 50 + "\n")


def main():
    # 路径1：可以正常工作的元数据
    path1_str = "/data/vla/uav_flow_lerobot_3w_final/train/uav_flow/meta"
    # 路径2：导致错误的元数据
    path2_str = "/data/vla/uav_flow_test/lerobot/uav_flow/meta"

    path1 = Path(path1_str)
    path2 = Path(path2_str)

    if not path1.is_dir() or not path2.is_dir():
        print("错误: 一个或两个目录路径无效。请检查路径。")
        sys.exit(1)
        
    print(f"正在比较目录:\n来源 1 (正常): {path1}\n来源 2 (报错): {path2}\n")

    # 比较 info.json
    compare_info_json(path1 / "info.json", path2 / "info.json")

    # 比较 .jsonl 文件
    jsonl_files = ["tasks.jsonl", "episodes.jsonl", "episodes_stats.jsonl"]
    for filename in jsonl_files:
        compare_jsonl_files(path1 / filename, path2 / filename)

if __name__ == "__main__":
    main()