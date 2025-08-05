import pandas as pd
import numpy as np

def compare_parquet_files(file1_path, file2_path):
    """
    比较两个 Parquet 文件的结构，包括列名、数据类型以及第一行数据的具体类型。

    Args:
        file1_path (str): 第一个 Parquet 文件的路径。
        file2_path (str): 第二个 Parquet 文件的路径。
    """
    print(f"--- 正在加载文件 ---")
    print(f"文件 1: {file1_path}")
    print(f"文件 2: {file2_path}\n")

    try:
        df1 = pd.read_parquet(file1_path)
        df2 = pd.read_parquet(file2_path)
    except Exception as e:
        print(f"加载 Parquet 文件时出错: {e}")
        return

    print("--- 列名比较 ---")
    cols1 = set(df1.columns)
    cols2 = set(df2.columns)

    if cols1 == cols2:
        print("两个文件的列名完全相同。\n")
        common_columns = sorted(list(cols1))
    else:
        print("文件列名存在差异:")
        only_in_1 = cols1 - cols2
        if only_in_1:
            print(f"  - 仅存在于文件 1 的列: {sorted(list(only_in_1))}")
        only_in_2 = cols2 - cols1
        if only_in_2:
            print(f"  - 仅存在于文件 2 的列: {sorted(list(only_in_2))}")
        common_columns = sorted(list(cols1.intersection(cols2)))
        print(f"  - 共同的列: {common_columns}\n")


    print("--- 数据类型 (Dtype) 比较 ---")
    print(f"{'列名':<20} | {'文件 1 Dtype':<15} | {'文件 2 Dtype':<15} | {'是否匹配':<10}")
    print("-" * 70)
    for col in common_columns:
        dtype1 = df1.dtypes[col]
        dtype2 = df2.dtypes[col]
        match = "✅" if dtype1 == dtype2 else "❌"
        print(f"{col:<20} | {str(dtype1):<15} | {str(dtype2):<15} | {match:<10}")
    print("\n")

    print("--- 第一行数据内容类型比较 ---")
    if df1.empty or df2.empty:
        print("至少一个文件为空，无法比较第一行数据。")
        return
        
    row1 = df1.iloc[0]
    row2 = df2.iloc[0]
    print(f"{'列名':<20} | {'文件 1 数据类型':<30} | {'文件 2 数据类型':<30} | {'是否匹配':<10}")
    print("-" * 100)
    for col in common_columns:
        type1 = type(row1[col])
        type2 = type(row2[col])
        
        # 特殊处理 numpy array，使其显示更清晰
        if isinstance(row1[col], np.ndarray):
            type1_str = f"numpy.ndarray(shape={row1[col].shape}, dtype={row1[col].dtype})"
        else:
            type1_str = str(type1)

        if isinstance(row2[col], np.ndarray):
            type2_str = f"numpy.ndarray(shape={row2[col].shape}, dtype={row2[col].dtype})"
        else:
            type2_str = str(type2)

        match = "✅" if type1 == type2 else "❌"
        print(f"{col:<20} | {type1_str:<30} | {type2_str:<30} | {match:<10}")


if __name__ == "__main__":
    # 定义要比较的两个 Parquet 文件路径
    file1 = '/data/vla/uav_flow_test/lerobot/uav_flow/data/chunk-000/episode_000000.parquet'
    file2 = '/data/vla/uav_flow_lerobot_3w_final/train/uav_flow/data/chunk-000/episode_000000.parquet'
    
    compare_parquet_files(file1, file2)