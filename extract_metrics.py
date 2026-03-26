#!/usr/bin/env python3
"""
从仿真日志中提取实验指标，支持单文件和批量处理。

用法:
    # 单文件
    python extract_metrics.py experiment_results/GRIPC_crossroad_r1.txt

    # 批量: 提取某方法在某场景的所有重复，输出均值±标准差
    python extract_metrics.py --batch experiment_results/ --method GRIPC --scenario crossroad

    # 批量: 提取所有方法、所有场景，输出汇总表
    python extract_metrics.py --batch experiment_results/ --summary

日志列格式 (来自 udmc_main.py, agent._log_data):
  [0] simu_time
  [1] dist_error      (横向跟踪误差)
  [2] yaw_error       (航向误差)
  [3] vel_error       (速度误差)
  [4] acc             (经 env.step 映射后的纵向加速度)
  [5] steer           (经 env.step 映射后的转向角)
  [6] apf_obs         (障碍物势场)
  [7] apf_ttc         (TTC 势场)
  [8] apf_pede        (行人势场)
  [9] apf_nc_road     (不可逾越车道线势场)
  [10] apf_c_road     (可逾越车道线势场)
  [11] apf_traffic    (红绿灯势场)
  [12] cost_time      (MPC 求解时间, 单位: 秒)
  [13] min_dist_to_obs (与最近障碍车辆的距离, 单位: 米)
"""
import sys
import os
import re
import argparse
import numpy as np
from pathlib import Path
from collections import defaultdict


def extract_single(log_file: str) -> dict:
    """从单个 .txt 日志中提取全部指标，返回字典。"""
    data = np.loadtxt(log_file)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    dt = float(data[1, 0] - data[0, 0]) if len(data) > 1 else 0.05
    total_time = float(data[-1, 0])
    n_steps = len(data)

    # ---- 纵向加速度 & jerk ----
    acc = data[:, 4]
    jerk = np.abs(np.diff(acc)) / dt
    max_jerk = float(np.max(jerk)) if len(jerk) > 0 else 0.0
    mean_jerk = float(np.mean(jerk)) if len(jerk) > 0 else 0.0

    # ---- 转向角 & 转向速率 ----
    steer = data[:, 5]
    steer_rate = np.abs(np.diff(steer)) / dt
    max_steer_rate = float(np.max(steer_rate)) if len(steer_rate) > 0 else 0.0
    mean_steer_rate = float(np.mean(steer_rate)) if len(steer_rate) > 0 else 0.0

    # ---- 横向加速度近似: a_lat ≈ v² × tan(δ) / L ----
    # 使用 vel_error 近似速度: v ≈ v_ref - vel_error（v_ref 取场景默认值）
    # 简化: 直接用 steer × speed 的量纲近似横向加速度
    # 更准确的方式需要在 agent 中记录实际速度，此处用 |steer × acc| 作为代理指标
    # TODO: 在 agent 中记录实际速度后替换为精确计算
    max_lat_acc = float(np.max(np.abs(steer) * np.abs(acc)))
    mean_lat_acc = float(np.mean(np.abs(steer) * np.abs(acc)))

    # ---- 跟踪误差 ----
    dist_err = data[:, 1]
    mean_dist_err = float(np.mean(np.abs(dist_err)))
    rmse_dist_err = float(np.sqrt(np.mean(dist_err ** 2)))
    mean_yaw_err = float(np.mean(np.abs(data[:, 2])))
    mean_vel_err = float(np.mean(np.abs(data[:, 3])))

    # ---- MPC 求解时间 ----
    mpc_times = data[:, 12]
    mean_mpc = float(np.mean(mpc_times))
    max_mpc = float(np.max(mpc_times))
    p95_mpc = float(np.percentile(mpc_times, 95))
    # 实时可行率: 在 dt 内完成求解的比例
    realtime_rate = float(np.sum(mpc_times <= dt) / n_steps)

    # ---- 与障碍物最小距离（第 14 列，可能不存在于旧日志） ----
    if data.shape[1] > 13:
        min_dists = data[:, 13]
        # 过滤掉 999.0（无障碍物标记）
        valid_dists = min_dists[min_dists < 900]
        if len(valid_dists) > 0:
            min_dist_overall = float(np.min(valid_dists))
            mean_min_dist = float(np.mean(valid_dists))
        else:
            min_dist_overall = 999.0
            mean_min_dist = 999.0
    else:
        min_dist_overall = -1.0  # 旧日志无此列
        mean_min_dist = -1.0

    # ---- APF 激活统计 ----
    apf_obs_time = float(np.sum(data[:, 6] > 1e-3) * dt)
    apf_ttc_time = float(np.sum(data[:, 7] > 1e-3) * dt)

    return {
        'total_time': total_time,
        'n_steps': n_steps,
        'dt': dt,
        # 舒适性
        'max_jerk': max_jerk,
        'mean_jerk': mean_jerk,
        'max_steer_rate': max_steer_rate,
        'mean_steer_rate': mean_steer_rate,
        'max_lat_acc': max_lat_acc,
        'mean_lat_acc': mean_lat_acc,
        # 跟踪精度
        'mean_dist_err': mean_dist_err,
        'rmse_dist_err': rmse_dist_err,
        'mean_yaw_err': mean_yaw_err,
        'mean_vel_err': mean_vel_err,
        # MPC 实时性
        'mean_mpc_ms': mean_mpc * 1000,
        'max_mpc_ms': max_mpc * 1000,
        'p95_mpc_ms': p95_mpc * 1000,
        'realtime_rate': realtime_rate,
        # APF
        'apf_obs_time': apf_obs_time,
        'apf_ttc_time': apf_ttc_time,
        # 与障碍物最小距离
        'min_dist_overall': min_dist_overall,
        'mean_min_dist': mean_min_dist,
    }


def print_single(log_file: str, metrics: dict):
    """打印单文件指标。"""
    m = metrics
    print(f"{'=' * 60}")
    print(f"  指标提取结果: {log_file}")
    print(f"{'=' * 60}")
    print(f"  总仿真时间:          {m['total_time']:.2f} s")
    print(f"  时间步长 dt:         {m['dt']:.4f} s")
    print(f"  仿真步数:            {m['n_steps']}")
    print()
    print(f"  --- 舒适性指标 ---")
    print(f"  最大 jerk:           {m['max_jerk']:.3f} m/s³")
    print(f"  平均 jerk:           {m['mean_jerk']:.3f} m/s³")
    print(f"  最大 steering rate:  {m['max_steer_rate']:.4f} rad/s")
    print(f"  平均 steering rate:  {m['mean_steer_rate']:.4f} rad/s")
    print(f"  最大 lat. acc (近似): {m['max_lat_acc']:.4f}")
    print(f"  平均 lat. acc (近似): {m['mean_lat_acc']:.4f}")
    print()
    print(f"  --- 跟踪精度 ---")
    print(f"  平均位置误差:        {m['mean_dist_err']:.4f} m")
    print(f"  RMSE 位置误差:       {m['rmse_dist_err']:.4f} m")
    print(f"  平均航向误差:        {m['mean_yaw_err']:.4f} rad")
    print(f"  平均速度误差:        {m['mean_vel_err']:.4f} m/s")
    print()
    print(f"  --- MPC 实时性 ---")
    print(f"  平均求解时间:        {m['mean_mpc_ms']:.2f} ms")
    print(f"  最大求解时间:        {m['max_mpc_ms']:.2f} ms")
    print(f"  95th 分位求解时间:   {m['p95_mpc_ms']:.2f} ms")
    print(f"  实时可行率:          {m['realtime_rate'] * 100:.1f}%")
    print()
    print(f"  --- 与障碍物距离 ---")
    if m['min_dist_overall'] > 0:
        print(f"  最小距离:            {m['min_dist_overall']:.2f} m")
        print(f"  平均最近距离:        {m['mean_min_dist']:.2f} m")
    else:
        print(f"  (旧日志，无 min_dist 列)")
    print()
    print(f"  --- APF 势场激活时长 ---")
    print(f"  障碍物 APF 激活:     {m['apf_obs_time']:.2f} s")
    print(f"  TTC APF 激活:        {m['apf_ttc_time']:.2f} s")
    print(f"{'=' * 60}")


def parse_filename(filename: str):
    """从文件名中解析 method, scenario, repeat。
    格式: {METHOD}_{SCENARIO}_r{N}.txt
    """
    name = Path(filename).stem
    match = re.match(r'^(.+?)_(.+?)_r(\d+)$', name)
    if match:
        return match.group(1), match.group(2), int(match.group(3))
    return None, None, None


def batch_summary(results_dir: str):
    """批量处理实验结果目录，输出汇总表。"""
    results_dir = Path(results_dir)
    # 按 (method, scenario) 分组
    grouped = defaultdict(list)

    txt_files = sorted(results_dir.glob('*.txt'))
    if not txt_files:
        print(f"未找到 .txt 日志文件: {results_dir}")
        return

    for f in txt_files:
        method, scenario, repeat = parse_filename(f.name)
        if method is None:
            continue
        try:
            m = extract_single(str(f))
            grouped[(method, scenario)].append(m)
        except Exception as e:
            print(f"  跳过 {f.name}: {e}")

    if not grouped:
        print("未解析到有效结果。")
        return

    # 按场景和方法排序
    methods_order = ['GRIPC', 'UDMC', 'FSM', 'GRIPC_noOBCA', 'GRIPC_noIGPR', 'GRIPC_noBSP', 'GRIPC_noQP']
    scenarios = sorted(set(s for _, s in grouped.keys()))

    # 要输出的关键指标
    key_metrics = [
        ('total_time', 'Travel(s)', '.1f'),
        ('max_jerk', 'MaxJerk', '.2f'),
        ('max_steer_rate', 'MaxStR', '.3f'),
        ('rmse_dist_err', 'RMSE(m)', '.3f'),
        ('min_dist_overall', 'MinDist', '.1f'),
        ('mean_mpc_ms', 'MPC(ms)', '.1f'),
        ('p95_mpc_ms', 'P95(ms)', '.1f'),
    ]

    # 打印表头
    header = f"{'Method':<16} {'Scenario':<18} {'N':>3}"
    for _, label, _ in key_metrics:
        header += f" {label:>12}"
    print(header)
    print('-' * len(header))

    for method in methods_order:
        for scenario in scenarios:
            key = (method, scenario)
            if key not in grouped:
                continue
            runs = grouped[key]
            n = len(runs)
            row = f"{method:<16} {scenario:<18} {n:>3}"
            for metric_key, _, fmt in key_metrics:
                vals = [r[metric_key] for r in runs]
                if metric_key == 'realtime_rate':
                    # 显示百分比
                    mean_val = np.mean(vals) * 100
                    std_val = np.std(vals) * 100
                else:
                    mean_val = np.mean(vals)
                    std_val = np.std(vals)
                row += f" {mean_val:{fmt}}±{std_val:{fmt}}"
            print(row)
        if any((method, s) in grouped for s in scenarios):
            print()  # 方法之间空一行


def main():
    parser = argparse.ArgumentParser(description='GRIPC 实验指标提取工具')
    parser.add_argument('files', nargs='*', help='日志文件路径（单文件模式）')
    parser.add_argument('--batch', type=str, help='批量处理: 指定实验结果目录')
    parser.add_argument('--summary', action='store_true', help='输出汇总表')
    parser.add_argument('--method', type=str, help='过滤方法名（与 --batch 配合）')
    parser.add_argument('--scenario', type=str, help='过滤场景名（与 --batch 配合）')

    args = parser.parse_args()

    if args.batch:
        batch_summary(args.batch)
    elif args.files:
        for f in args.files:
            try:
                m = extract_single(f)
                print_single(f, m)
            except Exception as e:
                print(f"处理 {f} 时出错: {e}")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
