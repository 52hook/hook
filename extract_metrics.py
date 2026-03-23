#!/usr/bin/env python3
"""
从仿真日志中提取对比实验指标。

用法:
    python extract_metrics.py unsig_s1.txt
    python extract_metrics.py unsig_s2.txt
    python extract_metrics.py unsig_s3.txt

日志列格式 (来自 udmc_main.py, agent._log_data):
  [0] simu_time
  [1] dist_error
  [2] yaw_error
  [3] vel_error
  [4] acc         (经 env.step 映射后的加速度)
  [5] steer       (经 env.step 映射后的转向)
  [6] apf_obs     (障碍物势场)
  [7] apf_ttc     (TTC 势场)
  [8] apf_pede    (行人势场)
  [9] apf_nc_road (不可逾越车道线势场)
  [10] apf_c_road  (可逾越车道线势场)
  [11] apf_traffic (红绿灯势场)
  [12] cost_time   (MPC 求解时间)
"""
import sys
import numpy as np


def extract_metrics(log_file: str):
    """从 .txt 日志中提取关键指标。"""
    data = np.loadtxt(log_file)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    dt = float(data[1, 0] - data[0, 0]) if len(data) > 1 else 0.05
    total_time = float(data[-1, 0])

    # ---- 加速度相关 ----
    acc = data[:, 4]
    steer = data[:, 5]

    # jerk = |da/dt|
    jerk = np.abs(np.diff(acc)) / dt
    max_jerk = float(np.max(jerk)) if len(jerk) > 0 else 0.0
    mean_jerk = float(np.mean(jerk)) if len(jerk) > 0 else 0.0

    # steering rate = |dδ/dt|
    steer_rate = np.abs(np.diff(steer)) / dt
    max_steer_rate = float(np.max(steer_rate)) if len(steer_rate) > 0 else 0.0
    mean_steer_rate = float(np.mean(steer_rate)) if len(steer_rate) > 0 else 0.0

    # ---- 跟踪误差 ----
    mean_dist_err = float(np.mean(data[:, 1]))
    mean_yaw_err = float(np.mean(data[:, 2]))
    mean_vel_err = float(np.mean(data[:, 3]))

    # ---- MPC 求解时间 ----
    mpc_times = data[:, 12]
    mean_mpc = float(np.mean(mpc_times))
    max_mpc = float(np.max(mpc_times))
    p95_mpc = float(np.percentile(mpc_times, 95))

    # ---- APF 激活统计 ----
    apf_obs_active = float(np.sum(data[:, 6] > 1e-3) * dt)
    apf_ttc_active = float(np.sum(data[:, 7] > 1e-3) * dt)

    print(f"{'='*60}")
    print(f"  指标提取结果: {log_file}")
    print(f"{'='*60}")
    print(f"  总仿真时间:          {total_time:.2f} s")
    print(f"  时间步长 dt:         {dt:.4f} s")
    print(f"  仿真步数:            {len(data)}")
    print()
    print(f"  --- 舒适性指标 ---")
    print(f"  最大 jerk:           {max_jerk:.3f} m/s³")
    print(f"  平均 jerk:           {mean_jerk:.3f} m/s³")
    print(f"  最大 steering rate:  {max_steer_rate:.4f} rad/s")
    print(f"  平均 steering rate:  {mean_steer_rate:.4f} rad/s")
    print()
    print(f"  --- 跟踪精度 ---")
    print(f"  平均位置误差:        {mean_dist_err:.4f} m")
    print(f"  平均航向误差:        {mean_yaw_err:.4f} rad")
    print(f"  平均速度误差:        {mean_vel_err:.4f} m/s")
    print()
    print(f"  --- MPC 实时性 ---")
    print(f"  平均求解时间:        {mean_mpc*1000:.2f} ms")
    print(f"  最大求解时间:        {max_mpc*1000:.2f} ms")
    print(f"  95th 分位求解时间:   {p95_mpc*1000:.2f} ms")
    print(f"  实时率 (dt={dt*1000:.0f}ms):  {mean_mpc/dt*100:.1f}%")
    print()
    print(f"  --- APF 势场激活时长 ---")
    print(f"  障碍物 APF 激活:     {apf_obs_active:.2f} s")
    print(f"  TTC APF 激活:        {apf_ttc_active:.2f} s")
    print(f"{'='*60}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python extract_metrics.py <日志文件.txt>")
        print("示例: python extract_metrics.py unsig_s1.txt")
        sys.exit(1)

    for f in sys.argv[1:]:
        try:
            extract_metrics(f)
        except Exception as e:
            print(f"处理 {f} 时出错: {e}")
