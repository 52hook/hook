# 2026-09-03 横向误差口径统一补丁 + 混合场景协议对齐 + 服务器重跑批 A

## 修改概述
1. 原版 UDMC 入口（udmc_carla-main）第 15 列 dist_error_raw 由「复制第 2 列」改为「到本步参考线采样点集 (cx, cy, ds=0.4 m) 的最近欧氏距离」，与主线 GRIPC 家族第 15 列口径一致。仅写日志，不进入任何控制量。
2. 三个基线入口（fsm_main / frenet_main / tcp_main）mixed_traffic 块：目标车速 25 → 35 km/h，信号灯冻红 → 冻绿，与 udmc_main.py 对齐（差异原样继承自上游 UDMC 仓库）。
3. analyze_v6_pedfix.py：rmse_unified 不再按方法分流，全部方法取第 14 列（0-based）最近点横向误差，缺失时回退第 1 列。
4. 新增 rerun_a_40901.sh：服务器批脚本，UDMC 原版 4 场景 × 50 seed + FRENET 混合 × 50 seed，1-attempt、failed.tsv 留痕、CARLA 自管只杀本端口进程、断点续跑。

## 修改详情
- udmc_carla-main/scripts/x_v2x_agent.py ~L519：`dist_error_raw = dist_error` → `min(hypot(cx - x_next, cy - y_next))`（try/except 回退），列注释同步。
- fsm_main(baseline).py L117/L122、frenet_main(baseline).py L114/L119、tcp_main(baseline).py L135/L140：Red → Green，25 → 35。
- analyze_v6_pedfix.py L42、L58–65：删除 RMSE_RAW_METHODS 分流，统一取 rmse_dist_err_raw。
- rerun_a_40901.sh（新建）。

## 修改原因
- 日志第 2 列 dist_error 是自车下一步预测状态到时间索引参考点的距离，含纵向滞后（停车让行时可达 v_ref·Δt ≈ 0.55 m）；第 15 列才是几何最近点横向误差。原分析器对 GRIPC 家族用第 15 列、对 UDMC 与 w/o B-spline 用第 2 列，表 1/表 2 的 RMSE 口径不一致（详见会话记录：GRIPC 0.149/0.182/0.145/0.135 vs w/o B-spline 第 15 列 0.099/0.098/0.124/0.099）。
- 混合场景协议：GRIPC/UDMC 入口 35 km/h 冻绿，基线入口 25 km/h 冻红。经本机 CARLA 实测四条路线，混合/多车道/环岛路线上均无信号灯，灯态差异无实际影响；FSM 的 target_v 仅为备用值、TCP 的 target_v 只进速度误差指标，均不需重跑；FRENET 的 target_v 直接决定 v_cmd，需重跑。

## 验证
- bash -n / py_compile 通过；mixed 块 grep 复核三文件均已改；本地无仿真验证（在服务器批中验证：新 UDMC 日志第 15 列须与第 2 列不同且量级 0.1–0.3 m）。

## 关联场景
全部四场景（UDMC 重跑）、mixed_traffic（FRENET 重跑）。

## 备注
FSM/TCP 的 mixed 块改动不触发重跑（行为不受影响）。论文文件不入 git。
