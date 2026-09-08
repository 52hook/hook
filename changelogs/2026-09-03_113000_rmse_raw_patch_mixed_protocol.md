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

## 2026-09-07 追加：批完成与数据并入
- 服务器批 09-04 00:41–05:12 完成：UDMC 199/200（roundabout r33 因 300 s 墙钟超时按 1-attempt 记失败）、FRENET mixed 50/50。结果拉回 experiment_results/rerun_a_40901/。
- 并入 v6_pedfix_50r：旧 UDMC 全部与 FRENET mixed 文件移入 experiment_results/v6_pedfix_50r_superseded_2026-09-07/；failed.tsv 删除旧 FRENET mixed 3 行、加入 UDMC roundabout r33；重跑 analyze_v6_pedfix.py。
- 新 UDMC 统一口径 RMSE 0.111/0.081/0.160/0.094 m（旧第 1 列 0.573/0.486/0.499/0.534），碰撞 0/20.4/8/4%，通行 34.5/70.1/18.9/66.6 s；FRENET mixed（35 km/h）RMSE 0.514、碰撞 12%（4 车 2 人）、通行 32.4±42.6 s。
- 论文同步：表 1 UDMC 四行与 FRENET 混合行、加粗重排、脚注；表 2 w/o B-spline RMSE 0.440→0.105；5.1 口径定义；5.2 安全/精度/效率/实时四段与摘要、结论中的 RMSE 与 jerk 表述全部按新数据改写；图 9、10 重生成并插入 5.2/5.3。编译 22 页 0 error。

## 2026-09-07 追加：指标改名为「车道中心偏差」并补远近拆分证据（用户选 A 方案）
- 表 1/表 2 列头、图 3(c)/图 4(d) 轴标签、5.1 定义、5.2 精度段、5.3 尾段、摘要、结论统一改为「车道中心偏差」，明确其为车道保持类指标而非对自身参考线的跟踪误差。
- 5.2/5.3 补入按最近周车距离拆分的证据：环岛远离交通时 GRIPC 0.172 m 对 w/o B-spline 0.043 m（参考线切角偏离）；贴近 6 m 内 GRIPC 0.261 m 对 w/o OBCA 0.162 m（硬约束侧向让位）；多车道/混合远离交通时两者仅差 0.01 m。
- 事实澄清：改口径前分析器对 GRIPC 家族用第 14 列（原始车道中心线），对 UDMC 等用第 1 列（时间索引参考点距离，含 0.2–0.75 m 纵向分量与 0.2 m 固定 y 偏置），并非「GRIPC 用平滑参考线」。
- 编译 22 页 0 error。
