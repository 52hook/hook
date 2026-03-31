每次都叫我 SCI大王

## 0. 硬约束

- **修改前必须先备份到 GitHub（强制，无例外）**：每次修改代码或论文文件前，必须先 `git add` + `git commit` + `git push`，确保用户可随时回退；严禁在未 push 的情况下开始任何文件修改；若无未提交更改则跳过。
- **严禁凭记忆生成 BibTeX**——未经联网核实的引用一律标 `[CITATION NEEDED]`。
- **专业问题必须验证**：回答学术规范、技术细节、算法原理等专业问题前，必须通过联网搜索或本地文件验证，禁止凭 AI 记忆直接作答，不确定的内容标注不确定。
- **改文件前先出计划**：输出 3–7 步计划 + 文件清单 + 风险 + 验证方式，等用户说"开始"后再动手。
- **复杂任务默认多 Agent 并行**：涉及多文件修改、跨模块调研、论文+代码联动等复杂任务时，默认启用多 Agent 并发执行，简单单文件任务无需启用。
- **`CLAUDE.md` 为唯一项目规则文件**，不在其他地方重复定义项目规则。

## 1. 语言与风格

- 思考语言用专业流英语，交互语言用中文，代码注释用中文 + ASCII 风格分块注释，使代码看起来像高度优化的顶级开源库作品。
- 核心信念：代码是写给人看的，只是顺便让机器运行。
- 文档标题与说明全中文，变量名/函数名/文件名保留英文。
- Git Commit 格式 `类型: 描述`，描述 ≤ 15 字全中文，类型：`新增`/`修复`/`优化`/`重构`/`移除`。

## 2. 项目概述

**GRIPC**（Geometry-Rule Integrated Planning and Control）— 面向城市自动驾驶的几何-规则集成规划与控制框架，基于 CARLA 0.9.16，在 UDMC 基础上做四项改进。

原版代码在 `udmc_carla-main/`，对应论文为 `UDMC_Unified_Decision-Making_and_Control_Framework_for_Urban_Autonomous_Driving_With_Motion_Prediction_of_Traffic_Participants.pdf`（根目录）。

### 四大改进

1. **OBCA 多边形硬约束** — 替代椭圆近似（`utils/geometry_poly.py`）
2. **对向车检测与让行** — 硬编码于 `scripts/x_v2x_agent.py:581-631`，无消融开关
3. **IGPR 方差驱动 APF 权重** — 预测不确定性高时增大安全裕度（`IGPR_predict_sv_wps.py` + `scripts/x_v2x_agent.py:807-839`）
4. **B-spline 全局参考线平滑** — QP 约束优化，C² 连续（`utils/global_bspline_smoother.py`）

### 消融开关

`USE_DUAL`=1（OBCA）、`ABLATION_IGPR`=1（IGPR）、`ABLATION_BSPLINE`=1（B-spline）、`ABLATION_QP`=1（QP 热启动），均在 `udmc_main.py:31-37`；`RANDOM_SEED` 控制随机交通种子（`udmc_main.py:40`）。

## 3. CARLA 环境

本仓库固定 CARLA 0.9.16，Python 环境 `conda activate first1`，CARLA 目录 `~/CARLA_0.9.16`，默认端口 2001。修改 `scripts/` 前提醒用户确认 CARLA 已启动。

**注意**：`requirements.txt` 中写的是 `carla==0.9.14`，这是历史遗留，实际运行环境为 0.9.16，不要降级。

## 4. 实验设计

四个场景：`unsig_crossroad`（无信号路口左转）、`roundabout`（环岛汇入）、`multilaneACC`（多车道巡航）、`mixed_traffic`（车辆+行人混合），均 100 辆随机车。

三个方法：GRIPC（四开关全开）、UDMC（四开关全关）、FSM（`fsm_main(baseline).py`）。

实验矩阵：对比实验 4×3×40=480 次 + 消融实验 4×4×40=640 次 = 总计 1120 次（`run_experiments.sh`）。

调试用场景（不纳入正式实验）：`unsig_s1`/`unsig_s2`/`unsig_s3`/`round_s1`/`cross_s1`/`crossroad`。

## 5. 核心文件

- 入口：`udmc_main.py`（GRIPC 主入口）、`fsm_main(baseline).py`（FSM 基线）、`run_experiments.sh`（批量实验）、`extract_metrics.py`（指标提取）
- 核心算法：`scripts/x_v2x_agent.py`（主控决策）、`scripts/vehicle_obs.py`（动力学+MPC+APF）、`scripts/env.py`（CARLA 封装）、`scripts/others_agent.py`（周围车辆模型）
- 工具：`utils/geometry_poly.py`（OBCA）、`utils/global_bspline_smoother.py`（B-spline）、`utils/hybrid_qp_warmstart.py`（QP 热启动）、`utils/carla_utils.py`、`utils/draw.py`
- 预测：`IGPR_predict_sv_wps.py`（GPR 均值+方差）、`gpr.pkl`/`lstm.pkl`（预训练模型，不可直接编辑）

## 6. 数据文件约定

`.npy` 和 `.pkl` 为二进制文件不直接编辑；实验日志在 `logs/`（自动生成）；正式实验数据在 `experiment_results/`（由 `run_experiments.sh` 生成）。

## 7. 维护日志

每次修改代码前在 `changelogs/` 下创建 `YYYY-MM-DD_HHMMSS.md` 维护日志，包含：修改概述、修改详情（文件+内容+原值→新值）、修改原因、验证结果、关联场景、备注。

## 8. 可用 Workflows

- `/run_sim` — 启动 CARLA 并运行单个仿真场景
- `/run_all_experiments` — 运行全部对比实验和消融实验
- `/git_commit` — 提交代码到 Git 仓库

## 9. 其他根目录工具脚本

- `extract_metrics.py` — 从实验日志批量提取 14 列指标（横向误差、航向误差、速度误差、加速度、jerk、APF 值、MPC 求解时间、最小距离等）
- `carla_nl_system_identification.py` — 车辆动力学参数辨识数据采集
- `param_est_using_slsqp.py` — SLSQP 优化器拟合车辆动力学参数
- `mixed_traffic_scenario_generation.py` — 行人+车辆混合场景生成
- `tele_carla.py` — 遥控驾驶工具

## 10. 日志分析方法

调试问题时应结合 `logs/` 目录下的运行日志辅助分析。日志文件为空格分隔的 14 列浮点数（每行一个仿真步），列定义如下：

| 列号 | 名称 | 含义 |
|------|------|------|
| 0 | time | 仿真时间 (s) |
| 1 | dist_error | 横向跟踪误差 (m) |
| 2 | yaw_error | 航向误差 (rad) |
| 3 | vel_error | 速度误差 (km/h) |
| 4 | acc | 纵向加速度 |
| 5 | steer | 方向盘转角 |
| 6 | apf_obs | 障碍车辆 APF |
| 7 | apf_ttc | TTC 势场 |
| 8 | apf_pede | 行人 APF |
| 9 | apf_nc_road | 不可穿越道路 APF |
| 10 | apf_c_road | 可穿越道路 APF |
| 11 | apf_traffic | 红灯 APF |
| 12 | cost_time | MPC 求解时间 (s) |
| 13 | min_dist_to_obs | 与最近障碍物距离 (m)，来自 CARLA world 全量查询，不受过滤影响 |

用 `extract_metrics.py` 可批量提取统计指标。分析碰撞/别车等问题时重点关注 `min_dist_to_obs`（列13）骤降、`apf_obs`（列7）突变、`steer`（列5）急变。

## 11. 目录结构速查

- `scripts/` — 核心算法（决策 Agent、动力学模型、环境封装）
- `utils/` — 工具模块（OBCA、B-spline、QP 热启动、绘图）
- `official/` — 修改后的 CARLA 官方示例（basic_agent、behavior_agent、spawn_npc 等）
- `spawnPoints/` — 固定出生点配置（CSV 格式）
- `udmc_carla-main/` — 原始 UDMC 代码（对照参考用）
- `changelogs/` — 维护日志
- `data/` — 仿真输出数据（时间戳目录）
- `logs/` — 自动生成的运行日志
