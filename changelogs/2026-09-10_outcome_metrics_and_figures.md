# 2026-09-10 结局口径落地：数据整合、分析脚本、表格生成、图 5–10 与论文改写

## 修改概述
按用户 2026-09-10 决定（碰撞率分母取全 50 集；FSM 不补跑；论文改用到达率、停滞率、成功率口径并引入非负速度约束说明），完成本地全套工作：把本地重跑批 B 的 UDMC 数据并入正式数据集、补跑被 NameError 截断的 8 个 GRIPC 各臂混合场景格子、分析脚本加入结局分类、表 1/表 2 改为脚本生成、图 5–10 重出、论文 §3/§5.1/§5.2/§5.3/§5.4/摘要/结论改写并新增 6 篇经核实的参考文献。

## 修改详情
### 数据
- `experiment_results/v6_pedfix_50r/`：旧 UDMC 200 集（服务器批，越界提前结束口径）归档至 `v6_pedfix_50r_superseded_2026-09-10/UDMC/`；并入 `rerun_b_local/` 的 119 集到达 run 与 81 集 `.failed.txt` 停滞 run 及其驱动日志；`failed.tsv` 去掉旧 UDMC 行并追加 81 条 `stall` 记录。
- 被 NameError 截断的 8 格（GRIPC r5、noBSP r2、noOBCA r2/r8、noIGPR r2/r21/r8、noQP r21，均混合场景）旧文件归档至 `…superseded_2026-09-10/truncated_mixed/`，用 `tools/rerun_cells_local.sh` 按原臂开关重跑，8 格全部正常到达。
### 代码
- `analyze_v6_pedfix.py`：新增结局分类（arrived / stall / passby / timeout）、`load_failed_tsv`、`outcome_summary`、`print_outcome_tables`；summary.csv 新增 n_stall、n_passby、n_timeout、arrival_rate、stall_rate、success_rate、collision_rate_all50、travel_time_success 等列；连续指标只在到达 run 上统计；配对检验二值指标（碰撞、到达、成功）在全部有数据的 run 上配对。OCP 臂旧数据中持续 ≥93 s 的 run 判为 stall（审计证实全部为停滞后异常中断）；FSM 混合场景判为 passby。
- `tools/make_paper_tables.py`（新增）：由 summary.csv 生成表 1/表 2 的 LaTeX 行并按规则加粗，避免手抄。
- `tools/rerun_cells_local.sh`（新增）：指定格子本地补跑器，开关映射与 `run_experiments.sh::run_udmc` 一致，停滞异常判失败。
- `tools/fig_capture_final.sh`（新增）：干净全景底图（`FIG_OVERVIEW_LAYER=0`）与补丁版 UDMC 个例采集批。
- `utils/carla_utils.py`：`_draw_overview_layer` 增加 `FIG_OVERVIEW_LAYER=0` 开关。
- `paper_figures/make_carla_figures.py`（子代理实施，主 AI 审核）：图 5 叠加自车实际轨迹并优先使用干净全景 `OVERVIEW_<scenario>`；图 6 混合行固定帧 1.9/3.4/7.1/25.4 s；图 7 点线改为两方法共用的原始参考线且在样条自交时整图不画（环岛接缝回折处自交，实际不画）；图 8 删除求解时间面板与 2 m 线；默认混合种子改为 84。
- `paper_figures/make_result_figures.py`：碰撞率分母改为 50；新增到达率面板；通行时间只取成功 run；偏差与最小间距只取到达 run；图 9 版面改为碰撞率 / 到达率 / 成功 run 通行时间 / 偏差，去掉耗时 ECDF。
### 论文（不入 git）
- §3：式 (eq:ctrl-limits) 后加 $v_{x,\min}=0$ 与两篇依据（ref58、ref59）。
- §5.1：结局定义（到达 5 m 且剩余路点 ≤10 或队列耗尽；停滞 90 s 无进展并对照 Leaderboard/Bench2Drive 180 s 与 NoCrash 8 s；超时；成功）、各指标分母、UDMC 基线保真度句扩写（允许倒车与负参考速度的官方实现、统一非负速度约束）。
- 表 1：列改为到达/Coll./成功/偏差/通行时间(成功)/jerk/耗时均值/P95，脚本生成；表 2：加四场景到达率列，脚本生成。
- §5.2：开篇、安全性、新增停滞段（引冻结机器人问题 ref60/ref61）、精度（重算的远近拆分）、效率（成功 run 口径与幸存者效应）、实时性（99.96%，45/91748 步）。
- §5.3：四个模块段落改为到达率/停滞率叙述与到达 run 统计；§5.4 机理句改写。
- 摘要、结论：UDMC 数字改为停滞率，jerk 四场景最低，实时率 99.96%。
- 图 5–9 图注同步。
- `latex/cas-refs.bib` 新增 ref58–ref63（Eiras 2022 T-RO、Pan 2023 T-ITS、Trautman 2010 IROS、Villaflor 2024 ICRA、Codevilla 2019 ICCV、CARLA Leaderboard 评测页），均由子代理经 CrossRef 与原文核实。

## 修改原因
用户 2026-09-10 决定：按全 50 集算碰撞率、不补跑 FSM、采用到达率/停滞率口径、把防倒车约束写入论文并借此展开停滞率讨论。

## 验证结果
- `analyze_v6_pedfix.py`：样本量核对无缺口（每格 50 = 到达 + 停滞 + 超时/掠过），有数据 run 共 1767；结局表与 `summary.csv` 一致；`tools/audit_rerun_b.py` 对 UDMC 200 格无异常结束。
- `tools/make_paper_tables.py` 生成的表 1（20 行）与表 2（5 行）已替换入稿；数字与 summary.csv 逐格一致。
- 补跑 8 格：GRIPC r5、noBSP r2、noOBCA r2/r8、noIGPR r2/r21/r8、noQP r21 全部以 `Reached destination` 到达（GRIPC r5 第一次尝试因本机负载 300 s 超时无产物，第二次成功）。
- 图 9/10 由 `make_result_figures.py` 重出（碰撞率分母 50；到达率面板；成功 run 通行时间；FSM 混合无成功 run 不绘制）；图 5 用 `FIG_OVERVIEW_LAYER=0` 采集的四张干净全景底图叠加 GRIPC 实际轨迹，无折线与烘焙终点框；图 6 混合行改用 1.9/3.4/7.1/25.4 s 帧，行人穿越可见；图 7 点线因原始样条在环岛接缝回折处自交而整图删除；图 8 改为 4 面板。
- 论文 xelatex + bibtex + xelatex×2：0 error、26 页、无未定义引用；表 1 以 `\resizebox{\textwidth}{!}` 收进版心后无 overfull；其余 overfull 均为改动前已有的小量（≤11.6 pt）与模板 \maketitle 一处。
- 实时率核实：GRIPC 到达 run 91748 步中 45 步超过 50 ms，实时率 99.96%，摘要/§5.2/结论均改为该值（原"精确 100%"不成立）。
- 远近拆分在到达 run 上重算：环岛远离交通 GRIPC 0.167 m 对 w/o B-spline 0.105 m，贴近 6 m 内 0.261 m 对 w/o OBCA 0.191 m；旧稿 0.043 m 等数字受停滞 run 拉低，已更正。
- 未完成项：图 7/8 仍用未打补丁 UDMC 的旧采集（终点前 18 m 结束）。补丁版 UDMC 重录在本机高负载（用户另一组实验，负载 30 到 39）下两次失败（环岛 60 卡死超时、交叉口 83 连接超时），换 seed 51 重试时 GRIPC 在采集里碰撞，判定高负载下采集不可用；`tools/fig_capture_run.sh` 加 `FIG_CARLA_WAIT` 环境变量，待负载降低后重录并重出图 7/8。

## 关联场景
四场景全部方法。
