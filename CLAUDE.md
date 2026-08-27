每次都叫我 SCI大王

## 0. 硬约束

- **修改前必须先备份到 GitHub（强制，无例外）**：每次修改代码或论文文件前，必须先 `git add` + `git commit` + `git push`，确保用户可随时回退；严禁在未 push 的情况下开始任何文件修改；若无未提交更改则跳过。
- **严禁凭记忆生成 BibTeX**——未经联网核实的引用一律标 `[CITATION NEEDED]`。
- **专业问题必须验证**：回答学术规范、技术细节、算法原理等专业问题前，必须通过联网搜索或本地文件验证，禁止凭 AI 记忆直接作答，不确定的内容标注不确定。
- **改文件前先出计划**：输出 3–7 步计划 + 文件清单 + 风险 + 验证方式，等用户说"开始"后再动手。
- **（最严格要求）复杂任务默认多 Agent 并行**：涉及多文件修改、跨模块调研、论文+代码联动等复杂任务时，默认启用多 Agent 并发执行，引入方法，需要更改代码必须先利用agent调研顶刊论文，做到有依据。
- **学术问题必须先读论文**：回答任何与本项目相关的学术问题前，必须先读 latex/cas-dc-template.tex 及相关章节，基于论文实际内容，然后进行顶刊论文的搜索与阅读回答，严禁凭印象或通用知识敷衍作答。
- **`CLAUDE.md` 为唯一项目规则文件**，不在其他地方重复定义项目规则。
- **联网必须用 Sonnet 5 最高推理等级执行（强制，无例外）**：所有联网操作（搜索、抓取网页、查文献）必须委派给 **Sonnet 5（`claude-sonnet-5`）子 Agent**，并将其推理等级设为**最高（`max`）**后再执行；主 Agent 不直接联网。子 Agent 内部可使用 WebSearch / WebFetch 等联网工具。
- **论文内容严禁进入 git（强制，无例外）**：涉及论文的文件（`*.tex`/`*.bib`/`*.docx`/`*.pdf`/`latex/` 目录/`changelogs/` 中论文相关日志等）**严禁 `git add`、严禁 `git commit`、严禁 `git push`、严禁任何形式的本地"备份 commit"**，必须始终保留在工作区未跟踪或未暂存状态。原因：若论文与代码混在同一条提交历史上，未来代码回退（`git reset`/`revert`）会把论文一起拖回旧版本，造成写作成果丢失。论文的历史管理由用户手动负责，AI 不介入，改论文前也不需要任何 git 备份动作，直接改就行。只有代码改动（`scripts/`/`utils/`/`*.py`/`*.sh`/`CLAUDE.md` 等）才可 `commit` + `push`。
- **重要回答须 codex 事后审 + 自审双复核（2026-07-30 改，取代 2026-07-26 的 Fable 条款）**：重要问题/建议，以及**一切代码改动完成后**，必须交 **codex `codex-auto-review` + 最高推理等级 `xhigh`** 做规格级/diff 级事后审；拿到报告后自己再审一遍，两关都过才可输出。评审提出阻塞项必须改后重审，**禁止跳过或伪造结论**。执行顺序：出计划 → 备份 → 改文件 → **codex 事后审 → 自审** → 输出。
  - **固定调用式（`-s read-only` 不可省——审查者禁改被审对象）**：
    ```
    codex exec -m codex-auto-review -c model_reasoning_effort='"xhigh"' -s read-only \
      "<短 prompt>" < /dev/null > <落盘文件> 2>&1
    ```
    随后**单独一行取 `$?`**。注意 codex **默认 `workspace-write`**，不加 `-s read-only` 它可以改工作区文件。
  - **① 首要纪律：prompt 必须写明「你必须自己直接完成，禁止派生任何子 Agent、禁止并行委派、禁止加载任何 skill、禁止读取 CLAUDE.md」**。
    **这是断流的真因（2026-07-30 对照实验实证，推翻 2026-07-27 的"长 prompt 必断"误诊）**：
    codex 会读本项目 `CLAUDE.md`，看到「复杂任务默认多 Agent 并行」后宣告要派子 Agent，**会话即死在那一步**
    ——输出停在一句开场白、**无 `tokens used` 页脚**、退出码仍为 0。
    对照实验：同一 `xhigh`、同一文件面，仅加这句禁令 ⇒ 从 16 行开场白变成 757 行完整分析 + 结论。
    **与 prompt 长度、推理等级（`high`/`xhigh` 均断）、文件面大小（收窄到 765 行仍断）无关。**
  - **其余调用纪律**：② **必须限定读取文件面**（prompt 里明确列出允许读哪几个文件；大文件先 `sed` 出摘录另存，别让它 dump 8000 行）；③ **必须 `< /dev/null` 闭 stdin**；④ 长任务用 `nohup` + 后台，勿前台干等；⑤ **必须落盘再读**；⑥ 待审内容写进文件，prompt 只给路径 + 审什么 + 输出格式。
  - **判活三条（缺一即视为"复核未完成"，不得采信）**：
    ① **落盘文件里必须有 `tokens used` 页脚**——这是会话正常结束的机械信号；**缺页脚即断流**，
    比"看有没有裁定段"更硬（断流时退出码仍是 0，且开场白常常长得像在干活）；
    ② **退出码必须单独取，严禁走管道**——`codex exec | tail` 的 `$?` 是 `tail` 的，会把 503 伪装成 completed；
    ③ 断流或 503 时**如实回报**并按纪律①重发，仍不通才改走备用通路，**严禁把断流当 APPROVE**。
  - **Fable 5 降为备用通路**：codex 连续断流 / 503 时改派 **Fable 5 子 Agent**（`Agent` 工具，`model: fable`，**只读、禁改文件、禁 git 写**）顶上，并在交付里写明**本次实际走的是哪条通路**。
- **主 AI/执行器分工（2026-07-26 改，强制）**：主 AI 只负责**写计划、任务拆解、调用 subagent、diff 级审核**；**主要执行（代码实现/大规模文件修改）默认按解耦边界拆成多个 subagent 并行实施**（`Agent` 工具；同一文件禁止并发写，跨组重叠文件须用 worktree 隔离后由主 AI 合并）。执行完由**主 AI 做 diff 级审核**，再交 **codex 事后审**（`codex-auto-review` + `xhigh` + `-s read-only`，纪律见上条），两关都过才算完成，问题打回原组重做。**谁执行、对方审——禁止同一 Agent 自实施自放行。**
  - **合并纪律（2026-07-26 事故 I1 后加，强制）**：**一旦执行过合并，所有后续返工一律在主树进行**，给任何执行器的返工指令**必须写明工作区**；**合并不是一次性动作**，每轮返工后都要重新核对"主树 == 执行器实际改动处"；**核对交付前必须先确认核对的是执行器真正写入的那棵树**——在错误的树上 grep 得到的"零命中"是**假阴性**，会把正确交付误判为未完成。
- **跨组契约测试为发批前置门（2026-07-26 定，强制）**：涉及多组并行实施时，必须有一个**用真实生产者产出、喂真实消费者校验**的常驻测试（本项目为 `tests_r5_crossgroup.py`）；**任何跨组契约改动，不跑通它不许发批**。
  - **依据**：R5 一轮实施中出现**七个跨组断裂**（B 样条永不生效致 noBSP≡GRIPC、弯道集全 invalid、分析器第一集即死、安全否决被静默弃权等），**全部发生在"每组自测都绿"的状态下**——单测 21/21、runner 自测 150/150、分析器自检 PASS 同时存在。
  - **根因**：**每组的自测都在证明"我按我以为的契约实现了"，没有任何一组在证明"契约两边一致"**。消费侧的 fixture 测不出生产侧的真实输出（类型/符号/位置/键名），反之亦然。
  - **该测试须含两道**：**动态**（真实 `arm_env` → 真实 exporter → 真实 validator）与**静态**（消费侧必需键全集 vs 生产侧键差集，抓"要某键但全库无生产者"——动态测试若 fixture 恰好补齐会漏掉这一类）。

## 1. 语言与风格

- 思考语言用专业流英语，交互语言用中文，代码注释用中文 + ASCII 风格分块注释，使代码看起来像高度优化的顶级开源库作品。
- 核心信念：代码是写给人看的，只是顺便让机器运行。
- 文档标题与说明全中文，变量名/函数名/文件名保留英文。
- Git Commit 格式 `类型: 描述`，描述 ≤ 15 字全中文，类型：`新增`/`修复`/`优化`/`重构`/`移除`。

## 2. 项目概述

**GRIPC**（Geometry-Rule Integrated Planning and Control）— 面向城市自动驾驶的几何-规则集成规划与控制框架，基于 CARLA 0.9.16，在 UDMC 基础上做四项改进。

本人论文：latex/cas-dc-template.tex。原版 UDMC 代码在 `udmc_carla-main/`，对应论文为 `UDMC_Unified_Decision-Making_and_Control_Framework_for_Urban_Autonomous_Driving_With_Motion_Prediction_of_Traffic_Participants.pdf`（根目录）。OBCA 改进参考论文为 `TDR-OBCA_A_Reliable_Planner_for_Autonomous_Driving_in_Free-Space_Environment.pdf`（根目录）。

> ## ⚠ 本节（§2 / §4）正在被 R5 统一修复重构，以下为**改造前的现状**
> **权威目标态见 `changelogs/2026-07-26_190000_unified_fix_spec_R5_v7_FINAL.md` + `..._230000_..._v7_1_PATCH.md` 合并版**（冲突以 v7.1 为准）。目标态要点：
> - **贡献收敛为三项**：①时序化对偶 OBCA　②R3-A/B/C 三修（种子/逐行/卫生门）　③B-spline 全局参考线平滑。
>   **④QP 热启动已被否决**（H-OBCA 2018 已有 argmax 对偶播种，三方确认）⇒ 降为工程实现细节。
>   **IGPR σ 消费移除**（R5-0）⇒ 不再是创新点。
> - **8 臂 × 4 场景 × 40 seed = 1280 集**：`GRIPC` / `UDMC-orig` / `FSM` / `GRIPC_noOBCA` / `GRIPC_noBSP` / `GRIPC_noR3A` / `GRIPC_noR3B` / `GRIPC_noR3C`。**`noIGPR` 与 `noQP` 两臂已退役。**
> - 新增消融开关 `R3A_SEED` / `R3B_ROWS` / `R3C_HYGIENE`（默认全 1）；`ABLATION_IGPR` 删净。
> - 日志三入口统一 **21 列**（新增 col20 `lat_err_ref`）；seeds 改 400–439；正式批在**远程服务器 40901** 跑、`RESTART_POLICY=block`。
>
> 代码尚未落地前，下方描述仍是仓库现状；R5-12 落地时本节整体改写。

### 四大改进（2026-07-21 重构：四创新与四消融一一对应，顺序对齐论文引入顺序）

1. **B-spline 全局参考线平滑** — QP 约束优化，C² 连续（`utils/global_bspline_smoother.py`）｜消融 `ABLATION_BSPLINE`→noBSP 臂
2. **IGPR 方差驱动 APF 权重** — 预测不确定性高时增大安全裕度（`IGPR_predict_sv_wps.py` + `scripts/x_v2x_agent.py`）｜消融 `ABLATION_IGPR`→noIGPR 臂
3. **OBCA 多边形硬约束** — 替代椭圆近似（`utils/geometry_poly.py`）｜消融 `USE_DUAL`→noNLP_POLY 臂
4. **QP 热启动** — 凸 QP 预规划为主 NLP 提供初值（`utils/hybrid_qp_warmstart.py`）｜消融 `ABLATION_QP`→noQP_INIT 臂

**注**：对向车让行（`scripts/x_v2x_agent.py` 内对向 APF 权重调度，`ABLATION_YIELD` 开关存在、默认开）**不是论文创新点**——属工程实现细节，论文不提；其逻辑在所有 GRIPC 入口臂均匀存在（UDMC 基线走原版入口天然没有），不影响四创新的消融归因。

### 消融开关

`USE_DUAL`=1（OBCA）、`ABLATION_IGPR`=1（IGPR）、`ABLATION_BSPLINE`=1（B-spline）、`ABLATION_QP`=1（QP 热启动），均在 `udmc_main.py:31-37`；`RANDOM_SEED` 控制随机交通种子（`udmc_main.py:40`）。

## 3. CARLA 环境

本仓库固定 CARLA 0.9.16，Python 环境 `conda activate first1`，CARLA 目录 `~/CARLA_0.9.16`，默认端口 2001。修改 `scripts/` 前提醒用户确认 CARLA 已启动。

**注意**：`requirements.txt` 中写的是 `carla==0.9.14`，这是历史遗留，实际运行环境为 0.9.16，不要降级。

## 4. 实验设计

四个场景：`unsig_crossroad`（无信号路口左转）、`roundabout`（环岛汇入）、`multilaneACC`（多车道巡航）、`mixed_traffic`（车辆+行人混合），均 100 辆随机车。

三个方法：GRIPC（四开关全开）、UDMC（四开关全关）、FSM（`fsm_main(baseline).py`）。

调试用场景（不纳入正式实验）：`unsig_s1`/`unsig_s2`/`unsig_s3`/`round_s1`/`cross_s1`/`crossroad`。

实验矩阵：对比实验 4×3×40=480 次 + 消融实验 4×4×40=640 次 = 总计 1120 次（`run_experiments.sh`）。
**（R5 目标态：8 臂 × 4 场景 × 40 seed = 1280 次，seeds 400–439，服务器 40901 + `RESTART_POLICY=block`，见 §2 顶部提示框。）**


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

## 10. 调试提示

调试问题时应主动读取 `logs/` 目录下的运行日志（空格分隔浮点数，**现为 20 列，R5 后为 21 列**，列定义见 `extract_metrics.py`）辅助分析。列编号口径：规格文档一律 **0-based**（= `data[:,N]`），代码注释为 1-based，差 1。

## 11. 目录结构速查

- `scripts/` — 核心算法（决策 Agent、动力学模型、环境封装）
- `utils/` — 工具模块（OBCA、B-spline、QP 热启动、绘图）
- `official/` — 修改后的 CARLA 官方示例（basic_agent、behavior_agent、spawn_npc 等）
- `spawnPoints/` — 固定出生点配置（CSV 格式）
- `udmc_carla-main/` — 原始 UDMC 代码（对照参考用）
- `changelogs/` — 维护日志
- `data/` — 仿真输出数据（时间戳目录）
- `logs/` — 自动生成的运行日志
