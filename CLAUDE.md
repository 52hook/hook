每次都叫我帅哥

## 项目概述

**GRIPC**（Geometry-Rule Integrated Planning and Control）— 面向城市自动驾驶的几何-规则集成规划与控制框架。基于 CARLA 0.9.16 仿真环境，在 UDMC 框架基础上进行四项核心改进。

### 原始 UDMC 参考资料

- **原版代码**：`udmc_carla-main/`（本仓库子目录）
- **对应论文**：`UDMC_Unified_Decision-Making_and_Control_Framework_for_Urban_Autonomous_Driving_With_Motion_Prediction_of_Traffic_Participants.pdf`

### GRIPC 四大核心改进（相对于原始 UDMC）

1. **OBCA 多边形硬约束** — 替代椭圆近似，消除几何保守性（`utils/geometry_poly.py`）
2. **对向车检测与让行** — 解决原论文路口左转对向车失败（硬编码于 `scripts/x_v2x_agent.py:581-631`，无消融开关）
3. **IGPR 方差驱动 APF 权重** — 预测不确定性高时自动增大安全裕度（`IGPR_predict_sv_wps.py` + `scripts/x_v2x_agent.py:807-839`）
4. **B-spline 全局参考线平滑** — QP 约束优化，C² 连续性（`utils/global_bspline_smoother.py`）

### 消融开关

| 环境变量             | 默认  | 含义                       | 代码位置            |
| -------------------- | ----- | -------------------------- | ------------------- |
| `USE_DUAL`         | `1` | OBCA 多边形对偶约束        | `udmc_main.py:31` |
| `ABLATION_IGPR`    | `1` | IGPR 方差驱动 APF 权重     | `udmc_main.py:33` |
| `ABLATION_BSPLINE` | `1` | B-spline 平滑              | `udmc_main.py:35` |
| `ABLATION_QP`      | `1` | QP 凸预规划热启动          | `udmc_main.py:37` |
| `RANDOM_SEED`      | 无    | 随机交通生成种子（可复现） | `udmc_main.py:40` |

> **注意**：对向车检测与让行（改进 2）为硬编码逻辑，无消融开关。

## 可用 Skills

### 项目 Skills

| Skill          | 路径                                        | 用途                               |
| -------------- | ------------------------------------------- | ---------------------------------- |
| CARLA 服务管理 | `.agents/skills/carla_server/SKILL.md`    | 启动、检查、重启 CARLA 仿真服务    |
| 实验执行       | `.agents/skills/run_experiments/SKILL.md` | 运行对比/消融实验的完整流程        |
| 代码导航       | `.agents/skills/code_navigation/SKILL.md` | 核心文件地图、高风险文件、调试技巧 |

### 论文写作 Skills

| Skill          | 路径                                                     | 用途                              |
| -------------- | -------------------------------------------------------- | --------------------------------- |
| 科学写作       | `.agents/skills/scientific-writing/SKILL.md`           | IMRAD 结构、两阶段写作法          |
| 写作 Prompt 集 | `.agents/skills/awesome-writing-prompts/SKILL.md`      | 中转英、润色、去 AI 味实战 prompt |
| 引用管理       | `.agents/skills/citation-management/SKILL.md`          | 搜索文献、生成 BibTeX、验证引用   |
| 文献综述       | `.agents/skills/literature-review/SKILL.md`            | 系统化文献搜索、主题综述撰写      |
| 科学批判思维   | `.agents/skills/scientific-critical-thinking/SKILL.md` | 评估实验设计、证据质量分析        |
| 同行评审       | `.agents/skills/peer-review/SKILL.md`                  | 模拟审稿人视角审查                |
| 学术评估       | `.agents/skills/scholar-evaluation/SKILL.md`           | ScholarEval 量化评分框架          |
| Matplotlib     | `.agents/skills/matplotlib/SKILL.md`                   | 底层绑图，完全自定义              |
| 科学可视化     | `.agents/skills/scientific-visualization/SKILL.md`     | 出版级图表、色盲友好配色          |
| 统计分析       | `.agents/skills/statistical-analysis/SKILL.md`         | 选择统计方法、APA 格式报告        |
| 期刊模板       | `.agents/skills/venue-templates/SKILL.md`              | Elsevier 等期刊格式规范           |
| ML 论文写作    | `.agents/skills/ml-paper-writing/SKILL.md`             | ML/AI 论文全流程                  |

### Skills 调用策略

根据修改任务类型，自动选择合适的 skill 辅助：

| 任务类型           | 推荐 Skill                                    | 说明                                       |
| ------------------ | --------------------------------------------- | ------------------------------------------ |
| 撰写/重写英文段落  | `scientific-writing`                        | IMRAD 结构、学术写作原则、两阶段写作法     |
| 中文想法→英文段落 | `awesome-writing-prompts`                   | 中转英、润色、去 AI 味等实战 prompt        |
| 润色已有英文段落   | `awesome-writing-prompts`                   | 表达润色、缩写/扩写、逻辑检查 prompt       |
| 添加/验证引用      | `citation-management`                       | 搜索文献、生成 BibTeX、验证引用准确性      |
| 补充文献综述       | `literature-review`                         | 系统化文献搜索、主题综述撰写               |
| 回应方法论质疑     | `scientific-critical-thinking`              | 评估实验设计、识别偏差、证据质量分析       |
| 自查论文质量       | `peer-review`                               | 模拟审稿人视角审查（方法、统计、图表）     |
| 评估论文整体水平   | `scholar-evaluation`                        | ScholarEval 量化评分框架                   |
| 绘制科学图表       | `matplotlib` + `scientific-visualization` | 出版级图表，色盲友好配色                   |
| 统计分析/检验      | `statistical-analysis`                      | 选择合适的统计方法、APA 格式报告           |
| 会议/期刊格式要求  | `venue-templates`                           | Elsevier 等期刊的格式规范与写作风格指南    |
| 写完整论文章节     | `ml-paper-writing`                          | ML/AI 论文写作全流程（引用验证、审稿标准） |

## 可用 Workflows

| 命令                     | 用途                          |
| ------------------------ | ----------------------------- |
| `/run_sim`             | 启动 CARLA 并运行单个仿真场景 |
| `/run_all_experiments` | 运行全部对比实验和消融实验    |
| `/git_commit`          | 提交代码到 Git 仓库           |

## 语言规范

- **代码注释**：全中文
- **文档标题与说明**：全中文
- **变量名 / 函数名 / 文件名**：保留英文（机器读取，不翻译）

## Git Commit 规范

格式：`类型: 描述`，描述 ≤ 15 字，全中文。

| 类型     | 适用场景             |
| -------- | -------------------- |
| `新增` | 新功能、新文件       |
| `修复` | Bug 修复             |
| `优化` | 性能或代码质量改进   |
| `重构` | 不改变功能的结构调整 |
| `移除` | 删除冗余代码或文件   |

示例：✅ `优化: 减少 GPR 预测延迟` ❌ `feat: optimize gpr`

## CARLA 环境检查

本仓库固定对应 **CARLA 0.9.16**，不要与 `~/phd/udmc_carla-main myfinal`（0.9.14 仓库）混用。

| 项目        | 0.9.16（本仓库）                        |                  0.9.14（旧仓库） |
| ----------- | --------------------------------------- | --------------------------------: |
| 代码仓库    | `~/phd/0.9.16udmc_carla-main myfinal` | `~/phd/udmc_carla-main myfinal` |
| Python 环境 | `conda activate first1`               |          `conda activate first` |
| CARLA 目录  | `~/CARLA_0.9.16`                      |                `~/CARLA_0.9.14` |
| 默认端口    | **2001**                          |                              2000 |

修改 `scripts/` 目录前，提醒用户确认 CARLA 0.9.16 已启动：

```shell
conda activate first1
cd ~/CARLA_0.9.16
./CarlaUE4.sh -carla-rpc-port=2001
```

## 实验设计

### 实验场景（4 个随机交通场景）

| 场景       | 关键字              | 特点                                 |
| ---------- | ------------------- | ------------------------------------ |
| 无信号路口 | `unsig_crossroad` | 常绿灯，100 辆随机车，左转让行最复杂 |
| 环岛       | `roundabout`      | 环岛汇入，100 辆随机车               |
| 多车道巡航 | `multilaneACC`    | 直线高速跟车，100 辆随机车           |
| 混合交通   | `mixed_traffic`   | 车辆+行人混合，100 辆随机车          |

### Baseline

| 方法         | 配置                      | 角色           |
| ------------ | ------------------------- | -------------- |
| GRIPC (full) | 四开关全开                | 本文方法       |
| UDMC         | 四开关全关                | 直接改进基线   |
| FSM          | `fsm_main(baseline).py` | 传统规则型基线 |

### 实验矩阵（`run_experiments.sh`）

- **对比实验**：4 场景 × 3 方法（GRIPC / UDMC / FSM）× 40 轮 = 480 次
- **消融实验**：4 场景 × 4 消融（noOBCA / noIGPR / noBSP / noQP）× 40 轮 = 640 次
- **总计**：1120 次仿真

### 其他可用场景（调试/演示用，不纳入正式实验）

`unsig_s1` / `unsig_s2` / `unsig_s3` / `round_s1` / `cross_s1` / `crossroad`

## 核心文件地图

### 入口文件

| 文件                      | 用途                                   |
| ------------------------- | -------------------------------------- |
| `udmc_main.py`          | GRIPC 主入口，消融开关初始化，场景定义 |
| `fsm_main(baseline).py` | FSM 基线入口，无 GRIPC 改进            |
| `run_experiments.sh`    | 批量实验自动化脚本                     |
| `extract_metrics.py`    | 实验数据后处理，支持批量提取 14 列指标 |

### 核心算法（`scripts/`）

| 文件                        | 用途                                            |
| --------------------------- | ----------------------------------------------- |
| `scripts/x_v2x_agent.py`  | 主控决策 Agent（ACC、超车、让行、四大改进集成） |
| `scripts/vehicle_obs.py`  | 车辆动力学模型、MPC OCP、APF 权重配置           |
| `scripts/env.py`          | CARLA 环境封装                                  |
| `scripts/others_agent.py` | 周围车辆行为模型                                |

### 工具模块（`utils/`）

| 文件                                         | 用途                                  |
| -------------------------------------------- | ------------------------------------- |
| `utils/geometry_poly.py`                   | OBCA 多边形 → 半空间约束 `Ax ≤ b` |
| `utils/global_bspline_smoother.py`         | B-spline 参考线平滑（OSQP 求解）      |
| `utils/hybrid_qp_warmstart.py`             | QP 预规划热启动                       |
| `utils/carla_utils.py`                     | CARLA 通用工具函数                    |
| `utils/draw.py` / `utils/draw_result.py` | 可视化绘图                            |

### 预测模块

| 文件                       | 用途                           |
| -------------------------- | ------------------------------ |
| `IGPR_predict_sv_wps.py` | GPR 轨迹预测，输出均值 + 方差  |
| `gpr.pkl` / `lstm.pkl` | 预训练模型文件（不可直接编辑） |

## Git 操作规范

- **每次修改代码前**，必须先 `git add -A && git commit && git push`
- 仓库**必须设为私密**（Private）

## 数据文件约定

`.npy` 和 `.pkl` 为二进制数据 / 模型文件，**不直接编辑**：

- 模型文件（`gpr.pkl` / `lstm.pkl`）→ 重新运行训练脚本
- 数据文件（`*.npy`）→ 重新运行 `carla_nl_system_identification.py`
- 实验日志（`logs/` 目录）→ 由仿真自动生成，带时间戳不覆盖
- 正式实验数据（`experiment_results/` 目录）→ 由 `run_experiments.sh` 生成，用 `extract_metrics.py` 提取指标

## 项目维护日志规范

**日志目录**：`changelogs/`

### 日志规则

每次对项目进行维护（修改代码、调参、修复 bug、新增功能等），**必须**在 `changelogs/` 下创建一份维护日志，格式如下：

- **文件名**：`YYYY-MM-DD_HHMMSS.md`（与维护时间对应）
- **内容模板**：

```markdown
# 维护日志 YYYY-MM-DD HH:MM

## 修改概述
（一句话总结本次维护目标）

## 修改详情

| 文件 | 修改内容 | 原值 → 新值 |
|------|----------|-------------|
| ... | ... | ... |

## 修改原因
（为什么要做这些修改？触发问题是什么？）

## 验证结果
（修改后的测试结果，是否解决了问题）

## 关联场景
（受影响的仿真场景）

## 备注
（其他需要记录的信息，如已知遗留问题、后续计划等）
```

### 写日志的时机

- 每次 `git commit` 前，先写维护日志
- 日志粒度：一次完整的维护任务对应一份日志（不要拆得太细，也不要合并多个不相关任务）
- 日志用中文撰写，技术术语保留英文
