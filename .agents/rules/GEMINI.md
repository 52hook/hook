---
trigger: always_on
---

每次都叫我帅哥

## 项目概述

**GRIPC**（Geometry-Rule Integrated Planning and Control）— 面向城市自动驾驶的几何-规则集成规划与控制框架。基于 CARLA 0.9.16 仿真环境，在 UDMC 框架基础上进行四项核心改进。

### GRIPC 四大核心改进（相对于原始 UDMC）
1. **OBCA 多边形硬约束** — 替代椭圆近似，消除几何保守性
2. **对向车检测与让行** — 解决原论文路口左转对向车失败
3. **IGPR 方差驱动 APF 权重** — 预测不确定性高时自动增大安全裕度
4. **B-spline 全局参考线平滑** — QP 约束优化，C² 连续性

### 消融开关
| 环境变量 | 默认 | 含义 |
|---------|------|------|
| `USE_DUAL` | `1` | OBCA 多边形对偶约束 |
| `ABLATION_OPP` | `1` | 对向车检测与让行 |
| `ABLATION_IGPR` | `1` | IGPR 方差驱动 APF 权重 |
| `ABLATION_BSPLINE` | `1` | B-spline 平滑 |

## 可用 Skills

### 项目 Skills
| Skill | 路径 | 用途 |
|-------|------|------|
| CARLA 服务管理 | `.agents/skills/carla_server/SKILL.md` | 启动、检查、重启 CARLA 仿真服务 |
| 实验执行 | `.agents/skills/run_experiments/SKILL.md` | 运行对比/消融实验的完整流程 |
| 代码导航 | `.agents/skills/code_navigation/SKILL.md` | 核心文件地图、高风险文件、调试技巧 |

### 论文写作 Skills
| Skill | 路径 | 用途 |
|-------|------|------|
| 科学写作 | `.agents/skills/scientific-writing/SKILL.md` | IMRAD 结构、两阶段写作法 |
| 写作 Prompt 集 | `.agents/skills/awesome-writing-prompts/SKILL.md` | 中转英、润色、去 AI 味实战 prompt |
| 引用管理 | `.agents/skills/citation-management/SKILL.md` | 搜索文献、生成 BibTeX、验证引用 |
| 文献综述 | `.agents/skills/literature-review/SKILL.md` | 系统化文献搜索、主题综述撰写 |
| 科学批判思维 | `.agents/skills/scientific-critical-thinking/SKILL.md` | 评估实验设计、证据质量分析 |
| 同行评审 | `.agents/skills/peer-review/SKILL.md` | 模拟审稿人视角审查 |
| 学术评估 | `.agents/skills/scholar-evaluation/SKILL.md` | ScholarEval 量化评分框架 |
| Matplotlib | `.agents/skills/matplotlib/SKILL.md` | 底层绑图，完全自定义 |
| 科学可视化 | `.agents/skills/scientific-visualization/SKILL.md` | 出版级图表、色盲友好配色 |
| 统计分析 | `.agents/skills/statistical-analysis/SKILL.md` | 选择统计方法、APA 格式报告 |
| 期刊模板 | `.agents/skills/venue-templates/SKILL.md` | Elsevier 等期刊格式规范 |
| ML 论文写作 | `.agents/skills/ml-paper-writing/SKILL.md` | ML/AI 论文全流程 |

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

| 命令 | 用途 |
|------|------|
| `/run_sim` | 启动 CARLA 并运行单个仿真场景 |
| `/run_all_experiments` | 运行全部对比实验和消融实验 |
| `/git_commit` | 提交代码到 Git 仓库 |

## 语言规范

- **代码注释**：全中文
- **文档标题与说明**：全中文
- **变量名 / 函数名 / 文件名**：保留英文（机器读取，不翻译）

## Git Commit 规范

格式：`类型: 描述`，描述 ≤ 15 字，全中文。

| 类型 | 适用场景 |
|---|---|
| `新增` | 新功能、新文件 |
| `修复` | Bug 修复 |
| `优化` | 性能或代码质量改进 |
| `重构` | 不改变功能的结构调整 |
| `移除` | 删除冗余代码或文件 |

示例：✅ `优化: 减少 GPR 预测延迟` ❌ `feat: optimize gpr`

## CARLA 环境检查

本仓库固定对应 **CARLA 0.9.16**，不要与 `~/phd/udmc_carla-main myfinal`（0.9.14 仓库）混用。

| 项目 | 0.9.16（本仓库） | 0.9.14（旧仓库） |
|------|----------------|--------------:|
| 代码仓库 | `~/phd/0.9.16udmc_carla-main myfinal` | `~/phd/udmc_carla-main myfinal` |
| Python 环境 | `conda activate first1` | `conda activate first` |
| CARLA 目录 | `~/CARLA_0.9.16` | `~/CARLA_0.9.14` |
| 默认端口 | **2001** | 2000 |

修改 `scripts/` 目录前，提醒用户确认 CARLA 0.9.16 已启动：

```shell
conda activate first1
cd ~/CARLA_0.9.16
./CarlaUE4.sh -carla-rpc-port=2001
```

## 实验场景体系

### 核心验证场景（消融 + 深入对比）
| 场景 | 关键字 | 验证目标 |
|------|--------|---------:|
| 无信号路口（低密度） | `unsig_s1` | 对向车检测 |
| 无信号路口（中密度） | `unsig_s2` | 对向检测 + IGPR 方差 |
| 无信号路口（高密度） | `unsig_s3` | 全部改进协同 |
| 环岛入口紧迫 | `round_s1` | OBCA + B-spline |
| 有信号路口对向逼近 | `cross_s1` | 对向检测（复现原论文失败） |

### 通用性验证场景
| 场景 | 关键字 |
|------|--------|
| 多车道巡航 | `multilaneACC` |
| 环岛 | `roundabout` |
| 混合交通（含行人） | `mixed_traffic` |

## Git 操作规范

- **每次修改代码前**，必须先 `git add -A && git commit && git push`
- 仓库**必须设为私密**（Private）

## 核心文件保护

以下文件改动风险较高，**修改前必须向用户提请审核**：

- `scripts/vehicle_obs.py` — 核心势场、多边形约束、MPC OCP
- `scripts/x_v2x_agent.py` — 主控决策（ACC、超车、让行）

## 数据文件约定

`.npy` 和 `.pkl` 为二进制数据 / 模型文件，**不直接编辑**：
- 模型文件（`gpr.pkl` / `lstm.pkl`）→ 重新运行训练脚本
- 数据文件（`*.npy`）→ 重新运行 `carla_nl_system_identification.py`
