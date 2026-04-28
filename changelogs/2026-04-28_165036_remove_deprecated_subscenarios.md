---
日期: 2026-04-28 16:50:36
类型: 移除
作者: AI 协作
关联场景: unsig_s1 / unsig_s2 / unsig_s3 / round_s1 / cross_s1
---

# 维护日志：移除淘汰的调试子场景

## 修改概述

删除 5 个已淘汰的调试用子场景（`unsig_s1` / `unsig_s2` / `unsig_s3` / `round_s1` / `cross_s1`）的全部代码引用，保留 `crossroad` 主场景与 4 个正式实验场景（`unsig_crossroad` / `roundabout` / `multilaneACC` / `mixed_traffic`）。

## 修改详情

### 1. udmc_main.py
- line 32 注释示例：`unsig_s1` → `unsig_crossroad`
- line 80-82 `VALID_SCENARIOS`：删除 `'unsig_s1'`, `'unsig_s2'`, `'unsig_s3'`, `'round_s1'`, `'cross_s1'`
- line 124-179：删除 3 个 setup 块（`unsig_s1/s2/s3`、`round_s1`、`cross_s1`）
- line 273-343：删除 3 个对应 spawn 块

### 2. fsm_main(baseline).py
- line 57-58 `VALID_SCENARIOS`：同步删除 5 个
- line 100-143：删除 setup 块
- line 208-243：删除 spawn 块
- 注：保留 line 348 `crossroad` 红绿灯切换逻辑（crossroad 保留）

### 3. scripts/x_v2x_agent.py:537
- `elif self._scenario in ('roundabout', 'round_s1')` → `elif self._scenario == 'roundabout'`

### 4. extract_metrics.py:197
- `KNOWN_SCENARIOS` 删除 5 个名字（保留 `crossroad`）
- 行为不变：未知 scenario 仍能解析（fallback 已存在）

### 5. test_all_scenarios.sh:25
- `FIXED_SCENARIOS` 数组删除 5 个，保留 `mixed_traffic`

## 修改原因

- 这 5 个 `_s*` 子场景是项目早期为复现原 UDMC 论文 rare-failure cases 设计的调试场景，已被实验设计淘汰
- 正式实验矩阵（4×3×40 + 4×4×40）只用 `unsig_crossroad` / `roundabout` / `multilaneACC` / `mixed_traffic` 四主场景
- 残留代码增加阅读心智负担、稀释 grep 信号、误导新维护者；清理后代码图更干净

## 验证结果

- `python -m py_compile udmc_main.py 'fsm_main(baseline).py' scripts/x_v2x_agent.py extract_metrics.py` → 退出码 0
- `bash -n test_all_scenarios.sh` → 退出码 0
- `grep -rn -E 'cross_s1|round_s1|unsig_s[123]' --include='*.py' --include='*.sh' --exclude-dir=udmc_carla-main` → 0 命中

## 关联场景

无（删除项不在正式实验矩阵中）

## 备注

- `crossroad` 主场景保留：CLAUDE.md 虽列其为调试用，但它是独立主场景，含 line 403 红绿灯切换逻辑，删除影响面更广，本轮不动
- 文档（CLAUDE.md / AGENTS.md / .agents/* / .claude/* / 下一步建议.md / 历史 changelogs/）会出现 stale 引用，本轮不动，下轮统一清理
- `udmc_carla-main/` 原始 UDMC 对照目录里同名场景未删除（按硬约束仅本仓库代码）
