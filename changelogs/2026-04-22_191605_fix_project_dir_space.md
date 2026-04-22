# 修复 run_experiments.sh PROJECT_DIR 路径含空格导致 cd 失败

**日期**：2026-04-22 19:16:05
**关联提交**：待 push

## 修改概述

修复 `run_experiments.sh:16` 历史遗留的路径 bug：变量定义中使用空格分隔，与实际磁盘路径（下划线）不符，导致 `set -euo pipefail` 下 `cd "${PROJECT_DIR}"` 立即失败。由 Route Z A/B 基础设施改动引入的独立 code-review agent 捕获。

## 修改详情

**文件**：`run_experiments.sh:16`

**原值**：
```bash
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main myfinal"
```
（`main` 与 `myfinal` 之间是空格）

**新值**：
```bash
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
```
（下划线连接，与实际目录 `/home/jh/phd/0.9.16udmc_carla-main_myfinal` 一致）

## 修改原因

- 实际磁盘路径为 `0.9.16udmc_carla-main_myfinal`（下划线），但脚本变量写作空格，`cd` 必然找不到目录。
- `set -euo pipefail` 配合 `cd` 失败会让脚本立刻退出，所有后续 `run_udmc` / `run_fsm` 调用都不会发生。
- 旧 1112 条实验数据推测是从项目根手动运行（未触发 `cd`），或使用另一份未污染的脚本副本。当前为 Route Z 全量重跑做准备，必须修复此 bug 以保证 `bash run_experiments.sh` 可直接启动。

## 验证结果

- `bash -n run_experiments.sh` 语法通过
- `ls "$HOME/phd/0.9.16udmc_carla-main_myfinal"` 确认路径真实存在
- `scripts/ab_bspline_routez.sh` 本身已经写的是下划线，不受此 bug 影响；本次修复只为未来全量跑 640 次 `run_experiments.sh` 铺路

## 关联场景

所有批量实验矩阵场景（`unsig_crossroad` / `roundabout` / `multilaneACC` / `mixed_traffic`）。

## 备注

- 此 bug 与 Route Z 无关，由 commit 3d6af8a 的 code-review agent 顺带发现，SCI大王指示顺手修掉。
- 不影响 60 次小样本 A/B（`scripts/ab_bspline_routez.sh`）执行。
