# 实验跑批 v2 + FSM 补 collision_flag

## 概述

为支持方案 B 全量重跑（4 场景 × 7 配置 × 40 seed = 1120 条），重写 `run_experiments.sh` 加入 resume / retry / validation 三件套，并补 `fsm_main(baseline).py` 第 16 列 collision_flag，使 FSM 输出与 GRIPC/UDMC 16 列契约一致。

## 触发原因

Codex 复核旧 runner 发现 10 处技术缺陷，其中 5 处严重：

1. **`timeout ... || true` 静默吞失败** — 旧 1112 条而非 1120 条说明已发生
2. **`ABLATION_BSPLINE_ADAPTIVE=1`** 是隐藏第五变量（与 udmc_main.py:38 默认值不一致）
3. **conda env 未激活** — runner 在裸 shell 跑会空跑（当前 shell 无 python）
4. **commit 锁定缺失** — 跑批中途代码漂移无感知
5. **FSM 只写 14 列** — 无 collision_flag，主实验无法用 sensor-based 碰撞率

## 修改详情

### 1. `run_experiments.sh`（125 → 257 行，重写）

| 改动 | 原值 | 新值 |
|---|---|---|
| RESULTS_DIR | `experiment_results/` | `experiment_results/v2_post_rollback/` |
| conda 激活 | 无 | `set +u` 包夹 + `python -c "import carla"` preflight |
| commit 锁定 | 无 | `git merge-base --is-ancestor 17670add... HEAD` |
| ABLATION_BSPLINE_ADAPTIVE | `1` | `0`（与 udmc_main.py:38 默认值一致） |
| PLOT_RESULT | 无 | `=0`（runner 内强制） |
| is_valid() | 无 | 列==16 + 行≥100 + 第 16 列值域 {0,1,2,3} |
| 重试策略 | 一次性 `\|\| true` | 最多 3 次，3 次全失败写 `failed.tsv` |
| resume | 无 | 已有 valid 文件直接跳过 |
| 完整性验证 | 无 | 末尾 assert 每个 (method, scenario) N==40 |
| timeout | 300s | 保持 300s（用户决策） |

### 2. `fsm_main(baseline).py`（2 处小改）

- L264-269：`agent._log_data.append([...])` 从 14 元素扩到 16 元素
  - 第 15 列 `dist_error_raw = dist_error`（FSM 无 B-spline raw/smoothed 区分，表注说明语义）
  - 第 16 列 `collision_flag = 0`（占位）
- L294 后插入：`env.world.tick()` 之后 `env.consume_collisions(frame)` 写入 `_log_data[-1][15]`，与 [udmc_main.py:301-302](../udmc_main.py#L301-L302) 对齐

## 关键决策点（用户已确认）

- **倒车保持禁用**（`lbx[3]=0.0`）：Grok-Search 显示城市动态交通倒车违反 human factors 共识，所有 baseline 同条件，公平比较
- **TrafficManager seed 不加**：动 env.py 会引入新方法变量，实验设计变复杂
- **timeout 不改 600s**：用户决策，保留 300s
- **smoke deterministic probe 不加**：依赖 utils/_smoke_collision.py 验证列结构即可
- **论文 C1/C4 文字不改**：理论部分 Grok-Search 已验证为主流做法

## 验证结果

- `bash -n run_experiments.sh` SYNTAX OK
- `python3 -m py_compile "fsm_main(baseline).py"` OK
- `git diff --stat`：fsm 11 行 + runner 205 行变更

## 关联场景

全部 4 场景：unsig_crossroad / roundabout / multilaneACC / mixed_traffic

## 备注

- 跑批前需启 CARLA：`DISPLAY= nohup ~/CARLA_0.9.16/CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2001 > /tmp/carla.log 2>&1 & disown`
- 全量跑批预估 14-16h，建议 nohup + tmux
- 输出目录 `experiment_results/v2_post_rollback/` 与旧 2237 条数据完全隔离
- 完整性 fail 时参考 `failed.tsv` 单独重跑
