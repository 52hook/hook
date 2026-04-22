# 2026-04-22 11:01 B-spline 场景自适应 α + dist_error_raw 日志列

## 修改概述
两件并行改动，目的是诊断 B-spline 平滑是否把 ego 全程拉偏：

1. **新增日志列 `dist_error_raw`**：ego 到 **B-spline 前**原始参考轨迹（`cx0/cy0`）最近点距离；对比现有 `dist_error`（基于 B-spline 后参考轨迹）。14 → 15 列。
2. **新增 `ABLATION_BSPLINE_ADAPTIVE` 开关 + 场景自适应 α**：
   - 默认 `0`（关闭），维持原硬编码 `α=0.7` 行为，不破坏旧实验对比语义。
   - 开启 `1` 时按场景查表：`roundabout=0.2`、`multilaneACC=0.4`、其他=0.7；红灯统一 0.0 不变。

## 修改详情

### `scripts/x_v2x_agent.py`
| 位置 | 改动 |
|---|---|
| `__init__` 签名 | 新增参数 `bspline_adaptive=False` |
| `__init__` 体内 | 新增 `self._bspline_adaptive` 与 `self._alpha_bs_by_scenario` 字典 |
| 原 430-433 行 alpha_bs 硬编码 | 改为三分支：红灯 0.0 / 自适应查表 / 旧行为 0.7 |
| 原 1088 行附近 | 新增 `dist_error_raw = min(hypot(ego - (cx0, cy0)))` |
| 原 1117 行 `_log_data.append` | 追加 `dist_error_raw` 作为第 15 列 |

### `udmc_main.py`
| 位置 | 改动 |
|---|---|
| 消融开关块（~36 行） | 新增 `USE_BSPLINE_ADAPTIVE = os.environ.get('ABLATION_BSPLINE_ADAPTIVE', '0') != '0'` |
| Xagent 构造（~218 行） | 传入 `bspline_adaptive=USE_BSPLINE_ADAPTIVE` |
| `[Config]` 打印 | 加 `bs_adaptive={...}` 字段 |

### `fsm_main(baseline).py`
不修改 Xagent 调用（保持默认关闭自适应），FSM 自动获得新列 `dist_error_raw`。

### `extract_metrics.py`
| 位置 | 改动 |
|---|---|
| 列定义文档（15-30 行） | 新增 `[14] dist_error_raw` 说明 |
| `extract_single()` | 增加列数判断：15 列时读取 `dist_error_raw` 并产出 `rmse_dist_err_raw`, `mean_dist_err_raw`；14 列老文件降级处理，字段置 `nan` |

## 修改原因
- 严格审查显示 GRIPC roundabout 全程横向误差偏大（0.868m vs UDMC 0.532m），猜测 B-spline 把参考线外扩，ego 实际跟随得很好但"参考线"本身已偏离原始车道中心。`dist_error_raw` 用于剥离"ego 追不上 B-spline 线"与"B-spline 线本身偏离原车道中心"两种效应。
- α 场景自适应：roundabout 曲率高 → 降 α 保留原始转弯点；multilaneACC 直道换道触发延迟 → 中度降 α；unsig_crossroad 原 α=0.7 表现良好 → 不动。

## 验证计划
- smoke test：`python -c "from scripts.x_v2x_agent import Xagent; from udmc_main import *"` import 无错
- 起 Explore agent 审查改动一致性（新列语义、分支覆盖、向后兼容）
- 单仿真试跑（用户手动）：对比 roundabout_GRIPC 新日志第 15 列值 vs 第 2 列

## 关联场景
- 影响：所有四个正式场景 + 调试场景
- 新开关默认关闭 → 历史 1112 条日志对比不受影响
- 旧 14 列日志 extract_metrics.py 仍可读

## 备注
- dist_error_raw 语义：**ego 到原始离散参考点最短距离**（点到点，非横向投影）；与现有 dist_error（ego 到**下一步参考点**）不完全等价，但都走 hypot 距离口径，同场景对比有意义。
- 若后续需要横向投影误差，另加列 `lat_error_raw_projected`，此次不做。
