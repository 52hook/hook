# OBCA E+A+G 修复 Step 7: 静态 smoke 验证 (无 CARLA / 无 IPOPT)

## 修改概述

为 Steps 1-6 的全部纯几何 / 纯字典逻辑加一个无外设依赖的静态
smoke, 验证三个最容易出错的接口:

1. `actors_to_Ab_horizon` 形状 + 静态退化分支
2. `actor_to_Ab_pose` 与原 `actors_to_Ab` 在重合 pose 下输出一致
   (即 Step 1 没有改坏单帧版本)
3. `_dual_cache_by_id` 跨 slot reorder 命中 + edge 不一致兜底
   (Step 6 G2 的核心保证)

不依赖 CARLA / IPOPT / CasADi, 任何机器执行 `conda activate first1
&& python utils/_smoke_obca_step17.py` 都能跑。

## 修改详情

### 新文件: utils/_smoke_obca_step17.py

- Mock CARLA actor (`_Loc / _Rot / _Tf / _Ext / _BB / MockActor`),
  补齐 `tf.transform(loc)` 与 `bb.location` (内部 `actor_to_Ab_pose`
  会用)。
- `test_actors_to_Ab_horizon`: horizon=9, 一辆动态 + 一辆缺预测 ->
  shape `(N+1, M, 4, 2) / (N+1, M, 4)`, 静态车 k=0..N+1 全相同 (退化
  分支), 动态车 k=0 vs k=1 不同。
- `test_pose_equals_static`: yaw_deg=30, x=5, y=3 (CARLA frame) ->
  right-hand pose (yaw=-30°, y=-3) 输出与 `actors_to_Ab(y_flip=True)`
  位 bit 一致, 防止 Step 1 改坏单帧 API。
- `test_dual_cache_by_id`: 两辆车 (id=301/302) 各 4 边, 写入 ->
  swap slot -> 期望按 id 重新对齐, 且 damping=0.7 应用到 lam/mu, s
  全 0。
- `test_dual_cache_edge_mismatch`: 旧 cache 6 边, 新期望 4 边 ->
  shape mismatch, 应 0 兜底, 不抛异常。

### 一次性 bug 修复

- 第一次跑 T3 报 `module 'scripts.vehicle_obs' has no attribute
  'VehicleObs'` -> 类名实际是 `Vehicle`, 修正三处 `vobs.VehicleObs.*
  -> vobs.Vehicle.*`。

## 验证结果

```
============================================================
  OBCA Steps 1-6 静态 smoke (无需 CARLA / IPOPT)
============================================================
[T1] actors_to_Ab_horizon ...
[T1] PASS
[T2] actor_to_Ab_pose == actors_to_Ab (重合 pose) ...
[T2] PASS
[T3] _dual_cache_by_id reorder ...
[T3] PASS
[T4] _dual_cache_by_id edge mismatch -> 0 兜底 ...
[T4] PASS
============================================================
[SMOKE PASS] All static tests OK
============================================================
```

四个测试全过, Steps 1-6 的几何 / 字典 / 形状逻辑得到独立验证。

## 关联场景

- 不直接覆盖 CARLA / IPOPT 闭环, 但能在改完 OBCA 后 < 1 秒回归确认
  纯几何接口未被改坏 — 后续动 `geometry_poly.py` 或 `_dual_cache_by_id`
  族方法时, 先跑这个 smoke。

## 备注

- 还差 CARLA 闭环验证 (单场景 30s smoke + A 路径 60 seed 配对回归),
  按 CLAUDE.md §3 要求由用户确认 CARLA 已启动后再触发。
- 未覆盖: IPOPT 求解收敛性 / dual warm-start 命中率 / MPC 时延 ->
  这些只能在 CARLA 闭环里量。
- 当前仅 4 测试, 未来若改 d_safe / s_max / damping 默认值, 应在此
  smoke 加 numerical regression baseline。
