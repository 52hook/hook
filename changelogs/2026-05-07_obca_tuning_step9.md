# 优化: OBCA 路口/混合分支 5 处调参 (Step 9)

时间: 2026-05-07

## 现象

A 路径 paired-seed 实验 (Steps 1-8 OBCA 修复后) 出现:

> ego 在距离对向/横向车辆还很远 (≈25-30m) 时就开始大角度绕行,
> 横向偏移幅度过大、不像人类驾驶员的紧凑避让.

## 根因 (codex 复核 + 文献交叉验证)

OBCA 多边形硬约束的"激活范围 + 障碍物充气 + ego 扩张 + 时间索引"
四组参数叠加, 在城市低速路口被放大:

1. **mask 漏标**: `max_dual_veh=4` 但 `max_obs=2`, 4 个 actor 进了候选,
   只有 2 个进 OBCA 硬约束, 另外 2 个被错误标进 `obca_obs_idx_set`,
   APF 漏算 -> 远距离同时受 APF + OBCA 双重作用
2. **激活范围过大**: `max_obca_range=32m`, MPC horizon 仅 0.5s,
   v_rel < 25m/s 时 32m 远超 horizon 视野, OBCA 提前 1s+ 介入
3. **k=0 索引错位**: `actors_to_Ab_horizon` 用 `traj[0]` 当作当前帧,
   IGPR `pred[0]` 实际是 k=1 的预测, 直接传入会让 OBCA 第一拍抢跑 0.05s,
   连续累积横向偏置
4. **障碍物虚胖**: `inflate_vehicle=0.6m` (单边), 比 TDR-OBCA 类工程值高
5. **ego 过分扩张**: `+0.6m/+0.4m` 与 inflate(0.6) + d_safe(0.30) 叠加,
   总缓冲 1.4m, 远超城市路口需要

## 修改详情

### 文件: `scripts/x_v2x_agent.py`

仅改路口/混合分支 (`unsig_crossroad`, `mixed_traffic`),
ACC/roundabout 分支不动 (经验已稳).

**修改 1 (line 555-556): max_dual_veh + max_obca_range**

```diff
-                max_dual_veh = 4
-                max_obca_range = 32.0
+                max_dual_veh = 2          # 与 max_obs=2 对齐, 修 mask 漏标
+                max_obca_range = 18.0     # 32m -> 18m, TTC 余量 0.25s
```

**修改 2 (line 587-602): traj 前置当前位姿, 修 k=0 索引**

```diff
+                    a_yaw_rh = -math.radians(float(a.get_transform().rotation.yaw))
                     ...
                     if best_i >= 0 and best_d2 < 1.0 and best_i < len(obs_infer):
                         pred = np.asarray(obs_infer[best_i]).reshape(-1, 3)
-                        traj = [(float(p[0]), float(p[1]), float(p[2])) for p in pred]
+                        traj = [(ax_rh, ay_rh, a_yaw_rh)]
+                        traj.extend((float(p[0]), float(p[1]), float(p[2])) for p in pred)
```

`utils/geometry_poly.py:267` 内部已有 `traj_list[:horizon+1]` 截断,
前置后 11 帧 = `now + pred[0..9]`, 正好对齐 horizon+1.

**修改 3 (line 614): inflate_vehicle**

```diff
-                    inflate_vehicle=0.6,
+                    inflate_vehicle=0.3,    # 0.6 -> 0.3 (工程调参值)
```

**修改 4 (line 633-634): ego 扩张**

```diff
-                ego_L_enlarge = ego_L + 0.6
-                ego_W_enlarge = ego_W + 0.4
+                ego_L_enlarge = ego_L + 0.3
+                ego_W_enlarge = ego_W + 0.2
```

总缓冲: 1.4m -> 0.9m (含 inflate=0.3 + d_safe=0.30 + ego+0.3/0.2 一半).

## 不修改的项 (避免引入二次震荡)

- `d_safe=0.30, s_max=0.30, beta_s=500.0, gamma_l2_s=1000.0` — Step 4 锁定值
- `multilaneACC max_obca_range=10.0` — 经验稳定
- `roundabout max_obca_range=12.0` — 经验稳定
- `max_obs=2` — 与新的 `max_dual_veh=2` 对齐
- `max_range=32.0` (用于 `min_obs_dist` / QP 触发) — 与 OBCA 检测分离

## 验证

- `python -m py_compile scripts/x_v2x_agent.py` 通过
- `from scripts.x_v2x_agent import Xagent` 通过
- codex 双复核 (`codex exec`) 结论 PASS:
  - yaw 单位/右手系符号正确
  - traj 前置无需调用侧 trim (geometry_poly 内截断)
  - 18m 对 city low-speed 合理, 高速汇入需后续监控
  - mask 漏标已修

## 风险

1. `max_dual_veh=2` 在密集路口可能漏掉第 3 辆稍远但更危险的 crossing vehicle.
   缓解: 当前距离排序已偏向最近, 后续若出现可改为 TTC 排序.
2. 18m 对 v_rel > 25m/s 高速场景不够, 当前四场景 `target_v ≤ 35 km/h` 不触发.

## 关联场景

- `unsig_crossroad` (主要影响)
- `mixed_traffic` (同走 else 分支, 也受影响)

## 备注

- Step 1-8 的 OBCA 静态正确性已建立, 本 Step 9 仅是"动态行为合理性"调参.
- 后续 A 路径补跑 (run_a_path_rerun.sh) 可直接观察改动效果.
- 如效果不达标再考虑回退 OBCA, 当前先以本 commit 为准.
