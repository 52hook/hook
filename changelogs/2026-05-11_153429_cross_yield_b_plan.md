# 新增: 侧向直行车刹停让行 (B 计划 + codex 5 修正)

时间: 2026-05-11

## 现象

Step 9 OBCA 调参后, A 路径 43-seed 补跑统计:
- 69.8% normal
- 27.9% slow-stuck (ego 被侧向直行车切入路口, 原地打转向)
- 2.3% crash

slow-stuck 截图: ego 整好被侧向直行车别的往右打转向, 然后被陷入路口
中央局部空间, OBCA 多边形硬约束几何死局, MPC 求解极慢 (37ms+) 但
slack saturated, IPOPT 给出 bang-bang steer ±0.6, ego 反复抖动.

## 根因

OBCA d_safe=0.30 + s_max=0.30 = 0.6m elasticity. 当物理穿透存在时,
slack 饱和 -> 优化器看见 "flat gradient feasible solution" ->
control 退化为震荡, 无法跳出.

只有让 ego 在路口被侧向直行车切入时**主动刹停等通过**, 才能根本
解决几何死局. 既有 `yield_for_opp` (对向 140°+) 不覆盖 50-130° 的
侧向车, 必须新增 `yield_for_cross`.

## B 计划设计 (初版)

5 处改动新增 cross_yield 让行:
1. `RuleWeights.obs_ratio_cross_yield = 150`
2. `__init__` 加 `cross_min_dist` 候选追踪
3. partition 50-130° 分支记录最近 cross 车距离
4. 距离 <10m + 路口内 -> `yield_for_cross = True`
5. 触发时 `obs_ratio_cross 60 -> 150` (强 APF)

## codex 审查 NEEDS-FIX (5 项)

初版 diff 让 codex 审查, 拿到 5 个必修问题:

1. `in_junction` 只看 ego, 漏 ego 离开路口但 cross 车还在路口横穿
2. pre-filter line 247 用 60-120°, partition 用 50-130° -> 50-60° /
   120-130° 斜向汇入车在 pre-filter 阶段被丢
3. 距离阈值无迟滞, 10m 边界震荡风险
4. obs_ratio_cross=150 是软 APF, cross 车若被 OBCA 接管会乘 0.4 floor
   衰减到 60, 不可靠. 必须显式 v_ref=0
5. **致命**: `not is_roundabout_like` 是纯曲率判定 (k>0.015),
   unsig_crossroad 左转弯也会命中 -> cross_yield 永远禁用 ->
   B 计划在主战场完全失效

## 修改详情

### 文件: `scripts/x_v2x_agent.py`

**修改 1 (line 65-67): RuleWeights 新增让行权重**

```python
obs_ratio_cross_yield: int = 150
```

与 `obs_ratio_front=150` 同量级, 强制 MPC 把 cross 车当成"必须刹停"
障碍处理.

**修改 2 (line 117-122): __init__ 加状态机 (codex Fix #3)**

```python
self._cross_yield_active = False
```

迟滞 trigger<10m / release>15m, 避免 10m 边界震荡.

**修改 3 (line 242-249): pre-filter 50-130° (codex Fix #2)**

```diff
-            if math.radians(60.0) <= abs_yaw_diff <= math.radians(120.0):
+            if math.radians(50.0) <= abs_yaw_diff <= math.radians(130.0):
```

与 partition 循环 50-130° 对齐. 修复 50-60° / 120-130° 斜向车被
错误归类为同向被丢的问题.

**修改 4 (line 758): partition 新增 cross_min_dist 追踪**

```python
cross_min_dist = None  # 最近的侧向直行车距离
```

**修改 5 (line 795-816): cross 车 ob_in_junction 判断 (codex Fix #1)**

```python
ob_in_junction = in_junction
if not ob_in_junction:
    try:
        ob_wp = self._map.get_waypoint(
            carla.Location(x=ox, y=-oy, z=0.0),
            project_to_road=True
        )
        ob_in_junction = bool(ob_wp.is_junction) if ob_wp else False
    except Exception:
        ob_in_junction = False
if ob_in_junction and rel_x > 0.0 and d < 15.0:
    if (cross_min_dist is None) or (d < cross_min_dist):
        cross_min_dist = d
```

`(in_junction or ob_in_junction)` 兼顾 ego/cross 车任一在路口的两种
语义. 短路求值仅在 ego 不在路口时才查 cross 车 waypoint, 避免性能
浪费. CARLA 左手系坐标转换: `y_carla = -y_rh`.

**修改 6 (line 832-852): scenario 排除 + 迟滞 (codex Fix #5 + #3)**

```python
cross_yield_allowed = (self._scenario != 'roundabout')
if cross_yield_allowed and (cross_min_dist is not None):
    if not self._cross_yield_active:
        if cross_min_dist < 10.0:
            self._cross_yield_active = True
    else:
        if cross_min_dist > 15.0:
            self._cross_yield_active = False
else:
    self._cross_yield_active = False
yield_for_cross = self._cross_yield_active
```

**关键**: 用 `self._scenario != 'roundabout'` 替代
`not is_roundabout_like`. 后者是 k_window_max>0.015 曲率判定,
unsig_crossroad 左转弯也会命中, 会让 cross_yield 在主战场被错误
禁用, B 计划失效.

**修改 7 (line 1037-1039): 触发时 v_ref=0 (codex Fix #4)**

```python
if yield_for_cross:
    obs_ratio_cross = self._rw.obs_ratio_cross_yield
    ref_traj[3, :] = 0.0
```

`ref_traj` shape=(6, horizon), 行 3 是参考速度 (z_ref[3] = v).
触发时全部置零 -> MPC 速度跟踪误差项 -> ego 主动刹停. 这是核心
执行机制: 不依赖 OBCA-hit 时 APF 衰减后的 60, 用 v_ref=0 硬保证.

## codex 二审 (修后)

5 个修复全部 PASS. 2 个非阻塞 caveat 记录但不处理:

1. QP warm-start 在 line 663 读了原 ref_traj (含原速度), 但只作
   initial guess, 最终 NMPC reference 仍是 0. 不影响功能.
2. `release >15m` 实际主要走 "no candidate" 分支 (partition 已限
   d<15.0), 行为等价. 显式 >15m 分支是死代码但无害.

## 不修改的项

- `yield_for_opp` (对向 140°+ 让行) — Step 之前已稳, 不动
- `is_roundabout_like` 本身 — 仍用于 max_lat 调整, 仅 cross_yield
  不依赖它
- Step 9 OBCA 参数 (max_dual_veh=2, max_obca_range=18, inflate=0.3
  等) — 静态正确性已建立

## 验证

- `python3 -m py_compile scripts/x_v2x_agent.py` PASS
- codex 二审 (`codex exec`) PASS, 5/5 fix, 仅 2 个 non-blocking caveat
- 未跑 CARLA 仿真, 后续单 seed 抓资 (原 131 慢卡 seed) 再验

## 风险

1. **死锁风险**: cross 车若永远停在 12-14m 区间, ego 永远刹停.
   现实中 cross 车通常以 20km/h+ 通过路口, 1-2s 内离开 15m 触发解除.
   交规上"路口内停车等让"也符合 safety wait 语义, 不算 bug.
2. **过度让行**: `(in_junction or ob_in_junction)` 扩大触发面, 可能
   在 ego 刚出路口时仍刹停. 缓解: rel_x>0 + d<15m 严格筛选.
3. **QP warm-start 不一致**: codex caveat #1, 后续若发现 hard-stop
   不彻底, 可在 yield_for_cross 时 invalidate warm-start.

## 关联场景

- `unsig_crossroad` (主战场, 27.9% slow-stuck 目标)
- `mixed_traffic` (同走 else 分支)
- `roundabout` (明确排除, 走自己的循环让行)
- `multilaneACC` (理论上 cross 车少, 触发概率低)

## 备注

- B 计划是 Step 9 OBCA 调参的补救: Step 9 调小约束, B 计划遇险刹停.
- 触发条件保守 (路口内 + 前方 + 近距 + 50-130°), 不会影响正常直行.
- 下一步: 单 seed 抓资 + 跑全量 A 路径 paired-seed 看新通过率.
