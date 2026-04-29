# OBCA E+A+G 修复 Step 8: codex 复核 NEEDS-FIX 三处补丁

## 修改概述

Steps 1-7 push 后, 由 OpenAI Codex CLI 对 OBCA 修复链做技术正确性
评审, verdict = **NEEDS-FIX**, 抓出 3 个我自审漏掉的接线 bug:

1. (F1) 本拍 0 OBCA 障碍时未调 `disable_dual_collision()`, 上一拍
   `dual_enabled=True / obs_Am_seq` 残留 -> 本拍 NLP 重建时被当成
   "幽灵车" 加进去, 真车已经离开还在被约束。
2. (F2) `obca_obs_idx_set` 存的是 `env.get_obs()` 全集 (size N) 下标,
   但下游 partition 循环用 `_filter_front_same_lane_obs()` 过滤后
   (size M ≤ N) 的下标 k. 集合 hit 检查彻底错位 -> Step 5 的
   `ratio*0.4 floor` 完全失效或随机命中。
3. (F3) `obca_qp_preplan` 内部 `HybridQPWarmStarter(d_safe=0.5)` 写死,
   与 Step 4 主 NLP `d_safe=0.30` 不一致, QP 在更紧约束下解出的轨迹
   作 primal 初值, 偏离 NLP 真正可行域。QP 只读 k=0 静态 obs 是已知
   简化, 此处不重构。

经"codex 复核 + 自审二次"双闸通过, 合并为一次 commit+push (按
`feedback_commit_granularity.md` 新规则)。

## 修改详情

### 文件: scripts/x_v2x_agent.py

1. **`_filter_front_same_lane_obs` 签名扩展** (line 165~260)
   - 新增第 4 个返回值 `keep_idx: np.ndarray(int)`, 记录每个保留 obs
     在原 obs 中的下标 (空时 `np.zeros((0,), dtype=int)`)
   - 三个 early-return 路径 (obs is None / obs.size==0 / keep_idx 空)
     都补齐 keep_idx 返回, 维持 4-tuple 形状一致。

2. **OBCA-hit 集合重映射** (主流程 line ~707)
   ```python
   obs, heights, obs_infer, keep_idx = self._filter_front_same_lane_obs(...)
   if len(obca_obs_idx_set) > 0 and len(keep_idx) > 0:
       pre2post = {int(p): post for post, p in enumerate(keep_idx)}
       obca_obs_idx_set = {pre2post[p] for p in obca_obs_idx_set
                           if p in pre2post}
   else:
       obca_obs_idx_set = set()
   ```
   下游 `is_obca_hit = (k in obca_obs_idx_set)` 中 k 现在与集合元素
   都在 post-filter 索引域中, 命中正确。

3. **0 OBCA 障碍 disable 兜底** (`if self._use_dual:` 块尾, line ~660)
   ```python
   else:
       if hasattr(self._model, "disable_dual_collision") and \
               getattr(self._model, "dual_enabled", False):
           self._model.disable_dual_collision()
       if hasattr(self._model, "_qp_ws_valid"):
           self._model._qp_ws_valid = False
   ```
   只有上一拍真的 enabled 才调 disable, 避免无意义的状态切换。
   disable_dual_collision 内部清 `obs_Am_seq / obs_bm_seq /
   obs_actor_ids / num_obs / dual_enabled`, 本拍 solver_add_cost
   /bounds 走非 OBCA 分支 (等价老 UDMC)。

### 文件: scripts/vehicle_obs.py

4. **QP `d_safe` 与 NLP 对齐** (line ~1056, `obca_qp_preplan` 内部)
   ```python
   qp_d_safe = float(getattr(self, 'd_safe', 0.30))
   self.qp_warmstarter = HybridQPWarmStarter(
       ..., d_safe=qp_d_safe, ...
   )
   ```
   NLP `self.d_safe = 0.30` (Step 4 设), QP 跟随读取。运行期改
   `self.d_safe` 也会自动同步 (本仓库无运行期改, 但接口对得上)。

## 修改原因

codex 复核报告 (`/home/jh/.nvm/.../codex exec --skip-git-repo-check
"..."`, 2026-04-29 11:59 UTC) 直接给出 NEEDS-FIX + top-3 必修问题。
我对每一项做了二次自审, 用 `grep` / `Read` 确认:

- F1: 编辑器查到 `solver_add_cost / solver_add_bounds` 每 tick
  从 x_v2x_agent.py:1037/1098 重新调用, 读 `self.dual_enabled`
  + `self.obs_Am_seq`. 确认 stale 路径真实存在。
- F2: 编辑器看到 line 587 `best_i` 是 env 全集下标, line 696 后
  obs 被重新编号; line 744 `(k in obca_obs_idx_set)` 用 post-filter
  k -> 错位真实存在。
- F3: 看到 line 1056 d_safe=0.5 写死; Step 4 主 NLP 是 0.30 不一致
  存在。但 QP-only-k=0 不在本 step 修复 (重构成本超 Tier 2 预算)。

## 验证结果

```
$ python -m py_compile scripts/vehicle_obs.py scripts/x_v2x_agent.py \
  utils/geometry_poly.py
(无输出 -> 通过)

$ python utils/_smoke_obca_step17.py
[T1] actors_to_Ab_horizon ... PASS
[T2] actor_to_Ab_pose == actors_to_Ab (重合 pose) ... PASS
[T3] _dual_cache_by_id reorder ... PASS
[T4] _dual_cache_by_id edge mismatch -> 0 兜底 ... PASS
[SMOKE PASS]
```

- 静态 smoke 4/4 全过, 几何接口 / dual cache 字典逻辑未被改坏。
- F2 改动仅扩展返回值, 已知唯一 caller (line 707) 同步更新, 无遗漏。
- F1 / F3 是 0 obs 与 d_safe 单值同步, 不会对 OBCA-on / NLP 求解
  逻辑产生副作用。

## 关联场景

- **F1 直接受益**: ACC 直线驶离前车 (multilaneACC) / 环岛汇入后车流
  分散 (roundabout) 等障碍数量动态归零的场景, 不再被幽灵车约束。
- **F2 直接受益**: 同侧前车 + OBCA-hit 同一辆 (mixed_traffic 紧贴
  前车) - APF 现在真的会按 ratio*0.4 让位给硬约束, 解决 Step 5
  名存实亡问题。
- **F3 直接受益**: QP-on (ABLATION_QP=1) 时主 NLP 收到的 primal 初值
  贴近真可行域, IPOPT 迭代次数下降。

## 备注

- codex 还在报告里指出 QP 只读 k=0 静态障碍是设计层面问题, 修复需要
  改 `HybridQPWarmStarter` 接受时序 (A_k, b_k) 序列, 或者 fallback
  关 QP. 不在本 step 范围, 留作 Step 9 候选。
- 静态 smoke 仍未覆盖 IPOPT 收敛 / MPC 时延 / dual warm-start 命中
  率, 这些只能在 CARLA 闭环 (Step 7b/7c, 用户网关) 量化。
- 按用户新规则, Steps 1-7 的 7 次 push 之后, 这次 codex 复核加修复
  共 4 个 bug 合并为 1 次 commit+push。
