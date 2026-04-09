# 2026-04-09 规则权重 dataclass 化

## 修改概述
将 `scripts/x_v2x_agent.py` 中硬编码的 APF 权重常量（`obs_ratio_front/cross/opp`、`k_sigma`、`sigma_scale_max`、以及环岛/让行调整量）收进 `RuleWeights` dataclass，消除"tunable magic numbers"的开源风险。**纯重构**，默认值与原硬编码字面量逐字节一致，运行结果 bit-exact。

## 修改详情

### 文件：scripts/x_v2x_agent.py

**1) 新增 dataclass（class Xagent 之前）**
```python
@dataclass
class RuleWeights:
    obs_ratio_front: float = 150.0
    obs_ratio_cross: float = 60.0
    obs_ratio_opp: float = 40.0
    obs_ratio_front_round: float = 80.0
    obs_ratio_cross_round: float = 40.0
    obs_ratio_opp_round: float = 30.0
    obs_ratio_opp_yield: float = 25.0
    k_sigma: float = 2.0
    sigma_scale_max: float = 1.5
```

**2) `__init__` 内新增 `self._rw = RuleWeights()`**

**3) 替换第 828–849 行硬编码**
- `obs_ratio_front = 150` → `self._rw.obs_ratio_front`
- `obs_ratio_cross = 60` → `self._rw.obs_ratio_cross`
- `obs_ratio_opp = 40` → `self._rw.obs_ratio_opp`
- 环岛分支 `80/40/30` → `self._rw.obs_ratio_*_round`
- 让行分支 `25` → `self._rw.obs_ratio_opp_yield`
- `k_sigma = 2.0` → `self._rw.k_sigma`
- `sigma_scale_max = 1.5` → `self._rw.sigma_scale_max`

## 修改原因
原硬编码散落在 `_get_nmpc_params` 内部，审稿人/开源用户无法一眼看到全部可调参数，存在"隐藏 tunable"的学术诚实性风险。收进 dataclass 后：
1. 所有规则权重集中可见、可追溯
2. 未来做 sensitivity analysis 只需枚举 dataclass 字段
3. 便于论文附录列出完整参数表

## 验证方式
1. 默认值逐字节比对原硬编码
2. 固定 seed 跑 `unsig_crossroad` smoke test，与旧日志 diff 14 列指标，确认零漂移

## 关联场景
所有 4 个正式场景 + 消融开关均不受影响（纯常量重构）。

## 备注
本次不改默认值、不新增任何调用点，仅搬家。
