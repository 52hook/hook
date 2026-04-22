# 2026-04-22 18:56 Route Z: B-spline α 曲率 sigmoid 逐点调度

## 修改概述

把 [scripts/x_v2x_agent.py](scripts/x_v2x_agent.py) 中 B-spline α 混合系数从"场景硬编码查表"（roundabout=0.2 / unsig_crossroad=0.3 / multilaneACC=0.4 / 其他=0.7）改为**曲率驱动的 sigmoid 逐点函数**：

```
α_i = α_max − (α_max − α_min) / (1 + exp(−β·(|κ_i| − κ0)))
```

- **α_max = 0.7**（直道充分平滑）
- **α_min = 0.2**（急弯贴原车道中心，避免 B-spline 切内道）
- **κ0 = 0.05 m⁻¹**（sigmoid 拐点曲率，转弯半径 ≈ 20 m）
- **β = 80 m**（陡峭度，过渡带约 ±0.025 m⁻¹）

## 修改原因

### 旧做法的痛点
场景查表 α 在方法论上站不住脚——审稿人会追问"α=0.4 是怎么取的？"，且论文里一旦写出"unsig_crossroad→0.3, multilaneACC→0.4"会被质疑过拟合评估场景。

### 顶刊文献依据（已通过本地 PDF 原文核对）

1. **Tarhini F., Talj R., Doumiati M.**, *Control Engineering Practice*, vol. 164, art. 106464, 2025 — Eq. (20a) 对 MPC 路径跟踪权重做 `q_μ,ρ = ζ₁·sigmoid(ρ) − ζ₂·sigmoid(μ) + ζ₃` 双 sigmoid 调度（SCI Q1，IF≈5.0）。
2. **Choi J., Chin H., Park H. 等**, *IEEE T-ITS*, vol. 26, no. 2, pp. 1797–1811, Feb. 2025 — Eq. (10) `J_fl = Σ w_i^fl · (K_i/κ_max)²` 以逐点曲率为驱动增加 flattening 权重（SCI Q1，IF≈8.5）。

两篇均为 **2025 年 SCI Q1**，共同支持"用曲率作为 α（或权重）的连续调度变量"的做法。本文 Route Z 把 sigmoid 直接作用到 α 本身，使结果是**几何内在量**而非场景标签。

## 修改详情

### 文件：[scripts/x_v2x_agent.py](scripts/x_v2x_agent.py)

#### (1) `__init__` 参数块（行 136–147 → 136–145）
**原**：
```python
self._alpha_bs_by_scenario = {
    'roundabout':      0.2,
    'round_s1':        0.2,
    'unsig_crossroad': 0.3,
    'multilaneACC':    0.4,
}
self._alpha_bs_default = 0.7
```

**新**：
```python
# B 样条曲率调度 α sigmoid（Route Z）
self._alpha_bs_max    = 0.7
self._alpha_bs_min    = 0.2
self._alpha_bs_kappa0 = 0.05
self._alpha_bs_beta   = 80.0
self._alpha_bs_default = 0.7   # bspline_adaptive=0 时回退常数
```

#### (2) α 计算块（行 446–456 → 445–459）
**原**：
```python
elif self._bspline_adaptive:
    alpha_bs = self._alpha_bs_by_scenario.get(
        self._scenario, self._alpha_bs_default
    )
```

**新**：
```python
elif self._bspline_adaptive:
    # Route Z: 曲率 sigmoid 逐点 α
    alpha_bs = self._alpha_bs_max - (
        self._alpha_bs_max - self._alpha_bs_min
    ) / (
        1.0 + np.exp(
            -self._alpha_bs_beta * (np.abs(ck0) - self._alpha_bs_kappa0)
        )
    )
```

红灯保持 `alpha_bs = 0.0`，`bspline_adaptive=0` 保持 `alpha_bs = 0.7`，向后兼容。

## 数值验证（sigmoid 形状）

| \|κ\| (1/m) | α_i    | 解读 |
|-------------|--------|------|
| 0.00        | 0.691  | 直道 → 近 α_max |
| 0.03        | 0.616  | 过渡 |
| 0.05 (κ0)   | 0.450  | 中点 |
| 0.07        | 0.284  | 过渡 |
| 0.10        | 0.209  | 环岛典型曲率 → 近 α_min |
| 0.15–0.20   | 0.200  | 急弯饱和 |

与三场景 A/B 经验值吻合：roundabout（κ≈0.08-0.15）→ α≈0.21-0.28，旧硬编码 0.2；unsig_crossroad 90° 左转（κ≈0.2）→ α→0.20，旧硬编码 0.3（Route Z 更激进）；multilaneACC 直道（κ≈0）→ α→0.69（比旧 0.4 更平滑，直道本就不需要贴原车道中心）。

## 验证计划

1. **语法检查**：`python -m py_compile scripts/x_v2x_agent.py` ✅ 已通过
2. **单场景烟雾测试**：`ABLATION_BSPLINE_ADAPTIVE=1 python udmc_main.py roundabout True`（≤10 步看 log 不报错）
3. **三场景 A/B 对比**：重跑 `scripts/ab_bspline_adaptive.sh {roundabout,unsig_crossroad,multilaneACC}`，对比旧查表版 vs Route Z 的 rmse_dist / rmse_raw / jerk / min_dist
4. **代码审查**：已通过 code-reviewer agent 独立审查

## 关联场景

- roundabout / round_s1（环岛汇入）
- unsig_crossroad（无信号路口左转）
- multilaneACC（多车道巡航）
- mixed_traffic（若含曲率路段也受益）

## 备注

- `_alpha_bs_by_scenario` 字典已整体删除（unused）。
- 消融开关 `ABLATION_BSPLINE_ADAPTIVE` 保留语义：0 = 固定 α=0.7，1 = Route Z sigmoid。
- 论文写作建议见 [changelogs/正文需要修改.md](changelogs/正文需要修改.md) 🔴 #3 节（非本次提交）。
