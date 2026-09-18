# 2026-09-18 图 2 窗口改为环岛入口前 30 m 并删除插图

## 修改概述
论文图 2（曲率调度参考线融合示意）默认窗口由自车在环岛入口前 15 m 改为 30 m，并删除 (a) 面板中整条路线的插图。

## 修改详情
- `paper_figures/make_carla_figures.py`
  - `Ctx.fig2_back_m` 默认值 15.0 → 30.0；`--fig2-back-m` argparse 默认 15.0 → 30.0；模块 docstring 与 `fig2_bspline` docstring 同步改为 30 m。
  - `_fig2_draw`：删除 (a) 面板的 `inset_axes` 插图块（整条路线灰线 + 处理段朱红 + 视窗框 + 起终点标记），`_inset_bounds` 辅助函数保留未调用。

## 修改原因
- 15 m 窗口的 31 个队列路点向前延伸约 80 m，包含环岛出口处路径规划器的接缝：原始路点先走 10.9 m，折 110° 走 1.43 m，再折 107°。B 样条可抹平（|κ| 0.15），但融合参考线受 0.35 m 位移上限约束只能贴原始路点，保留两个近 90° 折角（|κ| 3.1），在说明平滑效果的图里误导读者。该折角距自车约 70 m，队列清理阈值 10.6 m、MPC 时域约 5.5 m，对控制无影响。
- 30 m 窗口（弧长 60 m）覆盖入口缓弯、汇入弯与环内前段，无折角：α_eff 0.45→0.15，融合线横向偏离最大 0.125 m、逐点 0.206 m，未融合 B 样条最大 0.795 m。
- 插图视觉冗余，用户要求删除。

## 验证结果
- `python make_carla_figures.py --only fig2_bspline_method --no-wait` 重出 `latex/figures/fig2_bspline_method.pdf`，日志：25 waypoints after dedup (7 removed)，spline 151 pts = 60.0 m，contract issues: none。
- 论文重新编译 0 error。

## 关联场景
roundabout（图 2 仅用该场景）。

## 备注
论文正文与图注同步改为新窗口数字（tex 不入 git）。改前 PDF 备份于会话 scratchpad。

## 补记（同日）图例移入 (a) 面板
- `_fig2_draw`：整图底部 `fig.legend` 改为 `ax_a.legend(loc="lower right")`，字号 5.6，白底无边框；gridspec `bottom` 0.235 → 0.16。
- 重出 `fig2_bspline_method.pdf`，图例位于 (a) 右下空白区，不遮挡路径；论文重编译 0 error。
