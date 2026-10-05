# 图 2 (c) 改为切角对比，并新增分段统计

## 修改概述
审稿视角下，原 (c) 面板只画融合线相对原始线的偏离和 0.35 m 位移上限，既说明不了融合的效果，"低于名义上限"的比较也没有意义。改为同时画 B 样条（未融合）与融合参考线相对原始路线的横向偏离，直接展示融合对切角的抑制。同时在 `--stats` 中加入按基准线曲率分段的统计，保证论文中引用的数字能从仓库复现。

## 修改详情（paper_figures/make_carla_figures.py）
- `_fig2_draw` 的 (c) 面板：
  - 原值：融合线横向偏离（实线）+ 逐点 |blend−raw|（虚线）+ 0.35 m cap 水平线，ylim 固定为 0.45
  - 新值：B 样条横向偏离（蓝）+ 融合线横向偏离（橙），ylim 为 max(0.45, 1.15 × B 样条最大偏离)；纵轴标签改为 "Lateral deviation from raw course (m)"
- 新增 `segment_stats(route, v, step_m)`：以 ALPHA_BS_KAPPA0 为界划分低/高曲率段，剔除折返点及其前后各 2 个采样；统计
  - 曲率沿弧长变化率 |dκ/ds| 的 RMS（raw / B 样条 / 融合）
  - 最大横向偏离（B 样条 / 融合）
- `stats_report`：打印新增的 (v) 分段统计行，并写入 JSON 的 `segments` 字段

## 修改原因
用户确认按"保留正文说法、用与说法对应的指标证明效果"的方案修改（2026-10-05）。

## 验证
见本次会话：重新生成 latex/figures/fig2_bspline_method.pdf；`--stats` 输出与 scratchpad 预计算一致。

## 关联场景
roundabout（图 2 窗口）；统计覆盖 unsig_crossroad / roundabout / multilaneACC / mixed_traffic。

## 备注
GitHub 推送鉴权失败（token 失效），经用户同意，本次只做本地 commit，不 push。
