# CARLA 实景类插图脚本说明（`make_carla_figures.py`）

## 运行

```bash
source ~/anaconda3/etc/profile.d/conda.sh && conda activate first1
cd paper_figures
python make_carla_figures.py                         # 全部：等 DUMP_DONE / CAPTURE_DONE 后出图
python make_carla_figures.py --only fig2_bspline     # 只出某一张（逗号分隔可多张）
python make_carla_figures.py --only fig6_keyframes,fig8_timeseries --no-wait
python make_carla_figures.py --seeds roundabout=74 --udmc-seeds roundabout=74 --suffix _alt --only fig8_timeseries
python make_carla_figures.py --only fig2_bspline --fig2-back-m 20 --fig2-window-v 8   # 换自车位置 / 清理车速
python make_carla_figures.py --check-proj /tmp/proj  # 把路线叠到 overview.png 上核对相机投影
```

输出经 `style.savefig`：`latex/figures/<stem>.pdf` + `paper_figures/preview/<stem>.png`。

| stem | 内容 | 数据 |
|---|---|---|
| `fig2_bspline` | 参考线平滑（环岛，**单步 31 路点窗口**，自车在入口前 15 m）：俯视 / 曲率（线性，裁剪 ±0.4，越界段用三角标注并在图例计数）/ α_eff 与横向偏差 | 路线转储 |
| `fig2_bspline_int` | 同上，无信号路口左转窗口（自车在转弯前 15 m） | 路线转储 |
| `fig2_bspline_mul` | 同上，多车道直道窗口（`--fig2-mul-s`，默认 s = 40 m；α ≈ 0.5 区间） | 路线转储 |
| `fig2_bspline_full` | 同上，但整条路线一次性处理（对照用） | 路线转储 |
| `--stats` | 不出图：四场景每 5 m 一窗的参考线质量统计（\|κ\| max/p95、折角数/100 m、横向摆动、混合线偏差），并写 `preview/fig2_stats.json` | 路线转储 |
| `fig5_scenarios` | 四场景总览（overview.png 裁剪 + 起终点 + 行人路径） | 采集 |
| `fig6_keyframes` | 四场景 × 四关键帧 BEV | 采集 |
| `fig7_trajectories` | 环岛 / 路口：GRIPC（按速度着色）vs UDMC 轨迹 | 采集 |
| `fig8_timeseries` | 环岛五通道时序 GRIPC vs UDMC | 采集 |

## 种子与备用种子

- 主种子默认 `--seeds roundabout=60,unsig_crossroad=83,multilaneACC=82,mixed_traffic=69`，
  `--udmc-seeds roundabout=60,unsig_crossroad=83`；tag = `<METHOD>_<scenario>_s<seed>`。
- 备用种子默认 `--alt-seeds roundabout=74,unsig_crossroad=75,multilaneACC=72,mixed_traffic=84`，
  `--alt-udmc-seeds roundabout=74,unsig_crossroad=75`。主图完成后若存在
  `experiment_results/fig_capture/CAPTURE_BACKUP_DONE`（或 `--alt force`），自动追加输出
  `fig6_keyframes_alt` / `fig7_trajectories_alt` / `fig8_timeseries_alt`。

## 数据契约

- A) `experiment_results/fig_routes/<scenario>.npz`（`tools/dump_routes.py` 产出）+ `DUMP_DONE`。
- B) `experiment_results/fig_capture/<tag>.txt`（16 列，列义见 `style.LOG_COLS`）、
  `<tag>_fig.npz`（sidecar，键见脚本 `SIDECAR_KEYS`；`meta` 为 JSON 字符串）、
  `images/fig_capture/<tag>/bev_<step:06d>_t<t:.2f>.png` 与 `overview.png` + `CAPTURE_DONE`。
- 缺键 / 形状不符 / 帧缺失一律打印 `[CONTRACT] ...` 并在结尾汇总，不做静默兜底。

## 坐标与相机

- 图内坐标一律为智能体右手系：`x_rh = x_carla`，`y_rh = -y_carla`。
- BEV 相机 `pitch=-90, yaw=0` ⇒ 图像上 = `+x_carla`，右 = `+y_carla`；脚本把图像顺时针转 90°
  以对齐右手系（右 = `+x`，上 = `-y`），使 Fig.5/6 与 Fig.2/7 同向。米/像素 = `2·Z·tan(fov/2)/1024`。
  可用 `--bev-up/--bev-right` 改取向，`--no-rotate` 关闭旋转。

## Fig.2 与智能体流水线的关系

`scripts/x_v2x_agent.py:656-792` 的每一步都被逐行复刻（常量旁标了 file:line）。
主版本 `fig2_bspline` 严格模拟某一步：自车放在 dense 曲率首次 > 0.03 1/m 的入口前 15 m
（`--fig2-back-m` / `--fig2-kappa-thr`），队列清理阈值 `5 + 0.5 v`（v 默认取场景目标车速，`--fig2-window-v` 可改），
取 31 个前瞻路点、去重、样条、B 样条、Route-Z α、tanh 上限、7 点滑动平均、混合、重算曲率。
对照版本 `fig2_bspline_full` 把整条路线一次性送入。终端同时打印四场景每 5 m 一窗的滚动统计
（最近点 |blend − raw| 的 RMS / max / >0.10 m 比例，|κ| 的 max / p99 / 中位数）。
CARLA `GlobalRoutePlanner` 路线在路口只给入口/出口路点（环岛内 11–13 m 长弦）且路段边界有 0.9–1.4 m 回折，
原版样条在这些接缝处曲率尖峰，混合参考线（横向修正 ≤ 0.35 m）无法消除；(b) 用 symlog 轴同时显示环道曲率
（~0.05 1/m）与尖峰，不裁剪。这是智能体同样会遇到的真实输入，脚本不做清洗。
