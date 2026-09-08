# 2026-09-08 CARLA 插图采集链路定型与四张 CARLA 图入稿

## 修改概述
1. utils/carla_utils.py FigCaptureSession：BEV 相机加 sensor_tick=every·dt（FIG_SENSOR_TICK，默认开），只在存帧步渲染；回调改为按逐步登记表命名存帧；新增诊断开关 FIG_NO_DRAW / FIG_NO_CAM。
2. scripts/env.py、udmc_carla-main/scripts/env.py：FIG_TM_SYNC=1 时交通管理器同步并固定随机种子（诊断用，默认关闭，批次协议不变）。
3. tools/fig_capture_run.sh：每次运行前重启 CARLA，端口就绪后再等 20 s（地图加载）。
4. 新增 tools/fig_capture_batch.sh：按「方法:场景:seed」列表顺序采集并汇总每步墙钟与结果。
5. paper_figures/make_carla_figures.py：关键帧裁剪 768→600 px，自车位置加白色圆环标记。
6. 论文：插入图 3 场景俯视、图 5 关键帧、图 6 轨迹叠加、图 7 时间序列及图注与正文引用；舒适性段补 UDMC 转向抖动一句。编译 25 页 0 error。

## 根因记录
- 服务器（他人训练占用）上 GRIPC 普通模式 4/4 失败，不可用于出图。
- 本机反复失败根因：长时间运行的 CARLA 服务端累积状态与桌面负载；每集前重启 CARLA 后，GRIPC 环岛 60/路口 83/多车道 82/混合 84 与 UDMC 环岛 60/路口 83 全部干净到达，与正式批一致。混合 seed 69 三次死锁、seed 84 首次碰撞后重跑成功；图注只写实际使用的 seed。
- 采集模式每步 120–145 ms 也能成功，说明此前失败主要不是采集开销。

## 验证
tools/fig_capture_batch.sh 日志（batch_v3/v4/v5）逐集记录 rows/sim/vp/other/min_dist/step 时间；四张图目检；xelatex 0 error、图号连续 1–8。

## 备注
论文文件与 latex/figures 不入 git；experiment_results/fig_capture 与 images/fig_capture 为数据不入 git。

## 追加（03:55）：图 2 参考线方法示意入稿
- 用户决定 B 样条论述保持原样；参考线图采用「方法示意」版 fig2_bspline_method：(a) 环岛入口 31 点窗口俯视与走廊 (b) 曲率调度权重 α_i 与 α_eff (c) 融合线相对基准线的偏离与 0.35 m 上限；不含曲率对比面板。插入 §4.2 Route Z 段后，图号变为图 2，后续顺延至图 9；编译 25 页 0 error。
