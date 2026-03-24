---
description: 查看和编辑 GRIPC 核心文件的注意事项和代码地图
---

# GRIPC 核心代码导航

## 代码架构

```
udmc_main.py                  # 入口：场景选择、环境初始化、仿真循环
fsm_main(baseline).py         # FSM+PID baseline 入口
├── scripts/
│   ├── vehicle_obs.py        # ⚠️ 核心：APF 势场 + OBCA 多边形约束 + MPC OCP
│   ├── x_v2x_agent.py       # ⚠️ 核心：主控代理（ACC、超车、让行决策）
│   ├── env.py                # CARLA 连接、车辆生成、传感器
│   ├── vehicle_v2x_obs.py   # V2X 约束扩展
│   └── others_agent.py      # 周车行为（autopilot 模式）
├── utils/
│   ├── bspline_utils.py     # B-spline QP 平滑
│   ├── poly_collision.py    # 多边形碰撞约束工具
│   └── ...                  # 插值、绘图等辅助函数
└── run_experiments.sh        # 自动化实验脚本
```

## ⚠️ 高风险文件

修改以下文件前**必须先向用户确认**：

### `scripts/vehicle_obs.py`
- 势场定义（FV, FTTC, FNR, FCR, FTL, FPD）
- OBCA 多边形碰撞约束
- MPC 代价函数和约束构造
- IGPR 方差 → APF 权重映射

### `scripts/x_v2x_agent.py`
- 行为决策逻辑（跟车/超车/停车/让行）
- 对向车检测模块（`yield_for_opp` 相关）
- 消融开关的读取和传递

## 场景配置速查

| 场景 | 起点 | 终点 | target_v | 信号灯 |
|------|------|------|---------|--------|
| multilaneACC | (48.46, 7.32) | (225.20, 9.39) | 45 | — |
| roundabout / round_s1 | (1.69, 70.00) | (-6.85, 37.58) | 40 | — |
| crossroad / cross_s1 | (8.91, -96.98) | (-46.57, -139.27) | 25 | Red→Green |
| unsig_crossroad / unsig_s* | (8.91, -96.98) | (-46.57, -139.27) | 25 | 常绿 |
| mixed_traffic | (170.7, 94.3) | (126.4, 62.5) | 35 | 常绿 |

## 调试技巧

### 查看 MPC 求解时间
```bash
// turbo
tail -20 computing_time.txt
```

### 查看最近一次实验日志
```bash
// turbo
ls -lt experiment_results/ | head -5
```

### 检查势场输出
```bash
// turbo
ls -lt data/*.png | head -10
```

### 快速语法检查
```bash
// turbo
python3 -m py_compile udmc_main.py && python3 -m py_compile "fsm_main(baseline).py" && echo "OK"
```
