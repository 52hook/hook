---
description: 运行 GRIPC 对比实验和消融实验的完整流程
---

# 实验执行流程

## 前置条件
- CARLA 0.9.16 服务已启动（端口 2001）
- conda 环境 `first1` 已激活
- 工作目录：`~/phd/0.9.16udmc_carla-main myfinal`

## 实验组结构

### A) 核心场景深入对比（含消融）
- 场景：`unsig_s1`, `unsig_s2`, `unsig_s3`, `round_s1`, `cross_s1`
- 方法：GRIPC（全开）、UDMC（全关）、FSM+PID、4 种消融
- 每个配置重复 3 次

### B) 通用性验证
- 场景：`multilaneACC`, `roundabout`, `mixed_traffic`
- 方法：GRIPC、UDMC、FSM+PID
- 每个配置重复 3 次

## 执行方式

### 一键执行全部实验
```bash
conda activate first1
cd ~/phd/0.9.16udmc_carla-main\ myfinal
bash run_experiments.sh
```

### 单场景手动测试
```bash
// turbo
conda activate first1
cd ~/phd/0.9.16udmc_carla-main\ myfinal
CARLA_PORT=2001 python udmc_main.py unsig_s1 False
```

### 消融实验（单次）
```bash
# w/o OBCA
CARLA_PORT=2001 USE_DUAL=0 python udmc_main.py unsig_s1 False

# w/o 对向车检测
CARLA_PORT=2001 ABLATION_OPP=0 python udmc_main.py unsig_s1 False

# w/o IGPR 方差驱动
CARLA_PORT=2001 ABLATION_IGPR=0 python udmc_main.py unsig_s1 False

# w/o B-spline 平滑
CARLA_PORT=2001 ABLATION_BSPLINE=0 python udmc_main.py unsig_s1 False
```

### FSM baseline
```bash
// turbo
CARLA_PORT=2001 python "fsm_main(baseline).py" unsig_s1 False
```

## 消融开关速查表

| 环境变量 | 默认值 | 含义 |
|---------|-------|------|
| `USE_DUAL` | `1` | OBCA 多边形对偶约束 |
| `ABLATION_OPP` | `1` | 对向车检测与让行 |
| `ABLATION_IGPR` | `1` | IGPR 方差驱动 APF 权重 |
| `ABLATION_BSPLINE` | `1` | B-spline 全局参考线平滑 |

值为 `0` 表示关闭该模块，值为 `1`（默认）表示开启。

## 结果输出
- 日志文件：`experiment_results/{方法}_{场景}_r{重复}.txt`
- MPC 求解时间：`computing_time.txt`
- 势场/控制数据：`data/` 目录

## 仿真失败处理
1. 超时（timeout 300s）：自动跳过，日志中会记录
2. CARLA 崩溃：需手动重启 CARLA 后重跑
3. 碰撞/违规：由脚本自动记录，不中断
