# GRIPC：面向城市自动驾驶的几何-规则集成规划与控制框架

> **G**eometry-**R**ule **I**ntegrated **P**lanning and **C**ontrol for Urban Autonomous Driving with B-Spline Reference Lines and Hybrid Polygonal Collision Avoidance

本框架面向复杂城市道路场景，将行为决策、轨迹规划与车辆控制统一建模为有限时域最优控制问题（OCP），通过几何精确建模与交通规则显式编码，在安全性、效率、舒适性和实时性之间实现整体权衡。仿真环境基于 **CARLA 0.9.16**。

---

## 核心技术贡献

### 1. 约束 B-spline 全局参考线平滑

在 A\* 车道级拓扑路径之上，以 B-spline 控制点为决策变量构造二次规划（QP）子问题，将拟合精度、曲率约束与车道边界同时纳入约束集合，生成具有 C² 连续性且曲率可控的全局参考线，为下游 MPC 提供高质量的几何基准。

### 2. 显式参数化行为规则势场层

提出一套结构统一的规则势场编码机制，将车道保持、限速、跟车安全、交通信号与行人礼让等交通规则统一转化为低维特征向量中的可微标量惩罚项，在统一优化问题中实现隐式的行为决策与规划，显著提升对不同城市驾驶场景的适应性。

### 3. 混合多边形避障约束

利用凸优化强对偶理论，将多边形不相交几何条件重构为一组连续可微的代数约束，通过引入对偶变量与非负松弛因子构建软硬协同机制。在消除传统圆盘近似保守性的同时，为非线性优化提供高质量梯度信息，显著提升复杂环境下的几何安全性。

### 4. 混合 QP 热启动求解策略

针对含多边形碰撞约束与复杂势场代价的非凸 OCP，构造基于线性化动力学简化与顶点-半空间碰撞近似的凸 QP 预规划模块，快速生成局部安全轨迹作为主 NLP 初始解，大幅降低平均求解时间并提升迭代稳定性。

---

## 支持场景

| 场景 | 关键字 |
|---|---|
| 多车道自适应巡航 | `multilaneACC` |
| 环岛 | `roundabout` |
| 有信号路口 | `crossroad` |
| 无信号路口 | `unsig_crossroad` |
| 混合交通（含行人） | `mixed_traffic` |

---

## 快速开始

### 环境要求

| 项目 | 版本 |
|---|---|
| Ubuntu | 22.04.5 LTS |
| Python | 3.10（Conda 环境 `first1`） |
| CARLA | 0.9.16 |
| CasADi | 3.6.3 |

其余 Python 依赖见 `requirements.txt`。

### 启动步骤

**1. 激活环境**

```shell
conda activate first1
```

**2. 启动 CARLA 服务端**

```shell
cd ~/CARLA_0.9.16
./CarlaUE4.sh -carla-rpc-port=2001
```

**3. 运行仿真**

```shell
cd ~/phd/0.9.16udmc_carla-main\ myfinal
CARLA_PORT=2001 python udmc_main.py roundabout True
```

最后一个参数为 `True` 时随机生成周车；传 `False` 则从 `spawnPoints/` 读取固定初始条件，用于对比实验复现。

---

## 常用命令

### 场景仿真

```shell
CARLA_PORT=2001 python udmc_main.py multilaneACC True
CARLA_PORT=2001 python udmc_main.py roundabout True
CARLA_PORT=2001 python udmc_main.py crossroad True
CARLA_PORT=2001 python udmc_main.py unsig_crossroad True
CARLA_PORT=2001 python udmc_main.py mixed_traffic True
```

### FSM 基线对比

```shell
CARLA_PORT=2001 python "fsm_main(baseline).py" crossroad True
```

### 车辆动力学参数辨识

```shell
# 若需重新采集辨识数据
python carla_nl_system_identification.py

# 运行参数辨识
python param_est_using_slsqp.py
```

### 周车轨迹预测模型训练（IGPR）

```shell
python IGPR_predict_sv_wps.py
```

---

## 仓库结构

```
├── udmc_main.py                       # 主入口，选择场景并启动仿真
├── fsm_main(baseline).py             # 有限状态机基线，用于对比实验
├── IGPR_predict_sv_wps.py            # IGPR 周车轨迹预测模型训练
├── param_est_using_slsqp.py          # 车辆动力学参数辨识（SLSQP）
├── carla_nl_system_identification.py # 动力学辨识数据采集
├── mixed_traffic_scenario_generation.py  # 混合交通场景生成
│
├── scripts/
│   ├── env.py                         # CARLA 连接、车辆生成、仿真环境封装
│   ├── vehicle_obs.py                 # 核心逻辑：规则势场、多边形约束、状态优化
│   ├── x_v2x_agent.py                # 主控代理：ACC、超车、停车等行为决策
│   ├── vehicle_v2x_obs.py            # V2X 约束观测扩展
│   ├── others_agent.py                # 周车行为控制（跟车、变道）
│   ├── x_agent.py                     # 基础代理
│   └── lti_v2x.py                     # LTI 系统模型
│
├── utils/                             # 工具函数（插值、几何、多项式、平滑、绘图）
├── official/                          # 基于 CARLA 官方示例的改动代码
├── data/                              # 仿真输出（势场值、控制量、图表）
├── spawnPoints/                       # 固定初始点（非随机复现模式）
├── gpr.pkl                            # 训练好的 GPR 预测模型
├── lstm.pkl                           # 训练好的 LSTM 预测模型
└── requirements.txt                   # Python 依赖
```

---

## 关键配置项

`udmc_main.py` 中的主要全局参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `USE_DUAL` | `True` | 启用多边形对偶碰撞约束（GRIPC 核心特性） |
| `simu_step` | `0.05` | 仿真步长（秒） |
| `target_v` | `35` | 目标速度（km/h，因场景而异） |
| `veh_num` | `100` | 随机生成的周车数量 |

---

## 输出数据

| 路径 | 内容 |
|---|---|
| `data/` | 势场值、控制输入量、仿真过程图表 |
| `spawnPoints/` | 用于复现实验的周车初始点 |
| `computing_time.txt` | 求解耗时统计 |
| `{场景}.txt` | 各场景仿真运行日志 |

---

## 致谢

- [CasADi](https://web.casadi.org/) — 非线性优化求解器
- [CARLA](https://carla.readthedocs.io/en/0.9.16/) — 开源自动驾驶仿真平台

`official/` 目录包含基于 CARLA 官方示例修改的代码，相关第三方文件中已有的版权和许可证声明仍然有效。

---

## 联系方式

维护者：[@52hook](https://github.com/52hook)
