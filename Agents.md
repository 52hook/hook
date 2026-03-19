---
trigger: always_on
---

每次都叫我帅哥

## 项目级规则

### 语言规范

- **代码注释**：全中文
- **文档标题与说明**：全中文
- **变量名 / 函数名 / 文件名**：保留英文（机器读取，不翻译）

### Git Commit 规范

格式：`类型: 描述`，描述 ≤ 15 字，全中文。

| 类型 | 适用场景 |
|---|---|
| `新增` | 新功能、新文件 |
| `修复` | Bug 修复 |
| `优化` | 性能或代码质量改进 |
| `重构` | 不改变功能的结构调整 |
| `移除` | 删除冗余代码或文件 |

示例：✅ `优化: 减少 GPR 预测延迟` ❌ `feat: optimize gpr`

### CARLA 环境检查

本仓库固定对应 **CARLA 0.9.16**，不要与 `~/phd/udmc_carla-main myfinal`（0.9.14 仓库）混用。

- 0.9.16 代码仓库：`~/phd/0.9.16udmc_carla-main myfinal`
- Python 环境：`conda activate first1`
- CARLA 服务端目录：`~/CARLA_0.9.16`
- 默认端口：`2001`

修改 `scripts/` 目录下的任何文件前，请提醒用户确认 **0.9.16** 服务已启动，且客户端连的是正确端口：

```shell
conda activate first1
cd ~/CARLA_0.9.16
./CarlaUE4.sh -carla-rpc-port=2001
```

运行本仓库脚本时，默认使用：

```shell
CARLA_PORT=2001
```

0.9.16 典型调用方式：

```shell
conda activate first1
cd ~/phd/0.9.16udmc_carla-main\ myfinal
CARLA_PORT=2001 python udmc_main.py crossroad True
```

### 版本对应关系

0.9.14 对应信息如下，供切换版本时参考：

- 0.9.14 代码仓库：`~/phd/udmc_carla-main myfinal`
- Python 环境：`conda activate first`
- CARLA 服务端目录：`~/CARLA_0.9.14`
- 默认端口：`2000`

0.9.14 服务端启动方式：

```shell
cd ~/CARLA_0.9.14
./CarlaUE4.sh -carla-rpc-port=2000
```

0.9.14 典型调用方式：

```shell
conda activate first
cd ~/phd/udmc_carla-main\ myfinal
CARLA_PORT=2000 python udmc_main.py crossroad True
```

### 核心文件保护

以下文件改动风险较高，**修改前必须向用户提请审核**，说明改动意图和影响范围：

- `scripts/vehicle_obs.py` — UDMC 核心势场与约束逻辑
- `scripts/x_v2x_agent.py` — 主控决策逻辑（ACC、超车、停车）

### 数据文件约定

`.npy` 和 `.pkl` 为二进制数据/模型文件，**不直接内联编辑**。如需更新：

- 模型文件（`gpr.pkl` / `lstm.pkl`）→ 重新运行对应训练脚本
- 数据文件（`*.npy`）→ 重新运行 `carla_nl_system_identification.py`
