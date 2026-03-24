---
description: 启动、检查、重启 CARLA 0.9.16 仿真服务
---

# CARLA 仿真服务管理

## 前置条件
- Ubuntu 22.04，CARLA 安装在 `~/CARLA_0.9.16`
- conda 环境 `first1`

## 检查 CARLA 是否运行

```bash
// turbo
pgrep -f CarlaUE4 && echo "CARLA 运行中" || echo "CARLA 未启动"
```

## 启动 CARLA 服务端

```bash
cd ~/CARLA_0.9.16
nohup ./CarlaUE4.sh -carla-rpc-port=2001 -quality-level=Low &
```

> 使用 `-quality-level=Low` 可减少 GPU 占用。笔记本 4070 (8GB) 建议始终使用 Low 模式。

## 等待 CARLA 就绪（最多 30 秒）

```bash
// turbo
for i in $(seq 1 30); do
    python3 -c "import carla; c=carla.Client('localhost',2001); c.get_server_version()" 2>/dev/null && echo "CARLA 就绪" && break
    sleep 1
done
```

## 重启 CARLA

```bash
pkill -f CarlaUE4 || true
sleep 3
cd ~/CARLA_0.9.16
nohup ./CarlaUE4.sh -carla-rpc-port=2001 -quality-level=Low &
```

## 注意事项
- CARLA 启动约需 10-15 秒
- 端口固定为 **2001**，切勿改为 2000（那是 0.9.14 的）
- 如果仿真卡死（MPC 超时），先 `pkill -f udmc_main` 再重启 CARLA
