---
description: 启动 CARLA 并运行单个仿真场景
---

// turbo-all

## 步骤

1. 检查 CARLA 是否已运行
```bash
pgrep -f CarlaUE4 && echo "CARLA 已运行" || echo "CARLA 未启动"
```

2. 如果未运行，启动 CARLA
```bash
cd ~/CARLA_0.9.16 && nohup ./CarlaUE4.sh -carla-rpc-port=2001 -quality-level=Low &
```

3. 等待 CARLA 就绪
```bash
for i in $(seq 1 30); do python3 -c "import carla; c=carla.Client('localhost',2001); c.get_server_version()" 2>/dev/null && echo "就绪" && break; sleep 1; done
```

4. 激活 conda 环境并运行仿真
```bash
cd ~/phd/0.9.16udmc_carla-main\ myfinal && conda run -n first1 --no-banner CARLA_PORT=2001 python udmc_main.py <场景名> <True/False>
```

将 `<场景名>` 替换为具体场景：`unsig_s1`, `unsig_s2`, `unsig_s3`, `round_s1`, `cross_s1`, `roundabout`, `crossroad`, `multilaneACC`, `unsig_crossroad`, `mixed_traffic`。

将 `<True/False>` 替换为是否随机生成周车。对比实验用 `False`。
