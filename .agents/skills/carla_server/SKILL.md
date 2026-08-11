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

### 默认：无窗口 OffScreen 启动（跑批必须用）

```bash
cd ~/CARLA_0.9.16
DISPLAY= nohup ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2001 \
  > /tmp/carla.log 2>&1 & disown
```

关键教训：
- `DISPLAY=` 前缀必须保留为空值；如果继承 `DISPLAY=:1` 再配 `-RenderOffScreen`，CARLA 容易 `Signal 11` 崩溃。
- 跑正式实验必须用 `-RenderOffScreen`，不要开窗口。
- 不要把 `pkill -f CarlaUE4` 和启动命令写在同一条 shell 里；`pkill -f` 会匹配到当前 command line 里的 `CarlaUE4.sh`，导致 shell 自杀，后续启动根本不会执行。
- 在 Codex/agent 环境中，普通 `nohup ... &` 有时不会保活。如果 `pgrep -af '[C]arlaUE4'` 看不到真实进程且 `/tmp/carla.log` 为 0 字节，就改用 PTY session 直接运行同一条无窗口命令：

```bash
cd ~/CARLA_0.9.16
DISPLAY= ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2001
```

> `-quality-level=Low` 可减少 GPU 占用，但本项目历史稳定跑批命令以 `DISPLAY= + -RenderOffScreen + 2001` 为准。

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
pgrep -af '[C]arlaUE4' || true
pkill -TERM -f '[C]arlaUE4' || true
sleep 3
pkill -KILL -f '[C]arlaUE4' || true
cd ~/CARLA_0.9.16
DISPLAY= nohup ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2001 \
  > /tmp/carla.log 2>&1 & disown
```

## 注意事项
- CARLA 启动约需 10-15 秒
- 端口固定为 **2001**，切勿改为 2000（那是 0.9.14 的）
- 如果仿真卡死（MPC 超时），先 `pkill -f udmc_main` 再重启 CARLA
- 如果用户说“风扇没转”、2001 端口不起、`/tmp/carla.log` 为空，优先怀疑启动命令没有真正执行或后台进程被 agent shell 清理，不要立刻改端口或降级 CARLA。
