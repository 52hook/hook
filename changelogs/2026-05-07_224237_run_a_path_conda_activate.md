# 修复: run_a_path.sh 注入 conda activate

时间: 2026-05-07 18:5x (上一会话 34ab0978 实施, 本会话 22:42 补 commit)

## 修改概述

`run_a_path.sh` 在脚本开头注入 `conda activate first1`, 解决 nohup 起的
非登录 shell 不 source `~/.bashrc` 导致 conda 未激活、`python` 不在
PATH、`timeout 300 python ...` 立即返回 127、60 seed × 2 全程空跑
(每个 .log 仅 66 字节) 的根因。

## 修改详情

### 文件: `run_a_path.sh`

在 `log()` 函数定义之后、`run_udmc()` 函数定义之前, 新增 conda 激活块:

```bash
# 激活 conda 环境 first1
# nohup/sh 起的非登录 shell 不会 source ~/.bashrc, conda 没 activate,
# python 不在 PATH, timeout 直接报 "python: 没有那个文件或目录" 全程空跑.
# set -u 严格模式下 conda 脚本会踩 unbound var, 临时关闭 -u.
set +u
source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate first1
set -u
command -v python >/dev/null || { echo "[FATAL] python 未在 PATH, conda 激活失败"; exit 1; }
log "Python: $(command -v python) ($(python --version 2>&1))"
```

## 修改原因

上一会话 (34ab0978) 第一次起 7c 后发现:

- 每个仿真只跑 8 秒后立即退出
- 所有 .log 仅 66 bytes
- 内容: `timeout: 无法运行命令 "python": 没有那个文件或目录`
- 60 seed × 2 label = 120 次全部空跑, 白烧 ≈ 8 分钟

定位根因: `nohup bash run_a_path.sh ...` 起的 bash 是**非交互非登录** shell,
不会自动 source `~/.bashrc`, conda 没 activate, `python` 不在 PATH.
`run_udmc()` 内 `timeout 300 python ...` 立即失败 (exit 127), 但
`) || true` 把错吞掉, sleep 8 后跳下一个 seed.

## 验证结果

修复后 smoke 测试 (本地 ~12s 短跑):

- `Python: /home/jh/anaconda3/envs/first1/bin/python (Python 3.10.20)` ✓
- 子日志含 `INFO: Connecting to CARLA at localhost:2001` ✓
- 不再是 66B "python not found"

正式 7c 跑通: 18:57:35 启动, 21:31:20 完成, 60 seed × 2 = 120 次仿真,
共 229 文件输出 (47 GRIPC 完整 + 13 GRIPC timeout + 60 noOBCA 完整 + D2 2 件 + log).

## 关联场景

unsig_crossroad (A 路径 paired-seed 实验)

## 备注

- 同期还修复了 cwd 残留问题 (Step 2 `cd` 进子目录后没回根目录), 用绝对路径
  调用 `nohup bash /home/jh/phd/0.9.16udmc_carla-main_myfinal/run_a_path.sh`
  绕开. sh 本身已通过 `cd "${PROJECT_DIR}"` 自我矫正.
- 13 个 GRIPC seed timeout 失败是另一独立问题 (OBCA 在某些交通态势下
  仿真步数 1500-2400, 300s 不够), 由后续 `run_a_path_rerun.sh` 处理.
