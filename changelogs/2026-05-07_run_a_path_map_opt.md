# A 路径脚本注入 CARLA_MAP_ID, 配合 Town03_Opt 降级

## 修改概述

A 路径 OBCA 受控实验 (run_a_path.sh) 共 60 seed × 2 label = 120
次仿真, 每次都走 `client.load_world('Town03')`. 4/29 之后系统
driver/kernel 升级 (590.48.01-0ubuntu0.22.04.3 + kernel -110)
触发 UE4 4.26 Vulkan PSO 编译期 NULL 解引用, Town03 100% 重现
SIGSEGV; 必须改走 Town03_Opt 才能跑完 7c.

前置 commit f1ea647 已让 `udmc_main.py` 支持 `CARLA_MAP_ID`
环境变量, 本次把它注入到 `run_a_path.sh` 的两处.

## 修改详情

### 文件: run_a_path.sh

1. **全局配置默认值** (line ~31)
   ```bash
   # Town03 Vulkan PSO 崩溃 workaround: 默认走 Town03_Opt, 系统恢复后
   # 外部可 export CARLA_MAP_ID=03 回原行为
   CARLA_MAP_ID="${CARLA_MAP_ID:-03_Opt}"
   ```
   `${VAR:-default}` 模式: 外部已 export 时尊重外部值, 否则
   默认 `03_Opt`. 系统恢复后 `CARLA_MAP_ID=03 bash run_a_path.sh`
   一行回滚.

2. **`run_udmc()` 子 shell 环境透传** (line ~59)
   ```bash
   CARLA_MAP_ID="${CARLA_MAP_ID}" \
   ```
   插入位置紧跟 `CARLA_PORT`, 与其它 ABLATION_* 变量并列, 与
   现有风格一致.

## 修改原因

- 不改 7c 脚本: 跑出来 120 次全 SIGSEGV, 4 小时白费
- 在 udmc_main.py 改: 已 commit, 不再改一次
- 直接 hardcode `CARLA_MAP_ID=03_Opt` 到 sh: 系统恢复后还要改回来
- `${VAR:-default}` 让外部覆盖 + 默认绕过 = 现在就跑通 + 恢复后
  零代码改动回滚, 最干净

## 验证结果

```bash
$ grep -n CARLA_MAP_ID run_a_path.sh
31:# 外部可 export CARLA_MAP_ID=03 回原行为
32:CARLA_MAP_ID="${CARLA_MAP_ID:-03_Opt}"
59:        CARLA_MAP_ID="${CARLA_MAP_ID}" \
```

后续 Step D 单跑 `RANDOM_SEED=100 timeout 120 python udmc_main.py
unsig_crossroad True` 验证 map_id 透传链路 + 仿真自然 done.

## 关联场景

- A 路径 7c (60 seed paired) — 直接受影响, 不改 7c 必崩.
- 主对比 1120 次 (4 场景 × 3 方法 × 40 seed = 480) 与消融 (4×4×40=640)
  使用独立的 `run_experiments.sh`, 本次未改, 后续若启需同样注入.

## 备注

- 本 changelog 与 commit f1ea647 (Town03_Opt workaround) 是同一
  个工程问题的两次落地: 前者改 udmc_main.py 主入口, 本次改 7c
  专用脚本. 因为是两个独立"大修改", 按 §0 备份规则分两 commit.
- 系统 reboot 选 6.8.0-107 + driver -22.04.1 后, `unset
  CARLA_MAP_ID` 即恢复 Town03; 代码无需再改.
