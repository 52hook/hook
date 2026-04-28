# A 路径：CollisionSensor 接入与受控实验

**日期**：2026-04-28 18:12:16
**关联提交**：待 push（备份点 `0c06dc9` → A 路径工程改动）
**关联讨论**：本会话 Plan v4，A 路径=OBCA 受控实验（GRIPC vs GRIPC_noOBCA，paired-seed）

## 修改概述

为论文 OBCA 章节（[latex/cas-dc-template.tex:451](latex/cas-dc-template.tex#L451) 第 4.3 节）补受控实验数据：在 CARLA 中接入 `sensor.other.collision`，逐 tick 与车辆控制日志对齐，输出第 16 列 `collision_flag`，专用于 `unsig_crossroad` 左转场景下 GRIPC（OBCA on）vs GRIPC_noOBCA（OBCA off）的同种子对比。

## 修改详情

### 1. [scripts/env.py](scripts/env.py)
- **新增**：CarlaEnv 内 `_collision_queue: queue.Queue` + `_collision_sensor: carla.Sensor`
- **新增**：`_attach_collision_sensor(ego_actor)` 方法——`world.spawn_actor(sensor.other.collision, transform=carla.Transform(), attach_to=ego_actor)`，listen 回调把事件按 `event.frame` 入队
- **新增**：`consume_collisions(target_frame:int) -> int`——出队所有 `event.frame <= target_frame` 的事件，过滤 `event.other_actor.type_id` 不以 `vehicle.` 开头的（行人/路灯/护栏可单独标记）
- **新增**：`_sensor_list` 与 `clean_all_actors()` 中按 `actor.is_listening` / `type_id == 'sensor.other.collision'` 析构
- **新增**：TrafficManager 初始化使用 `client.get_trafficmanager(8000)` 并 `tm.set_random_device_seed(seed)` + `tm.set_synchronous_mode(True)`
- **风险**：sensor lifecycle 必须在 `world.tick()` 之前 attach，且销毁早于 ego；TM 端口 8000 须确认未被占用

### 2. [udmc_main.py](udmc_main.py)
- **修改**：主循环 `world.tick()` 后立即 `flag = env.consume_collisions(world_snapshot.frame)`
- **修改**：把 `flag`（0/1/2/3 → 无/车辆/行人/静态物）传给 `agent._log_data` 作为第 16 列
- **修改**：异常分支退出时 `env.clean_all_actors()` 必须释放 collision sensor

### 3. [scripts/x_v2x_agent.py](scripts/x_v2x_agent.py)
- **修改**：`_log_data` 签名增加 `collision_flag: int = 0`
- **修改**：`logger.write` 列数 15 → 16，新列写入 `collision_flag`

### 4. [extract_metrics.py](extract_metrics.py)
- **修改**：`extract_single` 增加 14/15/16 列兼容（`len(line)` 分支）
- **新增**：`collision_count = sum(1 for f in flags if f > 0)` 与 `collision_rate = collision_count / total_frames`
- **新增**：CSV 输出新列 `collision_count`、`collision_rate`、`collision_with_vehicle`

### 5. [utils/_smoke_collision.py](utils/_smoke_collision.py) （新建）
- 单场景 unsig_crossroad，同 seed 跑一次，验证：
  - sensor 能 spawn 且无 spawn_actor 异常
  - log 文件第 16 列存在且 dtype 为 int
  - clean_all_actors 后 `world.get_actors().filter('sensor.other.collision')` 为空

### 6. [run_a_path.sh](run_a_path.sh) （新建）
- 20 seeds × 2 labels（GRIPC / GRIPC_noOBCA）= 40 次仿真
- 仅跑 unsig_crossroad
- 同 seed 配对，labels 顺序锁定，保证 CARLA 状态影响均摊

## 修改原因

- 现有 r2 报告 OBCA 广义 benchmark p>0.1，但 SCI大王 反映原 UDMC 在 unsig 左转有撞车现象，r2 数据未覆盖该单场景下"是否撞"
- 论文 4.3 节 OBCA 实验对比为空（Agent#2 调研已证），需要 CollisionSensor 实测数据填补
- paired-seed 设计排除 CARLA 随机性影响，比 aggregate p-value 更适合"存在性证明"

## 验证结果

待 smoke test 完成后填写：
- [ ] sensor spawn 成功率 100%（10 次连跑）
- [ ] log 文件第 16 列存在
- [ ] clean_all_actors 后 sensor 为 0
- [ ] paired-seed 输出 collision_rate(GRIPC) ≤ collision_rate(GRIPC_noOBCA)

## 关联场景

- 主：`unsig_crossroad`（无信号路口左转，UDMC 撞车原观察场景）
- 副：`mixed_traffic`（含行人，验证 collision_flag 分类）

## 备注

- TrafficManager 端口固定 8000，与 CARLA 默认端口 2001 独立
- sensor lifecycle 顺序：spawn ego → spawn sensor → tick loop → destroy sensor → destroy ego
- CLAUDE.md 第 0 节："论文内容严禁进入 git"——本次 changelog 与代码改动可 commit，但实验产生的 logs/ 与 experiment_results/ 不进 git
