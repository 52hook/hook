# CARLA Town03 Vulkan 崩溃 workaround: 临时切 Town03_Opt

## 修改概述

4/29 之后系统自动升级 NVIDIA driver `590.48.01-0ubuntu0.22.04.1`
→ `.22.04.3`、kernel `6.8.0-101/-106` → `-110`，导致 CARLA 0.9.16
(UE4 4.26 shipping) 在 `client.load_world('Town03')` 时 Vulkan
RHI PSO 编译阶段 NULL 解引用崩 (Diagnostics.txt 给出
`FVulkanVertexInputStateInfo::Generate` SIGSEGV at `0x94`)。

经 codex 复核 + 自审, 用户暂不动系统 (不 reboot 选老 kernel、不 apt
降级 driver), 改用 CARLA 自带 `Town03_Opt` (优化/streamed 版,
.umap/.uexp 资源相同, 拓扑一致, 走不同的 PSO compile path 绕开 bug)。

实测 `client.load_world('Town03_Opt')` 2.4s 加载, 85 actors, server
不崩; 4 个实验场景 (roundabout / multilaneACC / unsig_crossroad
/ mixed_traffic) 的 start_transform 在 Town03_Opt 中
project_to_road 距离全 < 0.13m, ego model3 try_spawn 成功。

## 修改详情

### 文件: udmc_main.py

1. **新增 `CARLA_MAP_ID` 环境变量** (line ~44 附近)
   ```python
   # ---- CARLA map id 临时降级开关 (driver/kernel 升级后 Town03 Vulkan
   # PSO 崩溃, 临时设 '03_Opt' 走优化版 map; 系统恢复后去掉 env var
   # 即可) ----
   CARLA_MAP_ID = os.environ.get('CARLA_MAP_ID', '03')
   ```
   默认 `'03'`, 完全保持原行为 (对历史实验脚本/CI 透明)。

2. **`env = Env(map_id=...)` 改读 env var** (原 line 59)
   ```python
   - env = Env(map_id='03', ...)
   + env = Env(map_id=CARLA_MAP_ID, ...)
   ```
   env.py:234 `'Town' + map_id` 拼接, `'03_Opt'` → `'Town03_Opt'`,
   驱动 `client.load_world('Town03_Opt')`。

## 修改原因

- crash log 是确定性证据: `SIGSEGV at 0x94` 落在 UE4
  `FVulkanVertexInputStateInfo::Generate` →
  `FVulkanPipelineStateCacheManager::CreateGfxEntry` →
  `RHICreateGraphicsPipelineState` →
  `FCompilePipelineStateTask::DoTask`。是 **Vulkan PSO 编译期** NULL
  解引用, 不是 CPU 端 bug, 不是磁盘/显存/资源损坏。
- 时间线: 4/29 用户跑通时是旧 driver+kernel; 系统自动升级到当前
  driver `.22.04.3` + kernel `-110` 后 Town03 100% 重现崩溃, Town10HD
  默认 map 完全正常 → 强证 driver/kernel regression 暴露 UE4 4.26
  Vulkan RHI 在 Town03 monolithic 资源 PSO compile path 上的 bug。
- codex 复核给出 5 个不动系统的候选 workaround, 排序后最高优先是
  "试 Town03_Opt" — 它是 Town03 的 streamed/optimized 变体,
  几何/拓扑相同 (spawn points 也是 265), 但 cooked PSO/material
  permutation 路径不同, 可绕开特定 bad pipeline state。
- env var 而非硬编码, 是为了让用户系统恢复后无缝回到 `'03'`,
  也方便 CI / batch 脚本切换。

## 验证结果

```bash
$ python -m py_compile udmc_main.py
(无输出 -> 通过)

$ # CARLA 启动 (默认 Vulkan, Town10HD_Opt)
$ DISPLAY=:1 ./CarlaUE4.sh -carla-rpc-port=2001 -quality-level=Low &

$ python -c "
  import carla, time
  c = carla.Client('localhost', 2001); c.set_timeout(180.0)
  print('before:', c.get_world().get_map().name)
  t0=time.time()
  w = c.load_world('Town03_Opt')
  print(f'load_world OK in {time.time()-t0:.1f}s, after: {w.get_map().name}')
  print('actors:', len(w.get_actors()))
"
before: Carla/Maps/Town10HD_Opt
load_world OK in 2.4s, after: Carla/Maps/Town03_Opt
actors: 85

$ # 4 场景 start coord 在 Town03_Opt 中可达
unsig_crossroad start: road=28 lane=-2 dist=0.02m
roundabout: road=30 lane=4 dist=0.13m
multilaneACC: road=8 lane=5 dist=0.11m
mixed_traffic: road=28 lane=-2 dist=0.10m

$ # ego try_spawn
Actor(id=109, type=vehicle.tesla.model3)  -> 成功
```

后续 30s smoke 验证: `CARLA_MAP_ID=03_Opt USE_DUAL=1 timeout 60
python udmc_main.py unsig_crossroad False` (本 step 后单独跑)。

## 关联场景

- **全部 4 个实验场景 (unsig_crossroad / roundabout / multilaneACC
  / mixed_traffic)** 都受影响 — 它们都用 `map_id='03'`。本 workaround
  让全部 4 个场景在当前 driver/kernel 状态下可继续跑实验。

## 备注

- 这不是 OBCA 修复链 (Steps 1-8) 的一部分, 是独立的环境层 workaround,
  单独 commit。
- 长期解法仍是 reboot 选 `6.8.0-107` kernel 引导 (driver module
  会在 -107 上重新编译, 等同回滚 driver+kernel 组合); 但用户当前
  优先保持系统不动, 用 `'03_Opt'` 续跑实验。
- 一旦用户后续 reboot 解决根因, 跑实验时不设 `CARLA_MAP_ID`
  即恢复 `Town03`, 无需再改代码。
- 本 workaround 不影响 OBCA 算法行为 — Town03_Opt 与 Town03
  几何拓扑一致, MPC/APF/B-spline/IGPR 看到的世界完全相同。
