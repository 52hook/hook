# 2026-09-09 UDMC 路点队列钳位补丁与重跑批 B

## 修改概述
论文插图审查时发现 UDMC 原版基线的 run 在距终点约 18 m 处以 `deque index out of range` 提前结束（采集运行实测：环岛提前 18.4 m、交叉口提前 17.4 m），而 GRIPC 以路点耗尽结束（距终点 0.2 到 4.2 m），FSM/FRENET/TCP 以距终点 5 m 结束。通行时间取日志末行时刻，UDMC 因此少跑约 18 m，通行时间偏短 1.4 到 2.6 s。用户决定给 UDMC 打钳位补丁后在服务器 40901 重跑 200 集（每格只跑一遍，不重试），并要求使用含防倒车补丁的 udmc_carla-main 版本。

## 修改详情
### A. `udmc_carla-main/scripts/x_v2x_agent.py`
- 新增 `_wq(idx)`：与主线 `scripts/x_v2x_agent.py::_wq` 同源的钳位访问，队列为空时抛 `No waypoints to follow`，下标越界时钳到末尾。
- `run_step` 内三处直接下标访问改为钳位访问：
  - `self._waypoints_queue[0][1].value` → `self._wq(0)[1].value`
  - `self._waypoints_queue[3][1].value` → `self._wq(3)[1].value`
  - `self._waypoints_queue[1][0]` → `self._wq(1)[0]`
- 越界机理：清理距离为 5 m + 0.5 v（约 9.5 m），路点间距 2 m，剩余不足 4 个路点即下标 3 越界，对应终点前 15 到 18 m。补丁只改访问方式，不改任何决策逻辑；防倒车补丁（`vehicle_obs.py` 速度下界 0）保持不变。

### B. 新增 `rerun_b_40901.sh`
- 由 `rerun_a_40901.sh` 派生：只跑 UDMC 4 场景 × 50 种子（seed = 42 + r），每格一遍不重试，无效产物记 `failed.tsv` 并删除；结果目录 `~/ljh/rerun_b_results`。
- 批开始先杀本端口旧 CARLA（服务器上 9 月 4 日批留下的进程已运行 5 天 16 小时）再离屏拉起，端口就绪后等 20 s；端口掉线自动重拉。
- 每集成功时把结束原因（`No waypoints to follow` 或 `deque index out of range`）写入控制台日志，便于核验补丁生效。

## 修改原因
统一各方法的结束判据，消除 UDMC 通行时间的系统性偏短；用户明确要求补丁重跑而非文字说明。

## 验证结果
（发批前后追加）

## 关联场景
unsig_crossroad、roundabout、multilaneACC、mixed_traffic 的 UDMC 臂。

## 备注
- 论文相关改动（表 1 UDMC 行、§5.1 结束判据、§5.2 数字、图 5 到图 8）待批次完成并经用户指令后进行，本日志只记代码改动。
- 本地出图与采集工作按用户要求暂缓，等待指令。

## 追加（同日）：诊断发现与仪表化
- 补丁后的服务器冒烟（环岛 seed 60）运行 101.3 s 后以 `trying to operate on a destroyed actor` 结束，而非路点耗尽。回查正式数据：该异常在所有 OCP 臂的长 run 中普遍出现，首次出现时刻集中在 93 到 111 s；UDMC 环岛慢峰 29 个 run 全部以此结束，混合 28 个、交叉口 12 个；GRIPC 各消融臂的超过 93 s 的 run 同样受影响。FSM、FRENET、TCP 不调用 `env.get_obs`，不受影响。
- 两套入口 `udmc_main.py` 的 `except Exception` 块加入 `PRINT_TRACEBACK=1` 门控诊断：打印回溯、`other_vehicles_list` 句柄存活数与世界中车辆数，用于定位周车被销毁的来源。默认关闭，不影响批次行为。
- 用户决定（同日）：`trying to operate on a destroyed actor` 一律判仿真失败，理由是自车长时间找不到可行路卡死，周车才会静止到被交通管理器清除；不在 env 中加句柄守卫（已写好的守卫改动撤销未提交）。`rerun_b_40901.sh` 据此把该异常的 run 记入 `failed.tsv`（原因列 `stuck_destroyed_actor`）并删除其日志文件，超时或无效产物记 `invalid_or_timeout`。
- 冒烟核验：钳位补丁版 UDMC 环岛 seed 60 于 20.3 s 以 `No waypoints to follow`（路点耗尽）结束，405 行 16 列，无碰撞；结束判据已与 GRIPC 一致。
- 服务器 ssh 自 17:30 起握手阶段持续被断（含 ControlMaster 复用尝试均失败），用户指示改本地跑：新增 `rerun_b_local.sh`（同协议：只跑一遍、300 s 超时、卡死异常判失败；本机 CARLA 离屏、每 25 集例行重启），结果目录 `experiment_results/rerun_b_local/`，启动日志 `experiment_results/rerun_b_local_launch.log`，15:43 起批。服务器上 `~/ljh/code` 已同步至 f6de3e6 但批未启动。
- 本地批第 2 轮发现第三个问题：`udmc_main.py`（两套）中 `vel_ev_dir` 只在检出前车的分支内定义，混合场景无前车而有行人时统计分支抛 `NameError: name 'vel_ev_dir' is not defined`，整集在 8 到 27 s 被截断且被当作有效 run 计入通行时间。旧数据受影响：UDMC 混合 7 集、GRIPC 混合 2、noBSP 2、noIGPR 1、noOBCA 4、noQP 3（40 种子日志口径）。修复：两套入口在 `if lv_state:` 之前每步计算 `vel_ev_dir`。本地批从修复后的下一集起生效；已受影响的本批 run（混合 r=2，8.4 s）在批完成后按结束原因审计、删除并用断点续跑补齐。
- 本地批 attempt1（15:43–17:25，45 集：有效 19、失败 24）中止并归档到 `experiment_results/rerun_b_local_attempt1/`。原因：本机每集 python 进程 OpenBLAS 默认 32 线程，占 11 核以上空转，每仿真秒约 2 s 墙钟（服务器约 1.1 s），异步交通管理器行为随之改变，多车道场景卡死率升到 5/10（服务器批为 1/50），且 300 s 墙钟超时只覆盖约 150 s 仿真。`rerun_b_local.sh` 改为：BLAS/OMP/MKL 线程限 1；超时 420 s；失败集数据改名 `.failed.txt` 保留供审计；起批先探测 attempt1 中卡死的多车道 r=7/9/10 三格，再从 r=1 全量跑（断点续跑复用探测结果）。
