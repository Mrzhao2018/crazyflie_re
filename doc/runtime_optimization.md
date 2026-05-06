# 运行时优化（PR1 – PR9）

> 2026-04-19 记录。本文集中说明已合入 `main` 的主循环、算法、遥测层工程优化，对应 commit 范围 `0959b81 -> 5de6864`。
>
> 目标不是追求新功能，而是压低每帧 CPU、减少阻塞 I/O、让热路径更可控。默认行为保持不变，除非另行标注。
>
> 2026-05-06 补充：当前工作树在 PR1-PR9 后继续推进 full-state/Mellinger 运行链路、leader trajectory 启动 watchdog、startup follower align 和 safety streak 化。这些补充已经改变默认配置，应按当前 `config/*.yaml` 和 `src/` 实现为准。

---

## 优化全景

PR1 – PR4 偏"结构重整"，把 telemetry、主循环、`RealMissionApp` 拆成职责更清晰的模块；PR5 – PR9 偏"热路径向量化与缓存"。两组一起构成这一轮运行时优化。

| PR   | 主题                                               | 主要文件                                                                                   |
| ---- | -------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| PR1  | telemetry JSONL 拆成 `header / event / record`     | `src/runtime/telemetry.py`、`src/runtime/telemetry_replay.py`                              |
| PR2  | 主循环节流 + 范数/位置去重                           | `src/app/run_real.py`                                                                      |
| PR3  | `RealMissionApp` 拆分为若干聚焦模块                | `src/runtime/failure_policy.py` / `landing_flow.py` / `mission_telemetry_reporter.py`      |
| PR4  | 去除未用 transport hook + 文档刷新                 | `src/adapters/cflib_command_transport.py`                                                  |
| PR5  | AffineFrame 的 rank+cond 合并一次 SVD；向量化 controller | `src/runtime/affine_frame_estimator.py` / `follower_controller.py`                     |
| PR6  | 主循环 `fast_gate`、`health_bus` 去重、id-to-index 缓存 | `src/runtime/safety_manager.py` / `scheduler.py` / `pose_bus.py`                       |
| PR7  | PoseBus 预分配缓冲；follower_ref 内部改为 ndarray  | `src/runtime/pose_bus.py` / `src/domain/follower_reference.py`                             |
| PR8  | telemetry 后台 writer + record-shape 快速路径      | `src/runtime/telemetry.py`                                                                 |
| PR9  | scheduler 全集快速路径；AFCModel 暴露 ndarray 变体 | `src/runtime/scheduler.py` / `src/domain/afc_model.py`                                     |

当前分支补充可以理解为 PR1-PR9 之后的“实飞运行路径收口”：

| 主题 | 主要文件 | 目的 |
| --- | --- | --- |
| full-state/Mellinger 默认链路 | `config/fleet.yaml` / `src/runtime/follower_controller_v2.py` / `src/app/run_real.py` | host 只生成平滑 full-state reference，闭环交给 onboard controller |
| follower entry align | `src/app/run_real.py` / `src/adapters/follower_executor.py` | 进入 formation_run 前先把 follower 拉到 entry reference |
| leader trajectory start watchdog | `src/app/run_real.py` / `src/adapters/leader_executor.py` | start 后验证 leader 是否真正移动，失败重试/落地 |
| active drone scope | `src/app/bootstrap.py` / `src/adapters/cflib_link_manager.py` / `src/adapters/lighthouse_pose_source.py` / `src/runtime/pose_bus.py` | leader-only / subset follower 排障不被 inactive drone 阻塞 |
| safety streak 化 | `src/runtime/failure_policy.py` / `src/runtime/safety_manager.py` | watchdog、pose jump、fast-gate 先观测再干预 |
| full-state terminal path | `src/runtime/landing_flow.py` / `src/app/run_real.py` | HOLD / safety land 与 runtime controller 的 setpoint ownership 对齐 |
| single-drone probes | `src/app/*probe.py` / `src/app/cli.py` | 把 firmware/controller/坐标问题从 AFC mission 中拆出来 |

---

## 模块级细节

### 主循环（`src/app/run_real.py`）

- 每帧只做一次 `safety.evaluate`，ABORT 级前置检查改用 `SafetyManager.fast_gate(snapshot)`。`fast_gate` 只扫 `disconnected_ids` 与 `boundary`，不构造 `SafetyReason` 对象。
- `health_bus.latest()` 每帧只调用一次，存进 loop-local 变量后由 `safety.evaluate()` 与 telemetry record 复用。
- HOLD 状态只能被 full-safety `EXECUTE` 清除；`hold_entered_at` 不会被前置检查误 reset，`hold_auto_land_timeout` 能正确触发。
- 仍然保留 `sleep(0.01)` 轮询结构；没有引入 `threading.Event`，避免与 `pose_source` 回调线程产生新的竞态面。

当前分支补充：

- startup readiness 只等待 active drone ids；`active_follower_ids=[]` 时可以 leader-only 运行 startup/trajectory 排障。
- `wait_for_params` 与 `reset_estimator_and_wait` 都能按 `radio_group` 并行，避免 10 机顺序 reset 把启动时间拉长。
- follower entry align 在 takeoff/leader entry 后执行，失败直接归为 readiness failure，而不是让主循环带错位 follower 继续跑。
- auto 模式进入主循环时使用 `elapsed_start_offset` 把 `mission_start_time` 向 trajectory start 对齐，`run_entered` 事件记录该偏移。
- leader trajectory start 不再 inline 调 `leader_executor.execute(...)` 后立刻认定成功，而是进入 start/watchdog 状态机。
- `streaming_setpoint_active` 记录 executor 执行耗时、距离上次 setpoint 的间隔、发送/阻塞/stale group，便于用 telemetry 复盘控制流是否按预期持续输出。

### 定位与 frame 估计（`src/runtime/affine_frame_estimator.py`）

- `rank` 与 `condition_number` 合并为一次 `np.linalg.svd(diff, compute_uv=False)`；旧版等价于两次 SVD（`matrix_rank` + `cond`）。
- 组件 `__init__` 时缓存 `leader_ids_cache` 与 `leader_idx` 数组；热路径直接走数组索引，不再逐 id 调用 `fleet.id_to_index()`。
- 退化场景（rank < 3）返回 `cond = inf` 并标记 `valid=False`，决策路径不变。

### Follower reference（`src/domain/follower_reference.py`）

- 不再在 `compute()` 里复算 `rank` / `cond`——所有权归属 `AffineFrameEstimator`，只保留 NaN 防御。
- `FollowerReferenceSet.frame_condition_number` 现在固定为 `nan`（语义：follower_ref 不是 cond 的权威）。上游若需要 cond，请读 `frame.condition_number`。
- `_last_target_positions` / `_last_target_velocities` 内部状态改为 `(n_f, 3)` ndarray + `tuple(follower_ids)` 签名，不再每帧 dict-copy 全量。

### Follower 控制器（`src/runtime/follower_controller.py`、`follower_controller_base.py`）

- `compute()` 内循环全向量化：一次 `np.stack` 抓出所有 active follower 的 P/T，`V = -K * (P - T) * scales` 一次成型。逐 follower 循环只保留在前馈 clip 与输出范数 clip 上（follower 规模小，通路清晰优先）。
- `_compute_radial_scales` 用 `np.linalg.norm(T[:, :2], axis=1)` + `max` 一次得到所有 radius/ratio；不再逐 follower `np.array(...)`。
- `command_norms` 由向量化 `np.linalg.norm(V, axis=1)` 一次算出，safety 不再重复做范数判定。

`FollowerControllerV2` 当前 full-state 分支的语义已经从“二阶前馈控制器”调整为“平滑 reference 生成器”：

- host 侧不做 PD / 积分闭环，`gain / gain_xy / gain_z / feedforward_gain*` 当前默认全为 `0.0`。
- `target_position` 经过 smoothing alpha 和 max step 限幅。
- `target_velocity` 从相邻两帧平滑后的 target position 差分得到，并受 `max_velocity` 限幅。
- `target_acceleration` 从相邻两帧 derived velocity 差分得到，并受 `max_acceleration` 限幅。
- raw reference velocity / acceleration 不直接下发，只写入 diagnostics 的 ignored 列表，避免与平滑 position reference 不一致。
- full-state smoothing state 只有在 executor 成功发送后才 commit；HOLD / recovery / stale / missing reference 会 reset，避免内部 target 跑到飞机未收到的位置。

这意味着当前 full-state 模式的目标是先把 onboard Mellinger 接入得稳定可复盘，不是追求 host 侧最高性能或最激进前馈。

### PoseBus（`src/runtime/pose_bus.py`）

- `latest()` 复用预分配的 `_scratch_positions` / `_scratch_fresh` 缓冲；每次调用只 `fill(0) + 填入 + .copy()` 给返回值，避免高频 `np.zeros((n,3))` 分配。
- snapshot 之间仍然互相隔离（`.copy()` 保证），`update_agent` / `has_newer_than` 语义不变。
- 当前分支新增 `active_drone_ids`，scratch freshness 只按 active ids 判断；这让 leader-only / subset follower 排障不被 inactive follower 的 stale pose 影响。

### CommandScheduler（`src/runtime/scheduler.py`）

- `_group_drone_ids` 全集快速路径：当 `drone_ids == follower_ids` 或 `leader_ids` 全集时，直接返回预缓存的 `_follower_groups` / `_leader_groups`，不再 `for d in members if d in drone_set`。
- 子集 / 未知 id 仍走慢路径，行为向后兼容。

### Safety（`src/runtime/safety_manager.py`）

- `__init__` 一次性把 `boundary_min/max` 转成 ndarray，边界检查用 `np.any(P < bmin, axis=1)` 向量化。
- `fast_gate` 与 `evaluate` 共享同一套 cached boundary 数组。
- runtime pose jump / speed / vertical speed 异常现在带 per-drone streak；未达到 `runtime_pose_jump_hold_streak` 时作为 `TELEMETRY` 严重度记录，达到阈值后才升级为 HOLD。

### Failure policy（`src/runtime/failure_policy.py`）

- velocity stream watchdog 的 timeout 从固定 factor 改为 `safety.velocity_stream_watchdog_factor`。
- watchdog 干预从“单次 stale 立即 hold/degrade”改为“记录 stale streak，达到 `velocity_stream_watchdog_degrade_streak` 后才执行 action”。
- `hold_entered` 事件保留 safety reason codes 和结构化 details，后续 replay 不需要从文本 message 里猜测 HOLD 原因。
- full-state 模式下 HOLD 通过 `RealMissionApp._execute_hold_actions(...)` 发 full-state hold setpoint，而不是 high-level hold。

### Landing flow（`src/runtime/landing_flow.py`）

- 非紧急 orderly land：先 brake streaming followers，再 `notify_setpoint_stop`，full-state follower 切回 PID 后发 high-level land。
- safety / emergency land：不再试图让 follower high-level land，而是先发 direct descent streaming；velocity 模式发向下速度，full-state 模式递减 z target 并带向下 velocity。
- terminal event 增加 `follower_direct_descent_execution`、`landing_onboard_controller`，便于确认落地路径是否真的走到 follower。

这部分的重点是 setpoint ownership：正在接 full-state stream 的 follower 不应在最危险的 terminal path 上突然混入 high-level hold/land。

### AFC 模型（`src/domain/afc_model.py`）

- 提供 `steady_state_array(leader_positions)`，直接返回 `(n_f, 3)` ndarray + `tuple(follower_ids)`，方便上层继续走向量运算。
- 原 `steady_state(leader_positions) -> dict` 作为薄包装保留，向后兼容。

### Telemetry（`src/runtime/telemetry.py`）

- JSON 序列化 + 写盘移到 daemon writer 线程；主循环只 `queue.put(("record"/"event"/"header", payload))`，热路径零阻塞。
- `record` 走 `_json_safe_record` 快速路径（针对 `TelemetryRecord.asdict()` 的字段结构），避免深度递归 `isinstance` 链。
- `record` 批量 flush；`event` / `header` 立即 flush。`close()` 用 sentinel drain 并 join。
- 队列容量上限 4096：极端情况下 `record` 可能被丢弃并计入 `summary()["records_dropped"]`；`event` 始终阻塞 put（关键事件不丢）。

当前分支新增的关键 event：

- `follower_entry_align`
- `full_state_handoff_setpoint`
- `full_state_hold_setpoint`
- `streaming_setpoint_active` 的 timing/group diagnostics
- `trajectory_start` 的 requested/effective parameters 和 send timings
- `leader_trajectory_motion_confirmed / unconfirmed`
- `leader_trajectory_start_retry / failed`
- `fast_gate_group_degrade_pending`
- `follower_direct_descent_execution`

### FleetModel（`src/domain/fleet_model.py`）

- `all_ids() / leader_ids() / follower_ids() / get_group_members()` 改为返回 `tuple`（只读），不再每次 `.copy()` 一份 list。调用方若需要 mutate，请显式 `list(fleet.xxx_ids())`。

---

## 语义差异（需要留意）

这两处是向后兼容窗口之外的语义差异：

- `follower_ref.frame_condition_number` 现为 `nan`（cond 权威移到 `frame.condition_number`）。
- `LOW_BATTERY`（`min_vbat > 0` 时）触发从 pre-safety 移到 full-safety，晚一帧才进入 emergency_land；当前默认 `min_vbat = 0.0`，所以不会触发。

当前分支额外语义差异：

- 默认控制链路已从 velocity/PID 改为 full-state/Mellinger，`dynamics_model_order=2`，active follower 默认为全部 follower。
- host 侧 full-state raw velocity / acceleration 不直接下发；下发的是由平滑 position target 差分得到的 derived velocity / acceleration。
- `active_follower_ids=[]` 合法，表示 leader-only 排障；旧逻辑会把空列表视为配置错误。
- follower entry align 失败会在 readiness 阶段终止任务。
- HLC trajectory start 需要通过位移验动；失败会重试一次，仍失败则 orderly land。
- watchdog / fast-gate / pose jump 默认都需要 streak，短时异常可能只记录 telemetry 而不立即改变调度。
- safety / emergency land 中 follower 优先 direct descent，不再总是 high-level land。
- `LeaderExecutor` 当前实际下发 `start_trajectory(time_scale=1.0)`，配置请求值仅用于生成 trajectory pieces 和 telemetry 复盘。

---

## 覆盖面

- 36 条既有 + 新增 contract 测试全部通过，其中涉及本轮优化的包括：
  - `test_affine_frame_svd`
  - `test_follower_controller`
  - `test_pose_bus_buffers`
  - `test_safety_fast_gate`
  - `test_scheduler_group_fast_path`
  - `test_afc_steady_state_array`
  - `test_telemetry_async`
  - `test_run_real_watchdog`
- 原 PR1-PR9 阶段 telemetry schema、config 字段、mission 行为均未改变；当前分支已经新增配置字段并改变默认 full-state 运行行为。

当前分支的新增覆盖面集中在：

- CLI parser：新增 single-drone probes。
- ConfigLoader：full-state 默认值、onboard 参数覆盖、watchdog/fast-gate/pose jump/leader trajectory start/follower align 参数校验。
- FollowerControllerV2：full-state state commit/reset、derived feedforward、raw feedforward ignored。
- RealMissionApp：follower entry align、trajectory start watchdog、fast-gate pending streak、full-state hold、direct descent landing、structured HOLD reasons。
- Executors/adapters：leader trajectory send timing、follower `go_to_positions`、active drone ids、reset estimator per group。

注意：上面的“全部通过”是原 PR1-PR9 记录；当前工作树是否全量通过需以本地最新 `pytest` 输出为准。

---

## 价值与边界

这一阶段不主张"立刻改善飞行表现"；真正的收益方向是：

- 更紧的 per-tick CPU 预算，为未来加高频控制回路腾空间。
- telemetry 不再卡主循环，使长任务的 wall-clock 更可预期。
- 把"cond 的权威"等语义收口到单点，避免重复计算漂移。
- 模块拆分与契约测试覆盖，使后续再改运行时更安全。
- 把 full-state setpoint ownership、HLC trajectory start、follower entry align、terminal landing path 变成可观察状态，而不是靠飞行现象猜。
- 支持 leader-only / subset follower 排障，缩小每次真机实验的变量数。

**边界：**

- 尚未在真机上做"基线 vs 优化版" benchmark 对比，当前声称的 CPU / IO 节省是基于代码路径推断，不是测量数据。
- Telemetry 后台 writer 的队列容量上限（4096）已暴露到 `comm.telemetry_queue_max`，但 direct descent 的持续时间/速率/频率仍是代码常量。
- 当前默认 full-state/Mellinger 参数是实飞排障配置，不应直接当成论文/生产基线。
- HLC `time_scale` 临时固定为 `1.0` 是已知回滚点；恢复配置透传前需要单机 `probe-hlc-trajectory` 和多 leader telemetry 验证。
