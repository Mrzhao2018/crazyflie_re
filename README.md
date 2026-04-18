# Crazyflie AFC Swarm

> 2026-04-19 状态更新。
> 
> 这是一个面向 **Crazyflie + Lighthouse + cflib** 的仿射编队控制实验仓库，当前重点是 **真实平台集成、leader 轨迹执行、follower 在线速度闭环、结构化 telemetry 与离线复盘**。

## 当前仓库是什么

这个仓库现在更适合被理解为一个 **真实硬件导向的编队实验基线**，而不是纯仿真项目：

- 保留 `affine formation / stress matrix / AFC` 主线
- 采用 `config -> domain -> runtime -> adapters -> app` 分层组织
- 支持 **自动 leader 执行** 与 **手动 leader 控制** 两种启动模式
- 提供离线参考轨迹可视化、trajectory budget dry-run、telemetry replay 与多次 run 对比工具
- 面向真实平台调试，而不是宣称已经 fully flight-proven 或 production-ready

> 仓库中的部分历史文档、旧实验记录或旧表述仍可能提到 `4 leader + 2 follower` 基线；**当前默认行为请以 `config/*.yaml` 和 `src/` 中的实现为准**。

---

## 当前默认配置摘要

当前 `config/` 下的默认配置已经不是旧版 6 机，而是 **10 机编队**：

- **总数**：10 架 Crazyflie
- **leaders**：`1, 4, 7, 8`
- **followers**：`2, 3, 5, 6, 9, 10`
- **radio groups**：`0, 1, 2`
- **formation_type**：`pyramid`
- **任务时长**：`40s`
- **任务阶段**：
  - `settle`: `0s ~ 4s`
  - `trajectory_entry`: `4s ~ 6s`
  - `formation_run`: `6s ~ 40s`
- **leader motion**：`affine_rotation`
- **trajectory_enabled**：`true`
- **startup.mode**：`auto`
- **pose_log_freq**：`10 Hz`
- **follower_tx_freq**：`8 Hz`
- **leader_update_freq**：`1 Hz`
- **boundary**：`[-1.6, -1.6, -0.1] ~ [1.7, 1.7, 1.8]`
- **min_vbat**：`0.0`，即默认关闭电池阈值拦截

---

## 主要能力

### 1. 分层应用骨架

- `src/domain/`
  - 编队模型、stress matrix、AFC、mission profile、leader/follower reference
- `src/runtime/`
  - pose bus、affine frame estimator、follower controller（v1 向量化、v2 二阶前馈）、scheduler、safety manager（evaluate + fast_gate）、telemetry（异步 writer）、health
  - failure policy、landing flow、mission telemetry reporter、mission FSM、offline swarm sampler、telemetry replay
- `src/adapters/`
  - cflib link/transport、Lighthouse pose source、leader/follower executor、键盘手动输入
- `src/app/`
  - bootstrap、真机主循环、preflight、replay/visualization/comparison CLI

### 2. readiness / preflight

当前启动流程已经包含：

- `connect_all`
- `connect_group_start` / `connect_group_result`（按 `radio_group` 分阶段记录连接过程）
- `wait_for_params`
- 可选 `reset_estimator_and_wait`
- `pose_ready`
- `health_ready`
- `trajectory_prepare`（当 mission 启用 trajectory 时）

`PreflightRunner` 当前会检查：

- 是否至少有 `4` 个 leaders
- 是否至少有 `1` 个 follower
- leader affine span 是否有效
- 启用 trajectory 时是否已 upload / define 且内存可容纳
- pose snapshot 是否存在且 fresh
- 是否存在 disconnected drone
- 起飞前位置是否越界
- health sample 是否存在且新鲜
- 当 `min_vbat > 0` 时是否满足电量阈值

### 3. 两种启动模式

- `auto`
  - 默认模式。leader 按任务参考运行；若启用 trajectory，会在启动阶段完成 upload / define，并在运行阶段启动。
- `manual_leader`
  - 通过键盘实时改变 leader 共时结构参考；follower 仍保留在线闭环。
  - 当前键盘输入适配器基于 Windows `msvcrt`，更适合 Windows 本地终端使用。

### 4. 运行时约束

当前代码里已经明确保护了这些约束：

- 没有新 pose，不重算 follower 控制
- control 计算与无线发包分离
- follower 发包受频率限制与 deadband 限制
- follower / parked hold 的限流已按 `radio_group` 拆分，不再共用单个全局 follower 发送时钟
- safety 动作为 `EXECUTE / HOLD / ABORT`
- 主循环每帧只做**一次** `safety.evaluate`；ABORT 级前置拦截由轻量 `SafetyManager.fast_gate(snapshot)` 承担（只扫 `disconnected_ids` 与 `boundary`）
- HOLD 只能被 full-safety `EXECUTE` 清除，`hold_entered_at` 不会被 fast_gate 提前 reset
- `HOLD` 持续超时后会自动降落（`hold_auto_land_timeout`）
- follower velocity stream 带 watchdog；当速度指令流长时间未刷新时，会按 `velocity_stream_watchdog_action` 执行 `telemetry / hold / degrade`

当前 Phase 1 通信链路已经具备：

- group-aware follower rate limit：不同 `radio_group` 的 follower velocity 发包独立限流
- group-aware parked hold：degrade / parked follower 会按组下发 hold，不与活跃 velocity 路径互相阻塞
- mixed planning：同一轮 scheduler plan 中可以同时出现 active follower velocity 和 parked follower hold
- scheduler diagnostics 已暴露 group 级信息，例如 `parked_group_counts`、`hold_tx_groups_sent`、`follower_tx_groups_sent`

`velocity_stream_watchdog_action` 的行为差异：

- `telemetry`
  - 只记录 `velocity_stream_watchdog` 事件
  - 不改变当前调度与执行路径
- `hold`
  - 记录 watchdog 事件后，立即把超时 follower 切到 HOLD
  - `hold_entered` 会带上稳定 runtime 事件码，便于复盘
- `degrade`
  - 记录 watchdog 事件后，将超时 follower 放入降级集合
  - 后续调度中这些 follower 不再走 velocity，而是通过 `parked_follower_ids` 下发 hold
  - telemetry 会继续记录 `watchdog_degrade` / `watchdog_degrade_recovered`

### 5. Telemetry 与离线分析

真机运行会输出：

- `telemetry/run_real_YYYYMMDD_HHMMSS.jsonl`

当前 telemetry 使用 `schema_version=2` 的 JSONL 流结构，每一行都是一个带 `kind` 标记的 JSON 对象：

- 第一行 `kind=header`：`schema_version`、`config_fingerprint`、`readiness`、`fleet` 摘要（drone_count、leader/follower ids、radio groups）。
- 后续混合两种 `kind`：
  - `kind=event`：结构化事件（连接阶段、watchdog、mission_error、executor summary 等）。
  - `kind=record`：每次新 pose 到达时的运行期快照（snapshot_seq、measured_positions、safety_action、scheduler_diagnostics、follower_command_norms 等）。
- `record` 不再每帧拷贝 `phase_events` / `config_fingerprint` / `readiness`——这些字段只在 header 写一次，事件走独立流，整体 JSONL 体积大幅下降。
- JSON 序列化 + 写盘发生在一条 daemon writer 线程上；主循环只做 `queue.put`，不再阻塞在 `json.dumps` / `fh.write`。`event` 与 `header` 立即 flush；`record` 批量 flush。`close()` 时 drain 队列并 join 线程。
- 队列容量上限 4096：极端情况下 `record` 可能被丢弃并计入 `summary()["records_dropped"]`；`event` 始终阻塞 put（关键事件不丢）。

当前 telemetry 已记录的内容包括：

- header 里的 readiness / fleet 摘要 / config fingerprint
- connect 阶段的 `connect_group_start`、`connect_group_result`、带 `radio_groups` 摘要的 `connect_all`
  - 这些事件现在都带稳定的 `category / code / stage / outcome`
  - `connect_group_result` 会区分 `success / partial_failure / failed`
- `mission_error` 结构化错误事件，包含稳定的 `category / code / stage`
- watchdog 结构化事件，包含稳定的 `category / code / stage`
- watchdog / executor 事件里的 `radio_groups` 聚合摘要
  - executor failure 现在会带 `command_kind / error_type / failure_category / retryable / radio_group`
- record 流中的 snapshot 序号与测量时间
- measured positions / leader reference / follower reference
- frame validity / condition number
- safety action / reason codes
- scheduler diagnostics
- leader / follower action count
- follower command norms
- startup mode、manual axis、trajectory state 等上下文

`src/runtime/telemetry_replay.py` 提供 `iter_telemetry(path)` 一次性返回
`(header, events, records)`；旧版（记录内嵌 `phase_events` 列表、无 `kind`
字段）也会自动走兼容分支，因此现有 `analyze_records` / `build_replay` /
`load_records` API 对新老两种文件都能正常解析。

当前 `mission_error` 的错误定义已经按三类集中管理：

- `connection`：链路建立和设备连接阶段失败
- `readiness`：启动、readiness、preflight 与起飞验证阶段失败
- `runtime`：进入主循环之后的运行期异常

其中 connection 相关的稳定事件码现在包括：

- `CONNECT_GROUP_START`
- `CONNECT_GROUP_SUCCESS`
- `CONNECT_GROUP_PARTIAL_FAILURE`
- `CONNECT_GROUP_FAILED`
- `CONNECT_ALL_OK`
- `CONNECT_ALL_FAILED`

说明：

- `CONNECT_GROUP_PARTIAL_FAILURE` 表示某个 `radio_group` 已有部分成员完成连接，但组内后续成员失败，strict 策略会因此中止启动
- `CONNECT_GROUP_FAILED` 表示该组在进入连接阶段后没有任何成员成功建立链路
- `connect_all` 事件会额外带出 `failed_group_ids` 与整体 `outcome`，便于区分“整组失败”还是“已有部分连接后中止”

其中 watchdog 相关的稳定 runtime 事件码包括：

- `RUNTIME_VELOCITY_STREAM_WATCHDOG`
- `RUNTIME_WATCHDOG_HOLD`
- `RUNTIME_WATCHDOG_DEGRADE`
- `RUNTIME_WATCHDOG_DEGRADE_RECOVERED`

说明：

- 这些事件当前作为 telemetry phase events 记录，用于稳定复盘与后续自动分析
- 它们与 `mission_error` 共享同一套 `category / code / stage` 语义，但不一定意味着任务立即失败
- `hold` / `degrade` 属于运行期保护动作；只有后续演化为 `ABORT` 或其它终止路径时，才进入终止语义

PR10/PR11 通信链路侧新增的稳定 runtime 事件码：

- `RUNTIME_LINK_RECONNECT_ATTEMPT`
- `RUNTIME_LINK_RECONNECT_OK`
- `RUNTIME_LINK_RECONNECT_FAILED`

以及新增的 telemetry 事件（非 mission_error，不是终止语义）：

- `fast_gate_group_degrade`：单组掉线被降级为 parked hold 而非整队 ABORT 时记录
- `link_reconnect_attempt / link_reconnect_ok / link_reconnect_failed`：`comm.reconnect_enabled=true` 时才会出现

record 流中新增了 `radio_link_quality` 字段：按 `drone_id` 保存 `link_quality / uplink_rssi / uplink_rate / downlink_rate / uplink_congestion / downlink_congestion / last_update_t`，数据来自 cflib `radio_link_statistics` 回调。`link_quality_enabled=false` 或未装配 `LinkQualityBus` 时该字段为空 dict。

当前 Phase 3 前半段已经把 executor 失败结果从纯文本扩展为结构化字段：

- `command_kind`：失败发生在哪类命令上，例如 `batch_goto / velocity / hold / notify_stop`
- `error_type`：Python 异常类型名，例如 `TimeoutError / KeyError / RuntimeError`
- `failure_category`：归一化失败类别，例如 `timeout / link_lookup / invalid_command / transport_runtime`
- `retryable`：粗粒度可重试标记；当前只做语义暴露，还没有把它接入运行期策略
- `radio_group`：失败对应的无线组，便于后续做 group-aware 连续失败策略

当前 Phase 3 后半段已经把一部分 follower executor 失败接入运行期策略：

- 只看 `follower_velocity_execution` 的 failure，不改变 leader 路径
- 同一 `radio_group` 的 retryable failure 连续达到 `2` 次时，触发 `executor_group_degrade`
- 同一 `radio_group` 一旦出现 non-retryable failure，会立即进入组级策略触发
- 如果当前活跃 follower group 全部都满足触发条件，则直接记录 `executor_group_hold` 并进入 `HOLD`
- 如果只有部分 group 触发，则把对应 follower 放入降级集合，后续调度改走 parked hold

对应的稳定 runtime 事件码：

- `RUNTIME_EXECUTOR_GROUP_DEGRADE`
- `RUNTIME_EXECUTOR_GROUP_HOLD`

说明：

- 这一步仍然是保守实现，只接 follower velocity 执行失败，还没有把 leader failure、trajectory upload failure 或更复杂的 retry budget 纳入统一策略
- 当前连续失败阈值是代码内常量 `2`，还没有暴露到配置文件

仓库内已提供的离线工具包括：

- replay summary
- offline reference visualization
- thesis-style trajectory comparison
- multi-run comparison

当前 replay / comparison / compare-runs 的离线摘要还可以直接看出：

- watchdog 总触发次数
- watchdog 是 `telemetry / hold / degrade / degrade_recovered` 哪一种
- 多次 run 之间各类 watchdog 触发次数的差异
- executor group failure 总触发次数
- executor group failure 是 `degrade` 还是 `hold`
- 哪些 `radio_group` 更容易积累失败触发
- retryable / non-retryable failure 的离线计数差异

当前离线摘要新增字段：

- `executor_failure_summary`
  - `total`
  - `by_code`
  - `by_action`
  - `by_event`
  - `by_group`
  - `failure_categories`
  - `retryable_counts`

`compare-runs` 当前会额外抽出这些指标：

- `executor_failure_total`
- `executor_failure_degrade_count`
- `executor_failure_hold_count`
- `executor_failure_group_count`
- `executor_failure_retryable_count`
- `executor_failure_non_retryable_count`
- `radio_link_sample_count`（PR10）
- `radio_link_overall_min / mean / p5 / max`（PR10）
- `radio_link_worst_drone_min`（PR10）

PR10 之后，`replay` / `compare-runs` 的离线摘要新增 `radio_link_summary`：

- `overall`: `count / min / mean / p5 / max`
- `per_drone`: `{drone_id: {count, min, mean, p5, max}}`

缺失 `radio_link_quality` 字段（旧 run 或 `link_quality_enabled=false`）时 `overall.count == 0`，旧文件兼容解析。

`compare-runs --output ...` 现在会额外输出：

- `compare_communication.png`
  - 同时展示 watchdog 总数、executor failure 总数、executor degrade 次数、executor hold 次数

### 6. 可选组间并行 / 自适应 / 重连开关

仓库现在汇集了一批**默认关闭**的实验性能力，方便在真机上逐项 ablation，**默认行为保持不变**：

PR1 既有：

- `comm.connect_groups_in_parallel`：`connect_all()` 是否按 `radio_group` 并行连接（默认 `false`）
- `comm.trajectory_upload_groups_in_parallel`：startup 阶段 leader trajectory upload / define 是否按 `radio_group` 并行（默认 `false`）

PR10 新增（主要是可观测 + 已默认开启的代码路径加速）：

- `comm.connect_pace_s`（默认 `0.2`）：多机连接之间的 sleep；实飞稳定后可下调到 `0.05`
- `comm.connect_timeout_s`（默认 `5.0`）：单机 `open_link` 超时；radio 拥塞可抬到 `10.0`
- `comm.link_quality_enabled`（默认 `true`）：开启 cflib `radio_link_statistics` 回调接入 `LinkQualityBus`，写入 `record.radio_link_quality` 与离线 `radio_link_summary`
- `CflibLinkManager` 默认启用 `ro_cache=./cache/ro / rw_cache=./cache/rw`：TOC 可跨机复用，`wait_for_params` 在 `GroupExecutorPool` 上按 `radio_group` 并行
- `FollowerExecutor / LeaderExecutor` 默认通过 `GroupExecutorPool` 跨 `radio_group` 并行发包（组内保序）；`connect_all` 报告中新增 `parallel` 与 `per_group_duration_s`

PR11 新增（**默认全关**，真机 ablation 前不会改变 baseline 行为）：

- `comm.radio_driver`（默认 `auto`）：`auto / python / cpp`。选 `cpp` 需要 `pip install cflinkcpp`；选 `python` 会强制清除 `USE_CFLINK` 环境变量。
- `comm.link_quality_soft_floor`（默认 `0.0`）：`> 0` 时对 link_quality 低于该阈值的 group 启用 tx 降频 + deadband 放大
- `comm.link_quality_backoff_scale`（默认 `1.5`）：降频倍数。`follower_tx_interval × scale`
- `comm.link_quality_deadband_scale`（默认 `2.0`）：deadband 放大倍数。`follower_cmd_deadband × scale`
- `safety.fast_gate_group_degrade_enabled`（默认 `false`）：`true` 时只影响部分 `radio_group` 的 disconnect 不再 ABORT，而是把这些 group 的 follower 推入 `watchdog_degraded_followers`（走 parked hold 路径）
- `comm.reconnect_enabled`（默认 `false`）：`true` 时纯 disconnect 触发 fast_gate ABORT 前先做一次有限次数自动重连
- `comm.reconnect_attempts`（默认 `2`）
- `comm.reconnect_backoff_s`（默认 `0.5`）
- `comm.reconnect_timeout_s`（默认 `5.0`）

所有这些开关的配置校验由 `ConfigLoader` 负责；具体语义与失败事件已暴露到 telemetry，便于离线复盘对比。

### 7. 运行时性能与工程化优化

这一节集中记录**已经合入 main** 的主循环/算法/遥测层工程优化。目标不是追求新功能，而是压低每帧 CPU、减少阻塞 I/O、让热路径更可控。默认行为保持不变，除非另行标注。

**主循环（`src/app/run_real.py`）**

- 每帧只做一次 `safety.evaluate`，ABORT 级前置检查改用 `SafetyManager.fast_gate(snapshot)`。`fast_gate` 只扫 `disconnected_ids` 与 `boundary`，不构造 `SafetyReason` 对象。
- `health_bus.latest()` 每帧只调用一次，存进 loop-local 变量后由 `safety.evaluate()` 与 telemetry record 复用。
- HOLD 状态只能被 full-safety `EXECUTE` 清除；`hold_entered_at` 不会被前置检查误 reset，`hold_auto_land_timeout` 能正确触发。
- 仍然保留 `sleep(0.01)` 轮询结构；没有引入 `threading.Event`，避免与 `pose_source` 回调线程产生新的竞态面。

**定位与 frame 估计（`src/runtime/affine_frame_estimator.py`）**

- `rank` 与 `condition_number` 合并为一次 `np.linalg.svd(diff, compute_uv=False)`；旧版等价于两次 SVD（`matrix_rank` + `cond`）。
- 组件 `__init__` 时缓存 `leader_ids_cache` 与 `leader_idx` 数组；热路径直接走数组索引，不再逐 id 调用 `fleet.id_to_index()`。
- 退化场景（rank < 3）返回 `cond = inf` 并标记 `valid=False`，决策路径不变。

**Follower reference（`src/domain/follower_reference.py`）**

- 不再在 `compute()` 里复算 `rank` / `cond`——所有权归属 `AffineFrameEstimator`，只保留 NaN 防御。
- `FollowerReferenceSet.frame_condition_number` 现在固定为 `nan`（语义：follower_ref 不是 cond 的权威）。上游若需要 cond，请读 `frame.condition_number`。
- `_last_target_positions` / `_last_target_velocities` 内部状态改为 `(n_f, 3)` ndarray + `tuple(follower_ids)` 签名，不再每帧 dict-copy 全量。

**Follower 控制器（`src/runtime/follower_controller.py`、`follower_controller_base.py`）**

- `compute()` 内循环全向量化：一次 `np.stack` 抓出所有 active follower 的 P/T，`V = -K * (P - T) * scales` 一次成型。逐 follower 循环只保留在前馈 clip 与输出范数 clip 上（follower 规模小，通路清晰优先）。
- `_compute_radial_scales` 用 `np.linalg.norm(T[:, :2], axis=1)` + `max` 一次得到所有 radius/ratio；不再逐 follower `np.array(...)`。
- `command_norms` 由向量化 `np.linalg.norm(V, axis=1)` 一次算出，safety 不再重复做范数判定。

**PoseBus（`src/runtime/pose_bus.py`）**

- `latest()` 复用预分配的 `_scratch_positions` / `_scratch_fresh` 缓冲；每次调用只 `fill(0) + 填入 + .copy()` 给返回值，避免高频 `np.zeros((n,3))` 分配。
- snapshot 之间仍然互相隔离（`.copy()` 保证），`update_agent` / `has_newer_than` 语义不变。

**CommandScheduler（`src/runtime/scheduler.py`）**

- `_group_drone_ids` 全集快速路径：当 `drone_ids == follower_ids` 或 `leader_ids` 全集时，直接返回预缓存的 `_follower_groups` / `_leader_groups`，不再 `for d in members if d in drone_set`。
- 子集 / 未知 id 仍走慢路径，行为向后兼容。

**Safety（`src/runtime/safety_manager.py`）**

- `__init__` 一次性把 `boundary_min/max` 转成 ndarray，边界检查用 `np.any(P < bmin, axis=1)` 向量化。
- `fast_gate` 与 `evaluate` 共享同一套 cached boundary 数组。

**AFC 模型（`src/domain/afc_model.py`）**

- 提供 `steady_state_array(leader_positions)`，直接返回 `(n_f, 3)` ndarray + `tuple(follower_ids)`，方便上层继续走向量运算。
- 原 `steady_state(leader_positions) -> dict` 作为薄包装保留，向后兼容。

**Telemetry（`src/runtime/telemetry.py`）**

- JSON 序列化 + 写盘移到 daemon writer 线程；主循环只 `queue.put(("record"/"event"/"header", payload))`，热路径零阻塞。
- `record` 走 `_json_safe_record` 快速路径（针对 `TelemetryRecord.asdict()` 的字段结构），避免深度递归 `isinstance` 链。
- `record` 批量 flush；`event` / `header` 立即 flush。`close()` 用 sentinel drain 并 join。

**FleetModel（`src/domain/fleet_model.py`）**

- `all_ids() / leader_ids() / follower_ids() / get_group_members()` 改为返回 `tuple`（只读），不再每次 `.copy()` 一份 list。调用方若需要 mutate，请显式 `list(fleet.xxx_ids())`。

**覆盖面**

- 36 条既有 + 新增 contract 测试全部通过（包含 `test_affine_frame_svd` / `test_follower_controller` / `test_pose_bus_buffers` / `test_safety_fast_gate` / `test_scheduler_group_fast_path` / `test_afc_steady_state_array` / `test_telemetry_async` / `test_run_real_watchdog` 等）。
- telemetry schema、config 字段、mission 行为均未改变。
- 两处语义差异请留意：
  - `follower_ref.frame_condition_number` 现为 `nan`（cond 权威移到 `frame`）
  - `LOW_BATTERY`（`min_vbat > 0` 时）触发从 pre-safety 移到 full-safety，晚一帧才进入 emergency_land；默认 `min_vbat = 0.0` 无影响

---

### 8. 通信稳定性优化（PR10 / PR11）

PR1–PR9 收口了主循环、算法、遥测的热路径；PR10–PR11 这一轮把重点放在**底层链路与上层发包调度的耦合**——这是 PR1–PR9 触不到、但真机上真正导致"说不清的通信不稳"的地方。

**PR10 — 可观测性 + 组间并行（默认启用、行为兼容）**

- `LinkQualityBus`（`src/runtime/link_quality_bus.py`）：汇聚每架 drone 的 `link_quality / uplink_rssi / uplink_rate / downlink_rate / uplink_congestion / downlink_congestion`，来源是 cflib `cf.link_statistics` 的 6 个 `Caller`。
- `CflibLinkManager` 在 `open_link` 前挂回调，并调 `cf.link_statistics.start()`。`bus=None` 时不启动采集，避免无消费方白白消耗。
- `TelemetryRecord.radio_link_quality: dict[drone_id, metrics]` 字段；`replay` / `compare-runs` 汇总 `radio_link_summary.overall / per_drone` + `radio_link_worst_drone_min`。
- `GroupExecutorPool`（`src/adapters/group_executor_pool.py`）：每 `radio_group` 一条工作线程；`FollowerExecutor.execute_velocity / execute_hold`、`LeaderExecutor._execute_batch_goto`、`wait_for_params_per_group(...)` 都走这个池子。组内保序（同 dongle 本来就要串行）、组间真并行。
- `CflibLinkManager` 启用 `ro_cache=./cache/ro`，跨机复用 TOC；`connect_pace_s` / `connect_timeout_s` 可配置。
- `connect_all` 报告扩展：`parallel` 标记、`per_group_duration_s`，便于离线判断 connect 阶段瓶颈。

**PR11 — 链路层干预（默认关闭、逐项 opt-in）**

- `radio_driver: auto / python / cpp`（`src/adapters/radio_driver_select.py`）：cflib 通过 `USE_CFLINK` 环境变量与 `cflib.crtp.CLASSES` 决定驱动选择；`cpp` 需要 `cflinkcpp` 包，缺失时抛 `RuntimeError`。
- `CommandScheduler` 接收 `link_quality_provider(group_id) -> float|None`；`link_quality_soft_floor > 0` 时对低质量组把 `follower_tx_interval × link_quality_backoff_scale`，同时对该组 follower 的 `deadband × link_quality_deadband_scale`。`diagnostics.link_quality_backoff_groups` 暴露受抑制的组。
- `SafetyManager.fast_gate_decision(snapshot)` 返回 `FastGateDecision(action, reason_codes, degrade_groups)`：全组掉线或越界 -> `ABORT`，**部分组**掉线 -> `HOLD_GROUP` + `degrade_groups`；`safety.fast_gate_group_degrade_enabled=true` 时 `run_real` 走新路径，`FailurePolicy.apply_fast_gate_group_degrade` 把这些组的 follower 推进 `watchdog_degraded_followers`。旧 `fast_gate()` 签名保持不变。
- `CflibLinkManager.reconnect(drone_id, *, attempts, backoff_s, timeout_s)` 做有限次重连：close 旧 scf -> open 新 scf。`FailurePolicy.attempt_reconnect(drone_ids)` 消费 `comm.reconnect_*` 并记录 `RUNTIME_LINK_RECONNECT_{ATTEMPT,OK,FAILED}` 事件；`run_real` 在 fast_gate ABORT + 纯 disconnect 时会先尝试 reconnect，全部成功则跳过 emergency_land。

**契约覆盖面**

- 41 条契约测试（24 既有 + 17 新）全部通过：`test_link_quality_bus / test_link_quality_wire / test_telemetry_radio_link / test_replay_radio_link_summary / test_compare_runs_radio_link / test_comm_config_link_quality / test_group_executor_pool / test_follower_executor_group_parallel / test_leader_executor_group_parallel / test_wait_for_params_parallel / test_config_loader_connect_fields / test_link_manager_connect_report / test_radio_driver_select / test_scheduler_link_quality / test_safety_fast_gate_group / test_link_manager_reconnect` + 全部既有 run_real / scheduler / safety / telemetry 回归。
- telemetry schema、config 字段、mission 行为在默认配置下均未改变。
- 接入 cflib 0.1.30 的所有路径都走公开 API；`cflinkcpp` 为可选依赖。

---

## 仓库结构

```text
config/
  fleet.yaml
  mission.yaml
  comm.yaml
  safety.yaml
  startup.yaml
src/
  adapters/
  app/
  config/
  domain/
  experimental/
  runtime/
  tests/
telemetry/
artifacts/
doc/
docs/
archive/
main.py
scripts/
```

补充说明：

- `archive/`：历史脚本、旧实验与暂存内容
- `doc/`：项目设计与阶段性说明文档
- `docs/`：Sphinx 文档目录
- `artifacts/`：离线分析、对比图和运行产物
- `telemetry/`：真机运行 JSONL 记录

---

## 常用命令

### 1. 统一 CLI 入口（推荐）

现在推荐统一通过 `src.app.cli` 调用各类入口：

```bash
python -m src.app.cli --help
python -m src.app.cli run --help
python -m src.app.cli compare-runs --help
```

统一 CLI 当前支持的子命令包括：

- `run`：真机任务入口
- `budget`：trajectory 内存预算 dry-run
- `replay`：telemetry replay summary
- `viz`：离线参考轨迹可视化
- `compare`：单次 run 理想轨迹对比
- `compare-runs`：多次 run 汇总与回归检查
- `sim`：最小离线 smoke test

`scripts/` 下还提供了一组第一阶段调参辅助脚本：

- `scripts/generate_baseline_sweep.py`
  - 通过生成临时 `config/` 副本并循环调用 `python -m src.app.run_sim`
  - 输出 `baseline_results.json`
  - 用于建立当前一阶模型下的 Baseline RMSE 候选结果
  - 默认先用 `p-limit` 网格，只扫 `gain_xy / gain_z / max_velocity`
  - 支持 `--grid quick` 和 `--limit-trials N` 做快速试跑

当前 Phase 1A 推荐先这样固定第一版 Baseline：

```bash
python scripts/generate_baseline_sweep.py --grid p-limit --dt 0.25 --formation-rmse-threshold 0.05 --output artifacts/baseline_results.json
```

Phase 1B 目前已接入一版默认关闭的纯时滞补偿配置：

- `time_delay_compensation_enabled`
- `estimated_total_delay_ms`
- `delay_prediction_gain`

当前实现位置在 `FollowerReferenceGenerator`，逻辑是基于历史 target velocity 做一阶线性预测，不改变现有一阶 follower controller 结构。

`scripts/` 下还提供了时滞补偿离线对比脚本：

- `scripts/generate_delay_compensation_ablation.py`
  - 对比 `delay_off` 与 `delay_on` 两组配置
  - 输出 `delay_compensation_ablation.json`
  - 用于量化 `formation_rmse` / `follower_rmse` / `frame_valid_rate` 的差异

Phase 1C 当前已接入一版 Leader trajectory 条件数质量摘要与离线对比工具：

- `condition_penalty_enabled`
- `condition_soft_limit`
- `condition_penalty_scale`

当前这条线优先做的是离线 quality summary 和 on/off ablation，而不是直接改运行时安全策略。

- `scripts/generate_trajectory_condition_ablation.py`
  - 对比 `cond_penalty_off` 与 `cond_penalty_on` 两组配置
  - 输出 `trajectory_condition_ablation.json`
  - 用于量化 `formation_rmse` / `follower_rmse` / `frame_valid_rate` 与 `penalized_samples` 的差异

`scripts/` 下还提供了第二阶段模型阶次调参与公平对比脚本：

- `scripts/generate_second_order_baseline_sweep.py`
  - 通过生成临时 `config/` 副本并循环调用 `python -m src.app.run_sim`
  - 输出 `second_order_baseline_results.json`
  - 默认扫描 `gain_xy / gain_z / max_velocity / velocity_feedback_gain / acceleration_feedforward_gain / damping_coeff`
  - 支持 `--grid quick`、`--limit-trials N`
  - 长时运行时会打印 trial 进度条、当前 rmse、best rmse、elapsed 和 eta
- `scripts/generate_model_order_ablation.py`
  - 可同时读取 `baseline_results.json` 与 `second_order_baseline_results.json`
  - 输出 `model_order_ablation.json`
  - 用于做一阶与二阶各自使用最优基线参数的 tuned-vs-tuned 公平对比
  - 结果中会额外写出 `applied_control`，明确每个 trial 实际应用的控制参数

### Phase 1 Findings

第一阶段三条线的当前离线结论已经固定到 `artifacts/`：

- `artifacts/baseline_results.json`
- `artifacts/delay_compensation_ablation.json`
- `artifacts/trajectory_condition_ablation.json`

1. Phase 1A Baseline

- 当前一阶闭环离线基线的最佳参数为：
  - `gain_xy = 1.4`
  - `gain_z = 0.6`
  - `max_velocity = 0.55`
- 对应基线指标：
  - `formation_rmse ≈ 0.00463`
  - `follower_rmse ≈ 0.00801`
  - `formation_p95 ≈ 0.01000`
  - `frame_valid_rate = 1.0`

2. Phase 1B Delay Compensation

- 在上述 Baseline 参数集上，开启 reference 层的一阶纯时滞补偿后：
  - `formation_rmse` 从 `≈ 0.00463` 降到 `≈ 0.00422`
  - `follower_rmse` 从 `≈ 0.00801` 降到 `≈ 0.00734`
  - `frame_valid_rate` 无退化
- 结论：当前 delay compensation 在第一版 Baseline 上带来稳定正收益，可视为后续实机验证候选能力。

3. Phase 1C Leader Trajectory Condition Penalty

- 在默认 `affine_rotation` 任务上，leader 几何条件数本来就稳定在约 `3.04`，`cond_penalty_on` 不会触发，也不会改变 RMSE。
- 为了验证第三条线是否真实生效，额外引入了 `condition_stress_*` 压测轨迹参数：
  - `condition_stress_enabled`
  - `condition_stress_axis`
  - `condition_stress_min_scale`
  - `condition_stress_period`
- 在 stress trajectory 下：
  - `raw_condition_number_max` 从默认良态抬高到 `≈ 4.09`
  - `cond_penalty_on` 可将 `condition_number_max` 压回到 `≈ 2.34`
  - `penalized_samples = 9`
  - 当前离线 RMSE 仍未改善
- 结论：第三条线已经能在坏轨迹上真实降低几何条件数，但当前离线一阶模型下，这种几何修正尚未传导为 RMSE 收益。它更像几何鲁棒性保护，而不是当前基线下的主要性能增益来源。

当前推荐的工程结论是：

- Baseline 参数集可直接作为第一版一阶基线
- Delay compensation 值得继续做实机验证
- Condition penalty 应保留为轨迹质量保护机制，但不建议仅凭当前默认任务将其作为主收益项宣传

### Phase 2 Findings

第二阶段模型阶次与二阶 follower 内部动力学的当前离线结论已经固定到 `artifacts/`：

- `artifacts/second_order_baseline_results.json`
- `artifacts/model_order_ablation_tuned.json`

1. Phase 2A Second-order Baseline

- 当前二阶离线基线的最佳参数为：
  - `acceleration_feedforward_gain = 0.5`
  - `damping_coeff = 0.05`
  - `gain_xy = 1.4`
  - `gain_z = 0.5`
  - `max_velocity = 0.65`
  - `velocity_feedback_gain = 1.2`
- 对应基线指标：
  - `formation_rmse ≈ 0.00347`
  - `follower_rmse ≈ 0.00603`
  - `formation_p95 ≈ 0.00848`
  - `frame_valid_rate = 1.0`

2. Phase 2B Tuned-vs-Tuned Model-order Ablation

- 使用一阶最优基线与二阶最优基线分别配置后：
  - 一阶：`formation_rmse ≈ 0.00470`，`follower_rmse ≈ 0.00808`
  - 二阶：`formation_rmse ≈ 0.00347`，`follower_rmse ≈ 0.00603`
- 二阶相对一阶的改进为：
  - `formation_rmse delta ≈ -0.00123`
  - `follower_rmse delta ≈ -0.00205`
  - `frame_valid_rate delta = 0.0`
- 结论：在当前 `dt = 5.0`、`total_time = 10.0` 的离线窗口内，二阶 internal-dynamics follower path 已经带来稳定的跟踪收益，且没有牺牲有效帧率。

3. Phase 2C Engineering Note

- `scripts/generate_model_order_ablation.py` 当前会优先保留 `second_order_baseline_results.json` 中的 `velocity_feedback_gain` 与 `acceleration_feedforward_gain`，再补默认项。
- 这样做是为了确保 tuned-vs-tuned 公平对比真正使用二阶最优参数，而不是被脚本默认值回写覆盖。
- 当前证据仍来自短窗口 smoke-style 配置；是否在更长时域与更激进 leader 轨迹下继续保持优势，仍建议后续再做 `30s ~ 40s` 验证。

> `main.py` 仍保留旧入口兼容行为；如果你已经习惯 `python main.py`、`python main.py --startup-mode ...`、`python main.py --trajectory-budget`，这些命令仍可继续使用。

### 2. 真机主入口

```bash
python -m src.app.cli run
```

兼容旧写法：

```bash
python main.py
```

行为说明：

- 读取 `config/`
- 组装 `build_app("config")`
- 等待用户按 Enter 后启动
- 需要真实 Crazyflie、radio 和 Lighthouse 环境就绪

### 3. 覆盖启动模式

```bash
python -m src.app.cli run --startup-mode auto
python -m src.app.cli run --startup-mode manual_leader
```

兼容旧写法：

```bash
python main.py --startup-mode auto
python main.py --startup-mode manual_leader
```

### 4. 只做 trajectory 预算 dry-run

```bash
python -m src.app.cli budget --config-dir config
```

兼容旧写法：

```bash
python main.py --trajectory-budget
```

这个命令：

- **不会连接真机**
- 会输出每个 leader 的 piece 数、预计字节数、起始地址与是否 fit memory
- 当前摘要里也会带出 `config_dir` 与 `startup_mode`

### 5. telemetry replay summary

```bash
python -m src.app.cli replay telemetry/run_real_20260402_020853.jsonl
```

兼容底层入口：

```bash
python -m src.app.replay_analysis telemetry/run_real_20260402_020853.jsonl
```

注意：

- `src.app.replay_analysis` 不传路径时默认读取 `telemetry/run_real.jsonl`
- 但真机主入口默认写入的是 **带时间戳** 文件
- 因此平时更推荐 **显式传入 telemetry 文件路径**
- replay / comparison 产物中现在也会保留 `config_fingerprint`，便于确认不同 run 是否来自同一套配置

### 6. 离线参考轨迹可视化

```bash
python -m src.app.cli viz --config-dir config --output-dir artifacts/offline_reference_viz
```

兼容底层入口：

```bash
python -m src.app.offline_reference_viz --config-dir config --output-dir artifacts/offline_reference_viz
```

默认输出：

- `artifacts/offline_reference_viz/swarm_reference.png`
- `artifacts/offline_reference_viz/swarm_reference.gif`

常用参数：

- `--dt`：离线采样步长
- `--fps`：GIF 帧率
- `--trail`：动画轨迹尾迹长度
- `--total-time`：只渲染任务前一部分时长

### 7. 单次 run 理想轨迹对比

```bash
python -m src.app.cli compare telemetry/run_real_20260402_020853.jsonl
```

兼容底层入口：

```bash
python -m src.app.trajectory_comparison telemetry/run_real_20260402_020853.jsonl
```

特点：

- 默认输出到 `artifacts/<telemetry_stem>/`
- 生成 `trajectory_comparison_summary.json`
- 同时生成 overlay 图和 tracking error 图
- 若不传 telemetry 路径，会自动选择 `telemetry/` 下最新的 `run_real_*.jsonl`

### 8. 多次 run 摘要对比

```bash
python -m src.app.cli compare-runs artifacts/run_real_20260401_235152 artifacts/run_real_20260402_020853 --output artifacts/compare_runs.json
```

兼容底层入口：

```bash
python -m src.app.trajectory_compare_runs artifacts/run_real_20260401_235152 artifacts/run_real_20260402_020853 --output artifacts/compare_runs.json
```

这个命令会：

- 汇总多次 run 的 formation / leader / follower 指标
- 生成 `compare_runs.json`
- 额外生成 `compare_overview.png` 与 `compare_roles.png`
- 在 summary 中保留各次 run 的 `config_sha256`

### 9. 回归阈值检查

如果你已经有一组可接受的实飞基线，可以直接在多 run 对比时加阈值：

```bash
python -m src.app.cli compare-runs \
  artifacts/run_real_20260401_235152 \
  artifacts/run_real_20260402_020853 \
  --output artifacts/compare_runs_regression.json \
  --formation-rmse-threshold 0.11 \
  --frame-valid-threshold 0.82
```

当前支持的阈值包括：

- `--formation-rmse-threshold`
- `--frame-valid-threshold`
- `--leader-rmse-threshold`
- `--follower-rmse-threshold`

输出结果中会新增：

- `regression_checked`
- `regression_thresholds`
- `failing_runs`
- 每个 run 各指标的 `passed / failed` 检查结果

这适合用来做：

- 参数回归检查
- 多轮实飞筛选
- 论文图表产出前的批量质量门禁

### 10. 离线 smoke test

```bash
python -m src.app.cli sim --config-dir config --dt 0.25 --total-time 10
```

兼容底层入口：

```bash
python -m src.app.run_sim --config-dir config --dt 0.25 --total-time 10
```

这个入口当前是一个 **最小离线 smoke runner**，用于快速检查：

- 配置是否能正常装载
- leader / follower 参考链路是否能离线采样
- affine frame 与 follower validity 是否持续有效
- 当前 mission phase 与 leader mode 是否符合预期

如果要把结果落盘，可以直接调用底层入口：

```bash
python -m src.app.run_sim --config-dir config --dt 0.25 --total-time 10 --output artifacts/offline_smoke_summary.json
```

`sim` 现在更适合当作 **离线控制链冒烟测试**，而不是完整仿真器。

### 11. 部分测试示例

```bash
python -m src.tests.test_run_real
python -m src.tests.test_preflight
python -m src.tests.test_mission_profile
python -m src.tests.test_trajectory_budget_summary
python -m src.tests.test_offline_reference_viz
python -m src.tests.test_trajectory_comparison
python -m src.tests.test_trajectory_compare_runs
python -m src.tests.test_run_sim
python -m src.tests.test_cli
```

---

## 实飞优化过程总结

结合已有多轮实飞结果，当前这套基线的优化过程大致经历了以下几个阶段：

### 1. 早期高误差阶段

主要问题有两个：

- `trajectory` 时间基准存在问题
- follower 控制中缺少前馈补偿

这一阶段的典型现象是：

- 跟踪误差偏大
- leader / follower 参考与实测存在时间错位
- follower 在机动段更容易出现明显滞后

### 2. 中期强化机动阶段

在中期实验里，任务机动强度被进一步加大，用来主动暴露系统在动态跟踪上的瓶颈。

这个阶段的价值不在于追求最低误差，而在于确认：

- 当前误差是否主要来自时间问题
- 在更强机动下 follower 是否明显跟不上
- 哪些问题是控制器本身造成的，哪些问题是参考时序和执行链路带来的

### 3. 稳定期

稳定期的两个关键改动是：

- 加入安全拦截机制
- 加入时间戳对齐机制

这两个改动之后，系统行为开始明显收敛：

- telemetry 与 replay 的解释性更强
- 参考轨迹与实测轨迹的对齐更可信
- HOLD / ABORT 不再只是现象，而能和时序、状态、数据质量对应起来
- 多轮 run 之间的可比性明显增强

### 4. 后期参数收敛阶段

后期优化主要分两步：

1. 先加入前馈补偿
2. 再调整为径向渐变参数

这一阶段的核心目标是：

- 压低 follower 跟踪误差
- 减少大机动时的滞后
- 让控制参数在不同空间位置和不同半径下更平滑、更稳定

从当前多轮对比结果看，这一步已经带来了明显收益，系统也逐渐从“能飞”转向“误差可收敛、结果可解释”的状态。

### 当前结论

当前实飞结果说明，这条优化路径大致是有效的：

- **先解决时序问题**
- **再补控制前馈能力**
- **最后做参数渐变与稳定性收敛**

因此，项目下一步更适合做：

- 最优参数与最优工况的固化
- 各项改动的消融分析
- 不同机动强度、不同场地条件下的边界验证
- 将已有经验整理成可复现的实验结论

### 5. 工程侧运行时优化（非参数侧）

在参数侧基本收敛之后，工程层面又做了一轮从 telemetry 到主循环再到算法的运行时优化（对应已合入 main 的 PR1–PR9 系列）。这一阶段不改控制律、不改 mission 行为，只是把"**同一套决策路径在每帧上做的事情更少、更便宜、不阻塞**"：

- telemetry JSONL 拆成 `header / event / record` 三种 kind，record 不再内嵌 phase_events / config_fingerprint
- telemetry 的 `json.dumps` 与 `fh.write` 移到后台 daemon writer 线程
- `RealMissionApp` 从 1000+ 行巨类拆成 `FailurePolicy` / `LandingFlow` / `MissionTelemetryReporter` 等职责清晰的模块
- AffineFrameEstimator 的 rank + cond 合并为一次 `svdvals`
- FollowerReferenceGenerator 不再复算 rank/cond（权威归 frame）；内部状态改为 ndarray
- FollowerController 内循环向量化，radial scales 一次算出
- 主循环前置拦截由 `SafetyManager.fast_gate` 承担，`safety.evaluate` 每帧只跑一次
- PoseBus 预分配缓冲，不再每次 `np.zeros` 分配
- 组件 init 时缓存 `leader_idx / follower_idx` 数组
- FleetModel getters 改为只读 tuple
- scheduler `_group_drone_ids` 加全集快速路径
- AFCModel 暴露 `steady_state_array(...)` ndarray 变体

这一阶段不主张"**立刻改善飞行表现**"；真正的收益方向是：

- 更紧的 per-tick CPU 预算，为未来加高频控制回路腾空间
- telemetry 不再卡主循环，使长任务的 wall-clock 更可预期
- 把"cond 的权威"等语义收口到单点，避免重复计算漂移
- 模块拆分与契约测试覆盖，使后续再改运行时更安全

### 6. 通信链路层优化（PR10 / PR11）

PR9 之后，工程侧剩下的扰动不再是 per-tick CPU，而是**链路与发包调度**本身。PR10–PR11 这一轮做了两件事：

1. **PR10 可观测 + 默认生效的并行化**：把 cflib `radio_link_statistics` 的 6 个 `Caller` 接到 `LinkQualityBus`，每帧 snapshot 写进 `record.radio_link_quality`；离线 `replay` / `compare-runs` 直接出 `radio_link_summary`。`GroupExecutorPool` 给每个 `radio_group` 一条工作线程，`FollowerExecutor / LeaderExecutor.batch_goto / wait_for_params` 都按组并行；`ro_cache` 启用后 TOC 跨机复用；`connect_all` 报告新增 `parallel` 与 `per_group_duration_s`。
2. **PR11 默认关闭的链路层干预**：`radio_driver: auto / python / cpp` 开关；基于 link_quality 的 tx 降频 + deadband 放大；部分组掉线不再 ABORT 而是推入 parked hold（`fast_gate_group_degrade_enabled`）；disconnect 前有限次数自动重连（`reconnect_enabled`）。这些开关全部默认关，便于真机上逐项 ablation。

这一阶段和 PR1–PR9 一样，不直接宣称飞行表现变好；真正的价值是：

- 链路抖动从"说不清"变成"可定位"：每架 drone 每个时刻的 link_quality / RSSI 都在 telemetry 里
- 同 dongle 串行 + 跨 dongle 并行的语义正确落地，消除了之前跨组发包被 Python 主循环串行化的隐性瓶颈
- 为后续真机 ablation 提供了现成开关，不用再改代码做"要不要并行"、"要不要重连"这种对照

---

## `manual_leader` 模式键位

当前键位映射如下：

- `W / S`：`x + / x -`
- `A / D`：`y + / y -`
- `R / F`：`z + / z -`
- `Q / E`：缩放 `+ / -`
- `Z / X`：绕当前轴旋转 `- / +`
- `C`：切换旋转轴
- `V`：切换控制目标（整体结构或单个 leader）

相关步长和限制位于：

- `config/startup.yaml`

---

## 关键配置文件

### `config/fleet.yaml`

定义：

- drone id
- URI
- role（leader / follower）
- radio_group
- control 参数

### `config/mission.yaml`

定义：

- 任务总时长
- formation type
- nominal positions
- leader motion 模式
- phase 时间表
- trajectory 配置（sample dt、time scale、id、start addr 等）

### `config/comm.yaml`

定义：

- pose / leader / follower 更新频率
- follower deadband
- readiness 相关开关

补充说明：

- 当前这些频率配置仍然是全局阈值
- 但 scheduler 内部已经按 `radio_group` 维护 follower / parked hold 的独立发送状态
- 因此现阶段的含义是：各组共享同一组频率参数，但不会再共享同一个全局发送时间戳

### `config/safety.yaml`

定义：

- boundary
- pose timeout
- max condition number
- max command norm
- estimator variance threshold
- battery threshold
- hold auto-land timeout（当前已在 `config/safety.yaml` 中显式配置）
- velocity stream watchdog action（`telemetry / hold / degrade`）

其中：

- `hold_auto_land_timeout`
  - 控制系统处于 HOLD 状态持续多久后触发自动降落
- `velocity_stream_watchdog_action`
  - 控制 follower velocity stream watchdog 在发现超时发送链路时的处理策略
  - 推荐理解为：
    - `telemetry` = 只记事件
    - `hold` = 立刻进入 HOLD 保护
    - `degrade` = 仅把超时 follower 降级为 hold/parked，不影响其余 follower 的 velocity 路径

### `config/startup.yaml`

定义：

- `startup.mode`
- `manual_leader` 模式下的平移、缩放、旋转步长与限制

### `ConfigLoader` 当前会校验

- drone id 唯一
- leader 数量不少于 `4`
- `nominal_positions` 数量与 drone 数一致
- phase 连续、按时间排序，且从 `t=0` 开始并与 `duration` 对齐
- 各类频率为正
- boundary 合法
- trajectory 参数合法
- `manual_leader` 配置合法
- `velocity_stream_watchdog_action` 必须是 `telemetry / hold / degrade`

---

## 环境说明

仓库当前**没有统一维护的依赖锁定文件**（例如 `requirements.txt` 或 `pyproject.toml`）。

按当前代码使用情况，通常至少需要准备：

- `cflib`
- `numpy`
- `PyYAML`
- `matplotlib`

如果要构建文档，还需要：

- `Sphinx`
- `sphinx-rtd-theme`

---

## 当前限制与诚实描述

当前仓库应当被描述为：

> **一个已经具备真实平台 integration、trajectory 准备、在线 follower 闭环、结构化 telemetry 与离线分析能力的 Crazyflie AFC 实验基线。**

但它**不应被夸大**为：

- fully flight-proven swarm stack
- production-ready system
- 成熟的仿真平台
- 已完全收口的 transport / batching / watchdog 架构

当前已知限制包括：

- `main.py` 是真机导向入口，会等待用户确认并连接硬件
- `src/app/run_sim.py` 现在是最小离线 smoke 入口，但还不是成熟仿真平台
- transport 层仍有继续补强空间
- 默认 `min_vbat = 0.0`，即电池阈值保护默认关闭
- 历史文档和旧产物中仍可能保留旧编队规模或旧说法
- 运行时优化（PR1–PR9 系列）目前主要通过 36 条契约测试 + 离线 smoke 覆盖；**尚未在真机上做配套的基线 vs 优化版 benchmark 对比**——声称的 CPU / IO 节省是基于代码路径推断，不是测量数据
- Telemetry 后台 writer 的队列容量上限是 4096；超出后主循环会丢弃 `record`（计入 `records_dropped`），`event` 始终阻塞 put 保证不丢。这个上限目前是代码常量，尚未暴露到配置

---

## 相关文档

建议优先阅读：

- `doc/重构方案.md`
- `doc/主要信息文档.md`
- `doc/中期报告.md`

如果你想快速了解当前代码入口，建议从这些文件开始：

- `main.py`
- `src/app/bootstrap.py`
- `src/app/run_real.py`
- `src/app/preflight.py`
- `src/app/trajectory_budget_summary.py`
- `src/app/offline_reference_viz.py`
- `src/app/trajectory_comparison.py`

