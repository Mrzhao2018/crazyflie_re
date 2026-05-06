# 通信稳定性优化（PR10 / PR11）

> 2026-04-19 记录。本文集中说明 PR10 / PR11 两个 batch 在链路层与上层发包调度上的改动，对应 commit `e24c458`。
>
> PR10 默认启用、行为兼容；PR11 中 link-quality 自适应与自动重连默认关闭，部分组掉线降级默认开启，便于在真机上逐项 ablation。
>
> 2026-05-06 补充：当前工作树在 PR10/PR11 基础上继续补了 full-state/Mellinger 实飞排障、active drone scope、follower entry align、leader trajectory 启动验动，以及 watchdog / fast-gate 的 streak 化。本文保留 PR10/PR11 背景，同时记录这些尚未单独 commit 的运行链路变化。

---

## 为什么专门抽出这一层

PR1 – PR9 收口了主循环、算法、遥测的热路径；PR10 – PR11 的重点放在**底层链路与上层发包调度的耦合**——这是 PR1 – PR9 触不到、但真机上真正导致"说不清的通信不稳"的地方。

- 链路抖动从"说不清"变成"可定位"：每架 drone 每个时刻的 link_quality / RSSI 都在 telemetry 里。
- 同 dongle 串行 + 跨 dongle 并行的语义正确落地，消除了之前跨组发包被 Python 主循环串行化的隐性瓶颈。
- 为后续真机 ablation 提供了现成开关，不用再改代码做"要不要并行"、"要不要重连"这种对照。

---

## 2026-05-06 当前分支补充

这轮改动的核心不是再加一个无线优化开关，而是把“通信不稳”和“控制器没接上”拆开看。之前全队任务失败时，很难判断是 HLC trajectory 没启动、follower 没对齐、full-state handoff 瞬间跳了、还是某个 radio group 真掉线；当前分支把这些阶段拆成可观察事件，并尽量让每个阶段只连接必要的飞机。

### active drone scope

`active_follower_ids` 现在影响连接、参数等待、reset estimator、pose ready、health ready、Lighthouse log attach 和 PoseBus freshness 统计：

- `null` 表示全部 follower 参与，当前默认即 6 个 follower 全部参与。
- `[]` 表示 leader-only 排障：只连接 leader、只等 leader 参数和 pose，不会因为 follower 未连接阻塞 leader HLC trajectory probe。
- 非空列表表示只让指定 follower 起飞、切 runtime controller 并接收 setpoint。

对应实现点：

- `CflibLinkManager(active_drone_ids=...)` 只按 active id 分组连接。
- `LighthousePoseSource(active_drone_ids=...)` 只挂 active id 的 log config。
- `PoseBus(active_drone_ids=...)` 只把 active id 纳入 freshness 快速路径。
- `wait_for_params_per_group(..., drone_ids=...)` 和 `reset_estimator_per_group(..., drone_ids=...)` 只处理 active id。

这对实飞排障很关键：可以先让 4 个 leader 完整跑通 trajectory，再逐步加入 follower，避免一开始就把“某台 follower 没电/没定位”误判成全局链路问题。

### startup 阶段通信拆分

启动流程现在更细：

1. `connect_all` 仍按 radio group 分阶段连接，组内串行，组间可并行。
2. `wait_for_params` 可按 group 并行，进度按 active drone 数统计。
3. `reset_estimator_and_wait` 也可按 group 并行，并在进度细节里显示 `kalman.varP*` 最大值。
4. 设置 onboard controller 时，full-state 模式下 follower 先保持 PID，leader 仍按配置 controller；runtime 前再对 follower 切 Mellinger / INDI。
5. `onboard_param_overrides` 会在 readiness 中逐机写入，例如当前默认 `ctrlMel.massThrust=95000`，并记录 `set_onboard_param_override` 事件。
6. pose / health ready 只要求 active drone 就绪。

这避免了两个旧问题：一是 follower full-state 控制器过早接管 HLC 起飞，二是 inactive follower 仍然占用连接和 readiness 判定。

### follower entry align

leader 到达任务起始/trajectory entry 位置后，follower 不再假设自己“已经差不多在正确队形”。`RealMissionApp._align_followers_to_entry_reference(...)` 会：

- 使用 planned entry leader positions 或当前 measured leader frame 计算 follower reference。
- 通过 `FollowerExecutor.go_to_positions(...)` 对 active follower 下发 high-level `go_to`。
- 等待 `follower_align_duration_s + follower_align_settle_s`。
- 用 pose snapshot 校验每个 follower 到目标位置的误差和 freshness。
- 写入 `follower_entry_align` telemetry，包含目标、实测、误差、freshness、tolerance 和 failed ids。

默认参数：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `startup.follower_align_enabled` | `true` | 可关闭对照实验 |
| `startup.follower_align_duration_s` | `2.0` | go_to 时长 |
| `startup.follower_align_settle_s` | `0.5` | 到点后稳定时间 |
| `startup.follower_align_tolerance_m` | `0.25` | 验证误差阈值 |
| `startup.start_stabilize_s` | `1.0` | 进入主循环前的起始点缓冲 |

align 失败被归为 readiness failure，并触发落地；这样不会带着明显错位的 follower 进入 formation_run。

### leader trajectory start 验动

leader trajectory start 现在不是“命令发出就算开始”。`LeaderExecutor` 会记录每台 leader 的 `hl_start_trajectory` 下发起止单调时间、duration 和跨组 send skew；`RealMissionApp` 随后按配置延迟检查 leader 是否真的离开起始点：

| 字段 | 当前配置 | 作用 |
| --- | --- | --- |
| `safety.leader_trajectory_start_verify_delay_s` | `3.0` | start 后等多久检查位移 |
| `safety.leader_trajectory_start_min_displacement_m` | `0.06` | 认为“动了”的最小位移 |
| `safety.leader_trajectory_start_max_retries` | `1` | 没动时最多重发几次 start |

相关事件：

- `trajectory_start`：记录 attempt、reason、moving leader ids、requested/effective time scale、result/send timings。
- `leader_trajectory_motion_confirmed`：全部预期运动 leader 达到位移阈值。
- `leader_trajectory_motion_unconfirmed`：到检查时间后仍未全部达到阈值。
- `leader_trajectory_start_retry`：完全没动且还有 retry 额度时重发 start。
- `leader_trajectory_start_failed`：重试后仍没动或只有部分 leader 动，进入 orderly land。

注意：当前 `LeaderExecutor` 暂时把 HLC start 的实际 `time_scale` 固定为 `1.0`，并把配置请求值写入 `requested_time_scale`。这是为了回滚“把配置 time scale 传给 firmware 后 leader 不动”的实飞路径；后续如果确认 firmware/cflib 行为稳定，再恢复配置透传。

### full-state setpoint ownership

full-state/Mellinger 路径现在把 setpoint ownership 明确写入 telemetry：

- runtime controller 切换前后各发一次 `full_state_handoff_setpoint`，目标是当前 measured pose，避免 Mellinger 接手第一帧跳到离散 reference。
- `full_state_warmup` 使用同一批 handoff targets，不重新从过期内部状态推导。
- `streaming_setpoint_active` 增加 `execute_duration_s`、`dt_since_last_streaming_setpoint`、`sent_groups`、`blocked_groups`、`stale_groups`。
- HOLD 时 full-state follower 通过 `full_state_hold_setpoint` 继续发当前位置零速度 setpoint，而不是 high-level hold。
- 安全落地时 follower 先走 `follower_direct_descent_execution`，非紧急 orderly land 才切回 PID 并发 high-level land。

这些事件用于判断“setpoint 没发出去”“setpoint 发慢了”“Mellinger 接手瞬间 reference 错了”三类问题。

### streak 化保护

当前默认比 PR10/PR11 更偏向“先观测，再干预”：

| 保护 | 当前默认 | 行为 |
| --- | --- | --- |
| velocity stream watchdog | `action=telemetry`、`factor=12.0`、`streak=5` | 只记录 stale；若切到 `hold/degrade`，需连续 stale 达 streak |
| fast-gate group degrade | `enabled=true`、`streak=4` | 单组掉线先记录 pending，连续观察到后才 parked hold |
| runtime pose jump | `hold_streak=2` | 第一次 jump/speed/vz 异常以 telemetry 严重度记录，连续异常才 HOLD |

这样做牺牲了一点自动干预速度，换来更低的 false positive 风险，适合当前还在定位 full-state / HLC trajectory 行为的阶段。恢复更保守实飞策略时，可以把 watchdog action 改回 `degrade`，把 streak 下调，并重新打开 `min_vbat` / estimator variance 阈值。

---

## PR10 — 可观测 + 组间并行（默认启用，行为兼容）

### 链路质量观测

- `LinkQualityBus`（`src/runtime/link_quality_bus.py`）：汇聚每架 drone 的 `link_quality / uplink_rssi / uplink_rate / downlink_rate / uplink_congestion / downlink_congestion`，来源是 cflib `cf.link_statistics` 的 6 个 `Caller`。
- `CflibLinkManager` 在 `open_link` 前挂回调；`cf.link_statistics.start()/stop()` 由 cflib 自己挂在 connected/disconnected callback 上管理，项目侧不重复启动。`bus=None` 时不挂采集回调，避免无消费方白白消耗。
- `TelemetryRecord.radio_link_quality: dict[drone_id, metrics]` 字段；`replay` / `compare-runs` 汇总 `radio_link_summary.overall / per_drone` + `radio_link_worst_drone_min`。

### 组间并行

- `GroupExecutorPool`（`src/adapters/group_executor_pool.py`）：每 `radio_group` 一条工作线程；`FollowerExecutor.execute_velocity / execute_hold / takeoff / land / stop_velocity_mode`、`LeaderExecutor` 的 high-level 动作、`wait_for_params_per_group(...)` 都走这个池子。
- 组内保序（同 dongle 本来就要串行）、组间真并行。
- `CflibLinkManager` 默认启用 `ro_cache=./cache/ro / rw_cache=./cache/rw`：TOC 可跨机复用。
- `connect_all` 报告扩展：`parallel` 标记、`per_group_duration_s`，便于离线判断 connect 阶段瓶颈。

### PR10 相关 `config/comm.yaml` 字段

| 字段                    | 默认值 | 作用                                                      |
| ----------------------- | ------ | --------------------------------------------------------- |
| `connect_pace_s`        | `0.2`  | 多机连接之间的 sleep；实飞稳定后可下调到 `0.05` 做冷启动提速。 |
| `connect_timeout_s`     | `5.0`  | 单机 `open_link` 超时；radio 拥塞时可抬到 `10.0`。             |
| `link_quality_enabled`  | `true` | 开启后接入 cflib `radio_link_statistics`，写入 telemetry。     |
| `attitude_log_enabled`  | `false` | 诊断日志流：roll/pitch/yaw/thrust；默认关闭以降低 log 带宽。 |
| `motor_log_enabled`     | `false` | 诊断日志流：motor.m1~m4；默认关闭，核心 vbat/kalman variance 保留。 |

---

## PR11 — 链路层干预（默认关闭，逐项 opt-in）

### 驱动切换

- `comm.radio_driver: auto / python / cpp`（`src/adapters/radio_driver_select.py`）：cflib 通过 `USE_CFLINK` 环境变量与 `cflib.crtp.CLASSES` 决定驱动选择。
- 选 `cpp` 需要 `pip install cflinkcpp`，缺失时抛 `RuntimeError`。
- 选 `python` 会强制清除 `USE_CFLINK` 环境变量。

### 基于 link_quality 的自适应降频

- `CommandScheduler` 接收 `link_quality_provider(group_id) -> float|None`。
- 当 `link_quality_soft_floor > 0` 时，对低质量组做 `follower_tx_interval × link_quality_backoff_scale`，对该组 follower 的 `deadband × link_quality_deadband_scale`。
- `diagnostics.link_quality_backoff_groups` 暴露受抑制的组。

### 部分组掉线不 ABORT

- `SafetyManager.fast_gate_decision(snapshot)` 返回 `FastGateDecision(action, reason_codes, degrade_groups)`。
- 全组掉线或越界 → `ABORT`；**部分组**掉线 → `HOLD_GROUP` + `degrade_groups`。
- `safety.fast_gate_group_degrade_enabled=true` 时 `run_real` 走新路径，`FailurePolicy.apply_fast_gate_group_degrade` 把这些组的 follower 推进 `watchdog_degraded_followers`。
- 旧 `fast_gate()` 签名保持不变。

### 有限次自动重连

- `CflibLinkManager.reconnect(drone_id, *, attempts, backoff_s, timeout_s)` 做有限次重连：close 旧 scf → open 新 scf。
- `FailurePolicy.attempt_reconnect(drone_ids)` 消费 `comm.reconnect_*`，记录 `RUNTIME_LINK_RECONNECT_{ATTEMPT,OK,FAILED}` 事件。
- `run_real` 在 fast_gate ABORT + 纯 disconnect 时会先尝试 reconnect，全部成功则跳过 emergency_land。

### PR11 相关 `config/comm.yaml` 字段

| 字段                                           | 默认值  | 作用                                                           |
| ---------------------------------------------- | ------- | -------------------------------------------------------------- |
| `radio_driver`                                 | `auto`  | `auto / python / cpp`。选 `cpp` 需要 `cflinkcpp`。              |
| `link_quality_soft_floor`                      | `0.0`   | `> 0` 时启用自适应；推荐值 `60.0`。                             |
| `link_quality_backoff_scale`                   | `1.5`   | 低质量链路 `follower_tx_interval` 倍数。                        |
| `link_quality_deadband_scale`                  | `2.0`   | 低质量链路 `follower_cmd_deadband` 倍数。                       |
| `safety.fast_gate_group_degrade_enabled`       | `true`  | `true` 时部分组掉线只降级为 parked hold。                         |
| `comm.reconnect_enabled`                       | `false` | `true` 时纯 disconnect 前先尝试有限次重连。                      |
| `comm.reconnect_attempts`                      | `2`     | 重连次数上限。                                                  |
| `comm.reconnect_backoff_s`                     | `0.5`   | 重连之间的 backoff 秒。                                         |
| `comm.reconnect_timeout_s`                     | `5.0`   | 重连 open_link 超时。                                           |

---

## 既有 comm/watchdog 能力（Phase 1 – 3 累积）

这一部分不是 PR10/PR11 新增，但它们是 PR10/PR11 能落地的前提。

### group-aware 限流与 parked hold

当前 Phase 1 通信链路已经具备：

- **group-aware follower rate limit**：不同 `radio_group` 的 follower velocity 发包独立限流。
- **group-aware parked hold**：degrade / parked follower 会按组下发 hold，不与活跃 velocity 路径互相阻塞。
- **mixed planning**：同一轮 scheduler plan 中可以同时出现 active follower velocity 和 parked follower hold。
- scheduler diagnostics 已暴露 group 级信息，例如 `parked_group_counts`、`hold_tx_groups_sent`、`follower_tx_groups_sent`。

### `velocity_stream_watchdog_action` 行为差异

| 取值         | 行为                                                                                                         |
| ------------ | ------------------------------------------------------------------------------------------------------------ |
| `telemetry`  | 只记录 `velocity_stream_watchdog` 事件；不改变调度与执行路径。                                                |
| `hold`       | 记录 watchdog 事件；同一 follower stale streak 达阈值后切到 HOLD，`hold_entered` 带稳定 runtime 事件码。          |
| `degrade`    | 记录 watchdog 事件；同一 follower stale streak 达阈值后放入降级集合，后续经 `parked_follower_ids` 下发 hold；支持 recovered 事件。 |

### Executor 失败结构化

- 失败结果从纯文本扩展为结构化字段：`command_kind / error_type / failure_category / retryable / radio_group`。
- 同一 `radio_group` 的 retryable failure 连续达到 `safety.executor_group_failure_streak` 次时，触发 `executor_group_degrade`。
- 同一 `radio_group` 一旦出现 non-retryable failure，立即进入组级策略。
- 全部活跃 follower group 都满足触发条件时直接进入 `HOLD`；部分组触发则把对应 follower 放入降级集合。
- 当前默认连续失败阈值为 `2`，已经暴露为 `safety.executor_group_failure_streak`。

---

## 稳定事件码表

所有事件使用 `category / code / stage / outcome` 四元语义，方便离线工具直接 groupby。

### Connection（连接阶段失败）

| Event code                       | 说明                                                                 |
| -------------------------------- | -------------------------------------------------------------------- |
| `CONNECT_GROUP_START`            | 某个 `radio_group` 开始连接。                                         |
| `CONNECT_GROUP_SUCCESS`          | 该组全部成员成功建立链路。                                            |
| `CONNECT_GROUP_PARTIAL_FAILURE`  | 部分成员成功、后续失败；strict 策略因此中止启动。                       |
| `CONNECT_GROUP_FAILED`           | 进入连接阶段后没有任何成员成功建链。                                   |
| `CONNECT_ALL_OK`                 | 全部 group 连接完成；`connect_all` 事件携带 `radio_groups` 摘要。        |
| `CONNECT_ALL_FAILED`             | 整体失败；携带 `failed_group_ids` 与整体 `outcome`。                    |

### Watchdog（运行期保护动作）

| Event code                               | 说明                                          |
| ---------------------------------------- | --------------------------------------------- |
| `RUNTIME_VELOCITY_STREAM_WATCHDOG`       | 触发 velocity stream 超时检测。                |
| `RUNTIME_WATCHDOG_HOLD`                  | 超时 follower 被切到 HOLD。                     |
| `RUNTIME_WATCHDOG_DEGRADE`               | 超时 follower 降级为 parked hold。              |
| `RUNTIME_WATCHDOG_DEGRADE_RECOVERED`     | 降级的 follower 恢复正常。                      |

说明：watchdog 事件属于保护动作，不一定意味着任务立即失败；当前事件 details 会包含 `watchdog_factor / stale_streak / required_streak`，只有 action 为 `hold/degrade` 且 streak 达阈值时才执行干预，后续演化为 `ABORT` 或终止路径时才进入终止语义。

### Executor（发包失败策略）

| Event code                       | 说明                                                            |
| -------------------------------- | --------------------------------------------------------------- |
| `RUNTIME_EXECUTOR_GROUP_DEGRADE` | 某 `radio_group` 连续失败达到阈值，组级降级。                    |
| `RUNTIME_EXECUTOR_GROUP_HOLD`    | 全部活跃 follower group 均满足触发条件，直接进入 HOLD。            |

### Link reconnect（PR11 新增）

| Event code                           | 说明                                       |
| ------------------------------------ | ------------------------------------------ |
| `RUNTIME_LINK_RECONNECT_ATTEMPT`     | 正在尝试 reconnect。                        |
| `RUNTIME_LINK_RECONNECT_OK`          | reconnect 成功。                            |
| `RUNTIME_LINK_RECONNECT_FAILED`      | reconnect 失败，进入后续 ABORT / land 流程。   |

### 其他 PR10/PR11 事件（非 mission_error）

- `fast_gate_group_degrade`：单组掉线被降级为 parked hold 而非整队 ABORT 时记录。
- `fast_gate_group_degrade_pending`：单组掉线已被 fast gate 观察到，但尚未达到 `fast_gate_group_degrade_streak`。
- `link_reconnect_attempt / ok / failed`：`comm.reconnect_enabled=true` 时才会出现。
- `link_state_change`：`LinkStateBus` 记录到的 `connected / disconnected` 边沿，被主线程镜像到 telemetry；携带 `RUNTIME_LINK_STATE_CHANGE` 事件码、`drone_id / state / error / t_wall`。

### 当前分支新增运行事件

| Event | 说明 |
| --- | --- |
| `set_onboard_param_override` | readiness 中写入 firmware 参数覆盖，失败会中止启动。 |
| `follower_entry_align` | follower 起始队形对齐结果，含目标/实测/误差/freshness。 |
| `full_state_handoff_setpoint` | runtime controller 切换前后发给 follower 的 full-state handoff setpoint。 |
| `full_state_hold_setpoint` | full-state HOLD 时下发的当前位置零速度 setpoint。 |
| `streaming_setpoint_active` | follower streaming setpoint 活跃事件；现在带执行耗时和 group 级 diagnostics。 |
| `trajectory_start` | leader HLC trajectory start 尝试；带 requested/effective time scale 和 send timings。 |
| `leader_trajectory_motion_confirmed` | start 后 leader 位移达到阈值。 |
| `leader_trajectory_motion_unconfirmed` | start 后 leader 位移未达到阈值。 |
| `leader_trajectory_start_retry` | 未检测到运动时重发 start。 |
| `leader_trajectory_start_failed` | 重试后仍失败，触发 orderly land。 |
| `start_stabilize` | startup 完成前的起始点稳定缓冲。 |
| `landing_onboard_controller` | 非紧急 orderly land 前把 full-state follower 切回 PID。 |
| `follower_direct_descent_execution` | safety/emergency land 中 follower streaming direct descent 的执行摘要。 |

---

## 离线摘要新增字段

PR10 之后，`replay` 与 `compare-runs` 的摘要新增：

- `radio_link_summary.overall`：`count / min / mean / p5 / max`
- `radio_link_summary.per_drone`：`{drone_id: {count, min, mean, p5, max}}`
- `radio_link_worst_drone_min`：所有 drone 中最小的 `link_quality.min`
- `radio_link_sample_count`、`radio_link_overall_{min,mean,p5,max}`（compare-runs 专有）
- 旧 run 或 `link_quality_enabled=false` 时 `overall.count == 0`，旧文件兼容解析。

`executor_failure_summary`（Phase 3 后半段起）：

- `total / by_code / by_action / by_event / by_group / failure_categories / retryable_counts`

`compare-runs --output ...` 额外产物：

- `compare_communication.png`：同时展示 watchdog 总数、executor failure 总数、executor degrade 次数、executor hold 次数。

---

## 契约覆盖面

PR10/PR11 原始覆盖面包括：

- `test_link_quality_bus / test_link_quality_wire`
- `test_telemetry_radio_link / test_replay_radio_link_summary / test_compare_runs_radio_link`
- `test_comm_config_link_quality / test_config_loader_connect_fields`
- `test_group_executor_pool / test_follower_executor_group_parallel / test_leader_executor_group_parallel / test_wait_for_params_parallel`
- `test_link_manager_connect_report / test_link_manager_reconnect`
- `test_radio_driver_select / test_scheduler_link_quality / test_safety_fast_gate_group`
- 全部既有 `run_real / scheduler / safety / telemetry` 回归。

当前分支新增/扩展的覆盖面包括：

- CLI parser 覆盖 `probe-velocity`、`probe-full-state-circle`、`probe-params`。
- config loader 覆盖 full-state 默认值、`onboard_param_overrides`、watchdog factor/streak、pose jump streak、follower align、leader trajectory start verification、`active_follower_ids=[]`。
- `run_real` 回归覆盖 fast-gate pending streak、full-state send 后才 commit controller state、HOLD reset full-state state、full-state hold setpoint、direct descent landing、structured safety reasons。
- single-drone probe 单元测试覆盖 full-state circle reference、param filtering、leader trajectory start watchdog 等关键工具逻辑。

注意：这轮默认配置已经改变，不能再说“默认行为未改变”。当前默认是 full-state/Mellinger 实飞排障配置，`link_quality` 自适应和 reconnect 仍默认关闭，`cflinkcpp` 仍为可选依赖。

---

## 边界

- `telemetry_queue_max=4096`、`telemetry_flush_every_n=50`、`executor_group_failure_streak=2`、watchdog factor/streak、fast-gate streak 都已暴露到配置；但 direct descent 的 2s / 0.45mps / 10Hz 当前仍是 landing flow 常量。
- `link_quality_soft_floor` 推荐值 `60` 来自 cflib 惯例，真机条件下还需实测校准。
- PR11 中 link-quality adaptive 与 reconnect 默认关闭；fast-gate group degrade 默认开启但带 streak。开启 reconnect 或 link-quality backoff 前建议先单条开关 ablation，避免同时改动多条开关导致因果难判。
- HLC start `effective_time_scale=1.0` 是实飞排障回滚点，不是长期设计结论。
- 当前 watchdog 默认 `telemetry`，不会自动 hold/degrade；实飞时需要结合人工监控、boundary ABORT 与后续配置收紧。
