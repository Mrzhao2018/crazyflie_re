# 通信稳定性优化（PR10 / PR11）

> 2026-04-19 记录。本文集中说明 PR10 / PR11 两个 batch 在链路层与上层发包调度上的改动，对应 commit `e24c458`。
>
> PR10 默认启用、行为兼容；PR11 中 link-quality 自适应与自动重连默认关闭，部分组掉线降级默认开启，便于在真机上逐项 ablation。

---

## 为什么专门抽出这一层

PR1 – PR9 收口了主循环、算法、遥测的热路径；PR10 – PR11 的重点放在**底层链路与上层发包调度的耦合**——这是 PR1 – PR9 触不到、但真机上真正导致"说不清的通信不稳"的地方。

- 链路抖动从"说不清"变成"可定位"：每架 drone 每个时刻的 link_quality / RSSI 都在 telemetry 里。
- 同 dongle 串行 + 跨 dongle 并行的语义正确落地，消除了之前跨组发包被 Python 主循环串行化的隐性瓶颈。
- 为后续真机 ablation 提供了现成开关，不用再改代码做"要不要并行"、"要不要重连"这种对照。

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
| `hold`       | 记录 watchdog 事件后，立即把超时 follower 切到 HOLD；`hold_entered` 带稳定 runtime 事件码。                     |
| `degrade`    | 记录 watchdog 事件后，将超时 follower 放入降级集合，后续经 `parked_follower_ids` 下发 hold；支持 recovered 事件。 |

### Executor 失败结构化

- 失败结果从纯文本扩展为结构化字段：`command_kind / error_type / failure_category / retryable / radio_group`。
- 同一 `radio_group` 的 retryable failure 连续达到 2 次时，触发 `executor_group_degrade`。
- 同一 `radio_group` 一旦出现 non-retryable failure，立即进入组级策略。
- 全部活跃 follower group 都满足触发条件时直接进入 `HOLD`；部分组触发则把对应 follower 放入降级集合。
- 当前连续失败阈值是代码内常量 `2`，还没有暴露到配置。

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

说明：watchdog 事件属于保护动作，不一定意味着任务立即失败；只有后续演化为 `ABORT` 或终止路径时才进入终止语义。

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
- `link_reconnect_attempt / ok / failed`：`comm.reconnect_enabled=true` 时才会出现。
- `link_state_change`：`LinkStateBus` 记录到的 `connected / disconnected` 边沿，被主线程镜像到 telemetry；携带 `RUNTIME_LINK_STATE_CHANGE` 事件码、`drone_id / state / error / t_wall`。

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

PR10/PR11 带来 41 条契约测试（24 既有 + 17 新），全部通过：

- `test_link_quality_bus / test_link_quality_wire`
- `test_telemetry_radio_link / test_replay_radio_link_summary / test_compare_runs_radio_link`
- `test_comm_config_link_quality / test_config_loader_connect_fields`
- `test_group_executor_pool / test_follower_executor_group_parallel / test_leader_executor_group_parallel / test_wait_for_params_parallel`
- `test_link_manager_connect_report / test_link_manager_reconnect`
- `test_radio_driver_select / test_scheduler_link_quality / test_safety_fast_gate_group`
- 全部既有 `run_real / scheduler / safety / telemetry` 回归。

telemetry schema、config 字段、mission 行为在**默认配置下**均未改变。接入 cflib 0.1.30 的所有路径都走公开 API；`cflinkcpp` 为可选依赖。

---

## 边界

- 连续失败阈值 `2`、watchdog 队列上限 `4096` 仍为代码常量，未暴露到配置。
- `link_quality_soft_floor` 推荐值 `60` 来自 cflib 惯例，真机条件下还需实测校准。
- PR11 全部开关默认关闭；开启前建议先在 `comm.yaml` 里以单条开关做 ablation，避免同时改动多条开关导致因果难判。
