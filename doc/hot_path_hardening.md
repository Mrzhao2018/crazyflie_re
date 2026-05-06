# Host-Side Hot Path Hardening

> 2026-04-23 记录。本文汇总 `Host-Side Quick Wins` 这一轮 host-side 清理与加固，
> 重点是把主循环热路径中的同步 I/O、wall-clock 速率比较、链路状态观测缺口和几处
> 硬编码常量收口成更稳的默认实现。范围只覆盖 Python host 侧，不含 firmware 改动、
> `run_real.py` 大拆分、trajectory 压缩编码或大幅调参。
>
> 2026-05-06 补充：当前工作树又把 full-state/Mellinger 的 setpoint ownership、HOLD/landing 路径、leader trajectory 启动验动和 active drone scope 纳入 host-side 加固范围。它们仍然是 host 侧工程路径，不改 firmware。

---

## 本轮包含什么

这批改动对应 10 个小 PR：

| PR | 主题 | 主要文件 |
| --- | --- | --- |
| PR1 | 删除 debug hot-path 文件写 | `src/app/run_real.py` / `src/runtime/follower_controller_v2.py` / `.gitignore` |
| PR2 | full-state NaN/Inf/int16 越界防御 | `src/adapters/cflib_command_transport.py` |
| PR3 | 去掉多余 `link_statistics.start()` | `src/adapters/cflib_link_manager.py` |
| PR4 | `pytest` 统一入口 + requirements pin + lazy logging | `pyproject.toml` / `src/tests/conftest.py` / `requirements.txt` |
| PR5 | rate-limit / watchdog 时钟迁到 `time.monotonic()` | `src/runtime/scheduler.py` / `src/runtime/failure_policy.py` / `src/adapters/cflib_command_transport.py` |
| PR6 | telemetry queue / flush / executor streak 暴露到 config | `src/config/schema.py` / `src/runtime/telemetry.py` / `src/runtime/failure_policy.py` / `src/app/bootstrap.py` |
| PR7 | 安全默认值翻转：`min_vbat=3.15`、`fast_gate_group_degrade_enabled=true` | `config/safety.yaml` / `README.md` |
| PR8 | reset_estimator 阶段展示 `kalman.varP*` | `src/app/run_real.py` |
| PR9 | Lighthouse log 通路改为 callback 异步模式 | `src/adapters/lighthouse_pose_source.py` |
| PR10 | `LinkStateBus` + `connection_lost/disconnected` 观测 + telemetry 镜像 | `src/runtime/link_state_bus.py` / `src/adapters/cflib_link_manager.py` / `src/runtime/safety_manager.py` / `src/app/run_real.py` |

---

## 关键变化

### 1. 主循环不再做同步 debug 文件写

- 删除 `src/runtime/debug_session_logger.py` 与 `debug-2fa5ef.log`
- 去掉 `run_real.py` 和 `follower_controller_v2.py` 里为该文件服务的热路径 debug 调用
- 保留已有 `TelemetryRecorder` 作为唯一结构化运行时记录出口

收益：主循环 / controller compute 不再出现 `open(..., "a")` + `json.dumps(...)` + `write()` 的同步磁盘路径。

### 1.1 当前分支：full-state 热路径不再提前 commit 内部状态

`FollowerControllerV2` 的 full-state smoothing 状态现在分成两个阶段：

- `compute(...)` 只生成本帧候选 full-state reference，并把 `target_position / target_velocity / t_meas` 放入 `FollowerCommandSet.full_state_state`。
- `RealMissionApp` 只有在 `FollowerExecutor.execute_velocity(...)` 成功返回对应 follower id 后，才调用 `commit_full_state_state(...)`。
- 进入 HOLD、从 HOLD 恢复、follower reference 缺失或 pose stale 时，调用 `reset_full_state_state(...)` 清掉对应 follower 的 committed state。

旧路径的问题是：只要 controller 算出了目标，就会把 smoothing 内部状态向前推进；如果这一帧因为 scheduler blocked、group stale 或 executor failure 没真正发出去，下一帧就会从“未下发过的目标”继续平滑，导致 host 内部轨迹和飞机实际收到的轨迹不一致。现在 commit 跟 executor success 绑定，热路径语义更清楚。

### 1.2 当前分支：derived feedforward 与 raw feedforward 分离

full-state 模式下，host 侧仍不做闭环控制，但现在不再简单把 velocity / acceleration 全置零：

- position reference 先经过 smoothing 和 `full_state_max_position_step` 限幅。
- velocity 由相邻两帧**已平滑 position target**差分得到，再受 `max_velocity` 限幅。
- acceleration 由相邻两帧 derived velocity 差分得到，再受 `max_acceleration` 限幅。
- AFC reference 的 raw velocity / acceleration 若存在，只在 diagnostics 里记录为 ignored，不直接混入下发 tuple。

这样避免“位置被平滑，速度/加速度却来自 raw jump”的不一致。Telemetry diagnostics 会区分：

- `derived_feedforward_followers`
- `derived_acceleration_followers`
- `raw_feedforward_ignored_followers`
- `raw_acceleration_ignored_followers`

### 2. full-state setpoint 在 host 侧主动拒绝坏输入

`src/adapters/cflib_command_transport.py` 新增：

- `FULL_STATE_INT16_LIMIT = 32.0`
- `_validate_full_state_vector(...)`

`cmd_full_state()` 现在会在下发前 reject：

- `NaN`
- `+Inf / -Inf`
- 任意分量 `|v| > 32.0`

原因：cflib `send_full_state_setpoint()` 内部会把 pos/vel/acc 量化成 int16（mm / mm·s / mm·s²），越界时会在更底层炸 `struct.pack('<h', ...)`。现在统一转成 `ValueError`，被上层归类为 `invalid_command / retryable=false`。

### 3. 所有 cadence / watchdog 比较都基于 monotonic clock

迁移后的内部时间源：

- `CommandScheduler.last_follower_tx_time`
- `CommandScheduler.last_leader_update_time`
- `CommandScheduler.last_hold_tx_time`
- `CflibCommandTransport._last_velocity_command_time`
- `FailurePolicy.hold_entered_at`

仍保留 wall-clock 的字段：

- `TelemetryRecord.t_wall`
- `PoseSnapshot.t_meas`
- `HealthSample.t_meas`

收益：rate-limit / watchdog / hold timeout 不再受 NTP 校时或人工改系统时间影响。

### 4. Telemetry / failure policy 的硬编码变成显式配置

新增配置字段：

```yaml
# config/comm.yaml
telemetry_queue_max: 4096
telemetry_flush_every_n: 50

# config/safety.yaml
executor_group_failure_streak: 2
velocity_stream_watchdog_factor: 12.0
velocity_stream_watchdog_degrade_streak: 5
fast_gate_group_degrade_streak: 4
runtime_pose_jump_hold_streak: 2
leader_trajectory_start_verify_delay_s: 3.0
leader_trajectory_start_min_displacement_m: 0.06
leader_trajectory_start_max_retries: 1
```

对应行为：

- `TelemetryRecorder(queue_max_size=..., flush_every_n=...)`
- `FailurePolicy` 读取 `config.safety.executor_group_failure_streak`
- velocity stream watchdog 读取 factor 和 streak；未达 streak 时只记录 stale details
- fast-gate partial group disconnect 读取 degrade streak；未达 streak 时只记录 pending
- runtime pose jump/speed/vz 异常读取 hold streak；未达 streak 时 severity 为 `TELEMETRY`
- leader trajectory start 读取 verify delay / min displacement / retry 次数

收益：长任务可以单独放大 queue；executor、watchdog、fast-gate、pose jump、leader trajectory start 的干预阈值都可以做实验性调参，不再需要改代码。

### 5. 安全默认值从“全关”改成“保守可用”

2026-04-23 这一轮曾把默认值调成更保守：

- `min_vbat: 3.15`
- `fast_gate_group_degrade_enabled: true`

语义：

- 电池阈值默认启用，起飞前 / 运行时都检查
- 单 `radio_group` 掉线时优先降级该组 follower，而不是默认整队 ABORT

这两项都可以在 YAML 里显式改回旧值做对照。

2026-05-06 当前工作树为了 full-state/Mellinger 实飞排障，默认又临时调整为：

- `min_vbat: 0.0`
- `estimator_variance_threshold: 0.0`
- `velocity_stream_watchdog_action: telemetry`
- `velocity_stream_watchdog_factor: 12.0`
- `velocity_stream_watchdog_degrade_streak: 5`
- `fast_gate_group_degrade_enabled: true`
- `fast_gate_group_degrade_streak: 4`

这不是“最终安全默认值”的结论，而是为了先排除电池阈值、Kalman 方差阈值和短时通信毛刺对控制器排障的干扰。恢复真实任务默认时，应把电池/方差阈值重新打开，并根据单机 probe 与多机 telemetry 收紧 watchdog/fast-gate streak。

### 6. LighthousePoseSource 改成官方推荐的 callback 模式

旧路径：

```text
cflib incoming thread
  -> SyncLogger queue
  -> per-drone Python worker thread
  -> _on_pose_data / _on_health_data
```

新路径：

```text
cflib incoming thread
  -> LogConfig.data_received_cb
  -> _dispatch_log_packet(...)
  -> _on_pose_data / _on_health_data / _on_attitude_data
```

实现上：

- 去掉 `SyncLogger`
- 去掉 `_logger_worker`
- `stop()` 改为 `log_conf.stop() + remove_callback + delete()`
- 在 health / attitude payload 里额外附带 `cf_log_timestamp_ms`

收益：少一层 queue、少每机一条 worker thread，更贴近 Bitcraze 官方 Python API 的推荐用法。

### 7. 新增 `LinkStateBus`

新增模块：`src/runtime/link_state_bus.py`

数据来源：

- `cf.connection_lost`
- `cf.disconnected`
- `connect_all()` / `reconnect()` 成功后的 `mark_connected(...)`

消费方：

- `SafetyManager.fast_gate / fast_gate_decision`：把 `LinkStateBus.disconnected_ids()` 与 `snapshot.disconnected_ids` 合并
- `RealMissionApp._record_link_state_events()`：把边沿变化镜像成 `link_state_change` telemetry event

收益：掉线不必等 `pose_timeout` 超时后才被 fast gate 感知。

### 8. 当前分支：active drone scope 减少无关连接/日志工作

active follower scope 现在不只影响 scheduler，而是贯穿启动和定位：

- `CflibLinkManager` 只连接 active drone ids。
- `LighthousePoseSource` 只为 active drone ids 创建 log config。
- `PoseBus` 的 freshness 快速路径只遍历 active drone ids。
- `wait_for_params_per_group` 与 `reset_estimator_per_group` 只处理 active drone ids。

这直接降低 leader-only / single-follower 排障时的连接、TOC、log callback、pose snapshot 处理负担。更重要的是，它消除了 inactive follower 造成的 readiness false negative。

### 9. 当前分支：leader trajectory start 下发可测量

`LeaderExecutor._execute_start_trajectory(...)` 现在为每台 leader 记录：

- `start_monotonic_s`
- `done_monotonic_s`
- `duration_ms`
- `start_offset_ms`

并汇总 `send_skew_ms`。`trajectory_start` event 同时记录 requested 和 effective parameters。当前 effective `time_scale` 临时固定为 `1.0`，用于回滚实飞中“传配置 time scale 后 HLC 不动”的路径。

热路径收益不是 CPU，而是诊断闭环：如果 leader 轨迹不同步，可以区分“下发 skew 太大”“某台下发失败”“下发成功但 firmware 未运动”。

### 10. 当前分支：HOLD / landing 不再混用不匹配的控制接口

full-state follower 在运行期由 Mellinger 接 full-state setpoint；因此 HOLD 和 safety land 不再简单复用 high-level hold/land：

- `_execute_hold_actions(...)` 在 full-state 模式下用当前位置 full-state setpoint 做 hold，并写 `full_state_hold_setpoint`。
- safety / emergency land 先通过 streaming direct descent 下发向下速度或递减 z target，写 `follower_direct_descent_execution`。
- 非紧急 orderly land 才把 follower 切回 PID，再发 high-level land，写 `landing_onboard_controller`。
- `_notify_streaming_setpoint_stop(...)` 仍用于清理低层 setpoint ownership。

这样避免 Mellinger 正在接收 full-state 流时突然被 high-level commander 接管，减少 terminal path 上的控制权竞态。

---

## pytest 入口统一

仓库历史上绝大多数测试文件是“脚本风格”：

- 顶层构造对象
- 直接执行
- 模块级 `assert`
- 结尾 `print("[OK] ...")`

现在通过 `src/tests/conftest.py` 的 `pytest_collect_file` hook，把这类 `test_*.py` 包成单个 `ScriptTestItem`。因此两种调用方式都有效：

```bash
python -m src.tests.test_run_real_watchdog
python -m pytest src/tests
```

---

## 新增/更新的 focused tests

新增：

- `src/tests/test_cflib_command_transport_full_state.py`
- `src/tests/test_telemetry_queue_config.py`
- `src/tests/test_failure_policy_streak_config.py`
- `src/tests/test_run_real_reset_estimator_progress.py`
- `src/tests/test_lighthouse_pose_source_async.py`
- `src/tests/test_link_state_bus.py`
- `src/tests/test_link_manager_connection_lost.py`

更新：

- `src/tests/test_link_quality_wire.py`
- `src/tests/test_config_loader.py`
- `src/tests/test_config_loader_connect_fields.py`
- `src/tests/test_preflight.py`
- `src/tests/test_scheduler_link_quality.py`
- `src/tests/test_scheduler_parked_hold_watchdog.py`
- `src/tests/test_run_real_watchdog.py`
- `src/tests/run_real_fixtures.py`

当前分支继续新增/更新：

- `src/tests/test_full_state_circle_probe.py`
- `src/tests/test_hlc_trajectory_probe.py`
- `src/tests/test_leader_trajectory_start_watchdog.py`
- `src/tests/test_param_probe.py`
- `src/tests/test_reset_estimator_group_parallel.py`
- `src/tests/test_follower_controller_v2_regressions.py`
- `src/tests/test_run_real_follower_regressions.py`
- `src/tests/test_leader_executor_group_parallel.py`
- `src/tests/test_follower_executor_group_parallel.py`
- `src/tests/test_cli.py`
- `src/tests/test_config_loader.py`

---

## 边界

这一轮**不包含**：

- firmware 侧 Mellinger / HLC 行为修复
- 真机 benchmark：没有测量 CPU、latency 或 radio airtime 的基线 vs 当前对比
- `run_real.py` 1500 行拆分
- broadcast / appchannel 配套

当前工作树虽然已经接入 `poly4d_compressed`、full-state probes 和 leader trajectory start watchdog，但这些仍是 host-side 诊断与下发路径。若后续确认 HLC `time_scale`、Mellinger 参数或 compressed trajectory 在 firmware 侧有版本差异，需要单独记录 firmware / cflib 版本和实飞矩阵，不能把 host-side hardening 当作 firmware 结论。

这些应作为下一轮独立改动处理，避免把“host-side hardening”与“行为/性能策略改变”混在一起。
