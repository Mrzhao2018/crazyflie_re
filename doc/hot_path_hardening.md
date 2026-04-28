# Host-Side Hot Path Hardening

> 2026-04-23 记录。本文汇总 `Host-Side Quick Wins` 这一轮 host-side 清理与加固，
> 重点是把主循环热路径中的同步 I/O、wall-clock 速率比较、链路状态观测缺口和几处
> 硬编码常量收口成更稳的默认实现。范围只覆盖 Python host 侧，不含 firmware 改动、
> `run_real.py` 大拆分、trajectory 压缩编码或大幅调参。

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
```

对应行为：

- `TelemetryRecorder(queue_max_size=..., flush_every_n=...)`
- `FailurePolicy` 读取 `config.safety.executor_group_failure_streak`

收益：长任务可以单独放大 queue；executor 的 group failure 阈值可做实验性调参，不再是代码常量。

### 5. 安全默认值从“全关”改成“保守可用”

当前默认：

- `min_vbat: 3.15`
- `fast_gate_group_degrade_enabled: true`

语义：

- 电池阈值默认启用，起飞前 / 运行时都检查
- 单 `radio_group` 掉线时优先降级该组 follower，而不是默认整队 ABORT

这两项都可以在 YAML 里显式改回旧值做对照。

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

---

## 边界

这一轮**不包含**：

- trajectory memory 压缩编码（`POLY4D_COMPRESSED`）
- leader cadence / follower tx 频率的默认调参
- `run_real.py` 1500 行拆分
- firmware / broadcast / appchannel 配套

这些应作为下一轮独立改动处理，避免把“host-side hardening”与“行为/性能策略改变”混在一起。
