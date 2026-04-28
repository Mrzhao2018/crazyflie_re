# 运行时优化（PR1 – PR9）

> 2026-04-19 记录。本文集中说明已合入 `main` 的主循环、算法、遥测层工程优化，对应 commit 范围 `0959b81 -> 5de6864`。
>
> 目标不是追求新功能，而是压低每帧 CPU、减少阻塞 I/O、让热路径更可控。默认行为保持不变，除非另行标注。

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

---

## 模块级细节

### 主循环（`src/app/run_real.py`）

- 每帧只做一次 `safety.evaluate`，ABORT 级前置检查改用 `SafetyManager.fast_gate(snapshot)`。`fast_gate` 只扫 `disconnected_ids` 与 `boundary`，不构造 `SafetyReason` 对象。
- `health_bus.latest()` 每帧只调用一次，存进 loop-local 变量后由 `safety.evaluate()` 与 telemetry record 复用。
- HOLD 状态只能被 full-safety `EXECUTE` 清除；`hold_entered_at` 不会被前置检查误 reset，`hold_auto_land_timeout` 能正确触发。
- 仍然保留 `sleep(0.01)` 轮询结构；没有引入 `threading.Event`，避免与 `pose_source` 回调线程产生新的竞态面。

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

### PoseBus（`src/runtime/pose_bus.py`）

- `latest()` 复用预分配的 `_scratch_positions` / `_scratch_fresh` 缓冲；每次调用只 `fill(0) + 填入 + .copy()` 给返回值，避免高频 `np.zeros((n,3))` 分配。
- snapshot 之间仍然互相隔离（`.copy()` 保证），`update_agent` / `has_newer_than` 语义不变。

### CommandScheduler（`src/runtime/scheduler.py`）

- `_group_drone_ids` 全集快速路径：当 `drone_ids == follower_ids` 或 `leader_ids` 全集时，直接返回预缓存的 `_follower_groups` / `_leader_groups`，不再 `for d in members if d in drone_set`。
- 子集 / 未知 id 仍走慢路径，行为向后兼容。

### Safety（`src/runtime/safety_manager.py`）

- `__init__` 一次性把 `boundary_min/max` 转成 ndarray，边界检查用 `np.any(P < bmin, axis=1)` 向量化。
- `fast_gate` 与 `evaluate` 共享同一套 cached boundary 数组。

### AFC 模型（`src/domain/afc_model.py`）

- 提供 `steady_state_array(leader_positions)`，直接返回 `(n_f, 3)` ndarray + `tuple(follower_ids)`，方便上层继续走向量运算。
- 原 `steady_state(leader_positions) -> dict` 作为薄包装保留，向后兼容。

### Telemetry（`src/runtime/telemetry.py`）

- JSON 序列化 + 写盘移到 daemon writer 线程；主循环只 `queue.put(("record"/"event"/"header", payload))`，热路径零阻塞。
- `record` 走 `_json_safe_record` 快速路径（针对 `TelemetryRecord.asdict()` 的字段结构），避免深度递归 `isinstance` 链。
- `record` 批量 flush；`event` / `header` 立即 flush。`close()` 用 sentinel drain 并 join。
- 队列容量上限 4096：极端情况下 `record` 可能被丢弃并计入 `summary()["records_dropped"]`；`event` 始终阻塞 put（关键事件不丢）。

### FleetModel（`src/domain/fleet_model.py`）

- `all_ids() / leader_ids() / follower_ids() / get_group_members()` 改为返回 `tuple`（只读），不再每次 `.copy()` 一份 list。调用方若需要 mutate，请显式 `list(fleet.xxx_ids())`。

---

## 语义差异（需要留意）

这两处是向后兼容窗口之外的语义差异：

- `follower_ref.frame_condition_number` 现为 `nan`（cond 权威移到 `frame.condition_number`）。
- `LOW_BATTERY`（`min_vbat > 0` 时）触发从 pre-safety 移到 full-safety，晚一帧才进入 emergency_land；默认 `min_vbat = 0.0` 无影响。

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
- telemetry schema、config 字段、mission 行为均未改变。

---

## 价值与边界

这一阶段不主张"立刻改善飞行表现"；真正的收益方向是：

- 更紧的 per-tick CPU 预算，为未来加高频控制回路腾空间。
- telemetry 不再卡主循环，使长任务的 wall-clock 更可预期。
- 把"cond 的权威"等语义收口到单点，避免重复计算漂移。
- 模块拆分与契约测试覆盖，使后续再改运行时更安全。

**边界：**

- 尚未在真机上做"基线 vs 优化版" benchmark 对比，当前声称的 CPU / IO 节省是基于代码路径推断，不是测量数据。
- Telemetry 后台 writer 的队列容量上限（4096）目前是代码常量，尚未暴露到配置。
