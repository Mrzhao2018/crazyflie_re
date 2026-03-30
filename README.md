# Crazyflie AFC Swarm

一个面向 **Crazyflie + Lighthouse + cflib** 的仿射编队控制实验仓库。当前代码基线对应的是 **cflib-stage V2**：

- **4 个 leader + 2 个 follower**
- leader 走高层参考执行路径（支持低频同步参考与 trajectory prepare/start）
- follower 保留在线 velocity 闭环
- 重点解决 **时序清晰、通信受控、适合中期前真实平台验证** 的问题

这不是一个“所有能力都完全定型”的最终系统，更准确的表述是：

> **当前仓库已经具备一个可信的 cflib-stage V2 integration baseline**，
> 主要架构、readiness/preflight、health/safety、trajectory、telemetry/replay 都已经落地，
> 但 transport 层完整性、默认 trajectory 运行基线、以及真实飞行稳定性验证仍在继续收尾。

---

## 1. 项目目标

本仓库的目标不是“把所有东西都在线闭环跑起来”，而是围绕下面这条主线逐步收口：

- **算法正确**：保留 affine formation / stress matrix / AFC 这条理论与仿真主线
- **时序清晰**：没有新 pose，不重算 follower 控制
- **通信可控**：控制计算与无线发包解耦，leader 尽量从在线链路中移除
- **便于迁移**：未来可向 Crazyswarm2 / ROS2 风格执行架构过渡

对应的当前实验形态是：

- 6 架 Crazyflie
- 4 leader + 2 follower
- Lighthouse 定位
- cflib 执行链路

---

## 2. 当前状态：做到哪了

结合 `doc/重构方案.md` 与 `doc/主要信息文档.md`，当前仓库状态可以概括为：

### 已经明显对齐的部分

- **四层分离已经真实落地**：`config / domain / runtime / adapters / app`
- **运行时关键不变量已进代码**：
  - 4 leader 共时参考
  - follower 只在新 pose 到来时重算
  - control 与 tx 分离
  - leader / follower 命令族隔离
- **preflight / readiness 已成型**：
  - wait-for-params
  - 可选 estimator reset
  - pose ready
  - affine span / boundary / battery / health 检查
- **trajectory 支持已接通**：
  - leader trajectory prepare / upload / define / start
  - 支持自动生成 Poly4D 片段
- **telemetry / replay 已可用**：
  - JSONL 记录
  - phase events
  - readiness / health / scheduler / safety 数据
  - 离线 replay analysis CLI

### 还属于“部分对齐”的部分

- **默认 mission 配置并未启用 trajectory**：代码支持，但 `config/mission.yaml` 默认仍是 `trajectory_enabled: false`
- **transport 层完整度还没到最终形态**：已有 rate limit / deadband，但尚未完成更完整的 radio-group batching / watchdog fallback
- **测试覆盖已经不小，但仍更像 contract-script 集合**，不是完整成熟的测试体系

### 当前不建议对外夸大的点

不要把当前仓库描述成：

- fully flight-proven
- production-ready
- fully synchronized swarm stack
- already migrated to the recommended final architecture

更诚实的表述是：

> **V2 核心骨架已落地，且具备真实平台 integration / debugging / replay 的基础能力，但真实飞行稳定性与 transport 层完整性仍需继续验证。**

---

## 3. 当前架构

当前代码主结构如下：

```text
config/
domain/
runtime/
adapters/
app/
experimental/
tests/
```

### 3.1 Domain 层

负责数学与任务定义：

- `fleet_model.py`
- `formation_model.py`
- `stress_matrix_solver.py`
- `afc_model.py`
- `mission_profile.py`
- `leader_reference.py`
- `follower_reference.py`

这里已经承担：

- nominal formation
- stress matrix / AFC 相关计算
- mission phases
- affine transform
- leader trajectory / reference spec

### 3.2 Runtime 层

负责在线状态、控制、调度与记录：

- `pose_snapshot.py`
- `pose_bus.py`
- `affine_frame_estimator.py`
- `follower_controller.py`
- `mission_fsm.py`
- `safety_manager.py`
- `scheduler.py`
- `telemetry.py`
- `telemetry_replay.py`
- `health_bus.py`

### 3.3 Adapter 层

负责 cflib 相关接入：

- `cflib_link_manager.py`
- `cflib_command_transport.py`
- `lighthouse_pose_source.py`
- `leader_executor.py`
- `follower_executor.py`

### 3.4 App 层

负责组装与运行入口：

- `bootstrap.py`
- `preflight.py`
- `run_real.py`
- `replay_analysis.py`

---

## 4. 当前主运行链路

默认真实运行入口在：

- `main.py`

运行过程大致是：

1. 读取 `config/`
2. 组装 `build_app("config")`
3. `RealMissionApp.start()`
   - connect all
   - wait-for-params
   - 可选 reset estimator
   - 启动 Lighthouse pose source
   - 等待 pose ready
   - trajectory prepare（若 mission 为 trajectory 模式）
   - preflight
   - takeoff
   - settle
4. `RealMissionApp.run()`
   - 取 snapshot
   - pre-safety
   - 若有新 pose，则 frame -> follower ref -> follower control
   - safety
   - scheduler
   - leader/follower executor
   - telemetry

### 当前已经在代码里保护的关键运行约束

- 没有新 pose，不重算 follower 控制
- follower 正常控制受 scheduler 发包频率限制
- follower 发包还受 command deadband 限制
- safety 输出只分 `EXECUTE / HOLD / ABORT`
- readiness / trajectory prepare / preflight 都会留下结构化记录

---

## 5. 配置说明

当前使用的是仓库根目录下的：

- `config/fleet.yaml`
- `config/mission.yaml`
- `config/comm.yaml`
- `config/safety.yaml`

### 5.1 `fleet.yaml`

定义：

- drone id
- URI
- leader / follower role
- radio_group
- control 参数

当前默认配置是：

- 6 架飞机
- leaders: `1, 2, 3, 4`
- followers: `5, 6`

### 5.2 `mission.yaml`

定义：

- mission duration
- formation type
- nominal positions
- leader motion mode
- mission phases
- trajectory 相关参数

当前默认值里：

- `leader_motion.mode = "affine_rotation"`
- `trajectory_enabled = false`

也就是说：

> trajectory 代码路径已经支持，但默认 mission 并没有直接启用 trajectory。

### 5.3 `comm.yaml`

定义：

- `pose_log_freq`
- `follower_tx_freq`
- `leader_update_freq`
- `parked_hold_freq`
- `follower_cmd_deadband`
- readiness 开关

### 5.4 `safety.yaml`

定义：

- boundary min/max
- pose timeout
- max condition number
- max command norm
- estimator variance threshold
- min battery voltage

### 5.5 配置校验

`ConfigLoader` 会校验：

- 至少 4 leaders
- `nominal_positions` 数量与 drone 数一致
- phase 时间有序且不重叠
- 各类频率为正
- boundary 合法
- trajectory start addr 合法
- battery 阈值合法

---

## 6. trajectory 支持现状

当前仓库已具备 leader trajectory 的基础能力：

- 自动生成 Poly4D trajectory pieces
- upload 到 cflib trajectory memory
- define trajectory
- `RUN` 阶段发 `start_trajectory`

### 当前自动生成策略

如果：

- `trajectory_enabled = true`
- 且没有手写 `trajectory_pieces`

则 `MissionProfile` 会自动生成 trajectory。

当前已经支持：

- **phase-aware slicing**
- **phase 内按固定 `trajectory_sample_dt` 做 keyframe-style 切段**
- **leader-specific nominal target**

也就是说，不同 leader 不再共享同一条“代理轨迹”，而是基于各自 nominal target 生成分段轨迹规格。

### 当前还没做到的事

还没有做到：

- 高阶平滑优化
- yaw 轨迹规划
- 更复杂的 trajectory synthesis / fitting

所以它目前更像：

> **工程上可用的简化 Poly4D 生成器**，而不是最终轨迹优化器。

---

## 7. preflight / readiness / safety

### 7.1 readiness

当前 startup 支持：

- `wait_for_params`
- 可选 `reset_estimator_and_wait`
- `pose_ready`
- trajectory prepare

这些结果会被写入：

- readiness report
- phase event telemetry

### 7.2 preflight

当前 `PreflightRunner` 检查：

- leader 数量是否为 4
- follower 数量是否为 2
- affine span 是否有效
- pose 是否可用且 fresh
- 是否有 disconnected drone
- 初始位置是否越界
- health sample 是否存在
- `pm.vbat` 是否高于阈值

### 7.3 safety

当前 `SafetyManager` 已结构化输出 reason code，例如：

- `DISCONNECTED`
- `OUT_OF_BOUNDS`
- `FRAME_INVALID`
- `FRAME_DEGENERATE`
- `FOLLOWER_REF_INVALID`
- `COMMAND_SATURATED`
- `LOW_BATTERY`

---

## 8. telemetry 与 replay

当前 telemetry 会记录：

- `mission_state`
- `readiness`
- `phase_events`
- `snapshot_seq`
- `snapshot_t_meas`
- `health`
- `frame_valid`
- `frame_condition_number`
- `safety_action`
- `safety_reason_codes`
- `scheduler_reason`
- `scheduler_diagnostics`
- `leader_action_count`
- `follower_action_count`
- `follower_command_norms`

默认输出路径：

- `telemetry/run_real.jsonl`

### 离线 replay 分析

当前可直接运行：

```bash
python -m src.app.replay_analysis telemetry/run_real.jsonl
```

如果不传路径，默认读取：

- `telemetry/run_real.jsonl`

当前 replay summary 支持：

- `record_count`
- `event_counts`
- `safety_counts`
- `scheduler_reason_counts`
- `first_mission_state`
- `last_mission_state`
- `valid_frame_count`
- `frame_valid_rate`
- `frame_condition_min/max`
- `max_command_norm_per_drone`

因此当前 telemetry 已经不只是“日志文件”，而是一个轻量试飞复盘工具的基础版本。

---

## 9. 当前测试覆盖

当前 `src/tests/` 已覆盖多个关键路径，包括：

- `test_run_real.py`
- `test_startup_flow.py`
- `test_preflight.py`
- `test_safety_manager.py`
- `test_scheduler.py`
- `test_mission_fsm.py`
- `test_mission_profile.py`
- `test_leader_trajectory.py`
- `test_cflib_adapters.py`
- `test_telemetry.py`
- `test_telemetry_replay_analysis.py`
- `test_pose_bus.py`
- `test_affine_frame.py`
- `test_integration.py`

### 说明

这些测试已经足够说明：

- 当前仓库不是“只有架构图，没有行为约束”

但也要诚实说明：

- 测试形态很多仍然是轻量 contract-script 风格
- 真实 cflib / Lighthouse / 多 radio / 实飞鲁棒性仍不能只靠仓库内测试证明

---

## 10. 运行方式

### 10.1 主入口

```bash
python main.py
```

注意：

- `main.py` 当前是**真实硬件导向入口**
- 运行时会等待用户按 Enter 后启动
- 需要真实 Crazyflie 连接

### 10.2 replay 分析

```bash
python -m src.app.replay_analysis telemetry/run_real.jsonl
```

### 10.3 离线参考轨迹可视化

可以直接基于当前 `config/` 和 `build_core_app("config")` 的任务/参考管线，离线生成全机群参考轨迹图：

```bash
python -m src.app.offline_reference_viz --config-dir config --output-dir artifacts/offline_reference_viz
```

默认会输出：

- `artifacts/offline_reference_viz/swarm_reference.png`
- `artifacts/offline_reference_viz/swarm_reference.gif`

可选参数包括：

- `--dt`：离线采样时间步长
- `--fps`：GIF 帧率
- `--trail`：动画中每架无人机显示的历史轨迹长度
- `--total-time`：只渲染任务前一部分时长，便于快速 smoke test

### 10.4 测试示例

```bash
python -m src.tests.test_run_real
python -m src.tests.test_startup_flow
python -m src.tests.test_mission_profile
python -m src.tests.test_telemetry_replay_analysis
python -m src.tests.test_integration
python -m src.tests.test_offline_swarm_sampler
python -m src.tests.test_offline_reference_viz
```

---

## 11. 当前限制与已知事实

README 里最重要的是不要夸大。当前仓库应当被描述为：

### 可以明确说的

- cflib-stage V2 架构已落地
- readiness / preflight / health / safety / scheduler / telemetry / replay 已成型
- leader trajectory 支持已打通到 prepare / define / start
- follower 在线 velocity 闭环主线清楚

### 不该夸大的

- 不是 fully flight-proven
- 不是 production-ready swarm stack
- 不是完整 Crazyswarm2/ROS2 架构
- transport 层还没完全做到文档中的最终形态
- 默认 mission 不是 trajectory-on by default
- `src/app/run_sim.py` 目前并不是成熟的 simulation entrypoint

---

## 12. 当前建议如何描述这个项目

如果你要在中期答辩、README、或对外介绍里概括当前状态，我建议用下面这句：

> **本仓库已经实现一个面向 Crazyflie + Lighthouse + cflib 的 V2 分层执行基线：支持 4 leader + 2 follower 的仿射编队实验主流程，具备 readiness、preflight、安全决策、trajectory 支持、结构化 telemetry 与离线 replay 分析能力；但 transport 层完整性、默认 trajectory 基线及真实飞行鲁棒性仍需继续验证。**

---

## 13. 后续最自然的方向

如果继续沿当前方向推进，最自然的是：

1. 继续提高 leader trajectory 质量
2. 继续增强 replay/analysis 输出
3. 补强 transport 层的 radio grouping / batching / watchdog
4. 做更多真实平台验证而不是只扩功能

---

## 14. 相关文档

更完整的设计背景见：

- `doc/重构方案.md`
- `doc/主要信息文档.md`

这两个文档定义了仓库当前对齐的目标基线，也是理解当前 V2 结构的最佳入口。
