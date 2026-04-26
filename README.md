# Crazyflie AFC Swarm

> 2026-04-26 状态更新。
>
> 面向 **Crazyflie + Lighthouse + cflib** 的仿射编队控制实验基线，重点在 **真实平台集成、leader 轨迹执行、follower 在线闭环、结构化 telemetry、离线复盘**。

## 当前仓库是什么

这是一个 **真实硬件导向的编队实验基线**，不是纯仿真项目：

- 保留 `affine formation / stress matrix / AFC` 主线
- 采用 `config -> domain -> runtime -> adapters -> app` 分层
- 支持 **auto leader 执行** 与 **manual leader 键盘控制** 两种启动模式
- 提供 trajectory budget dry-run、telemetry replay、多 run 对比等离线工具
- 面向真实平台调试，而不是宣称已 fully flight-proven 或 production-ready

> 历史文档/旧实验记录中可能仍保留 `4 leader + 2 follower` 等旧表述；**当前默认行为以 `config/*.yaml` 和 `src/` 中的实现为准**。

---

## 当前默认配置摘要

`config/` 默认已是 **10 机编队**：

| 项目                    | 默认值                                     |
| ----------------------- | ------------------------------------------ |
| 总数                    | 10                                         |
| leaders                 | `1, 4, 7, 8`                               |
| followers               | `2, 3, 5, 6, 9, 10`                        |
| radio groups            | `0, 1, 2`                                  |
| formation_type          | `pyramid`                                  |
| 任务时长                | `40s`                                      |
| 阶段                    | `settle 0~4` → `trajectory_entry 4~6` → `formation_run 6~40` |
| leader motion           | `affine_rotation`                          |
| trajectory_enabled      | `true`                                     |
| trajectory_time_scale   | `0.95`                                     |
| startup.mode            | `auto`                                     |
| control model           | `dynamics_model_order = 2`                 |
| pose_log_freq           | `10 Hz`                                    |
| follower_tx_freq        | `8 Hz`                                     |
| leader_update_freq      | `1 Hz`                                     |
| parked_hold_freq        | `0.5 Hz`                                   |
| attitude / motor log    | `false / false`（默认关闭诊断日志流）        |
| link_quality telemetry  | `true`                                     |
| link_quality adaptive   | `off`（`link_quality_soft_floor = 0.0`）    |
| reconnect               | `false`（实验性，默认关闭）                  |
| boundary                | `[-1.6, -1.6, -0.1] ~ [1.7, 1.7, 1.8]`    |
| min_vbat                | `3.15`（默认开启电量阈值拦截）              |
| estimator variance      | `0.001`                                    |
| watchdog action         | `degrade`                                  |
| fast_gate group degrade | `true`                                     |

---

## 主要能力

### 1. 分层应用骨架

- `src/domain/` — 编队模型、stress matrix、AFC、mission profile、leader/follower reference
- `src/runtime/` — pose bus、affine frame estimator、follower controller（v1 向量化、v2 二阶前馈）、scheduler、safety manager、telemetry、health、failure policy、landing flow、mission FSM、offline swarm sampler、telemetry replay、link quality / link state bus
- `src/adapters/` — cflib link/transport、Lighthouse pose source、leader/follower executor、group executor pool、radio driver select、键盘手动输入
- `src/app/` — bootstrap、真机主循环、preflight、replay/visualization/comparison CLI

### 2. readiness / preflight

启动流程覆盖 `connect_all` → `connect_group_start/result`（按 `radio_group` 分阶段）→ `wait_for_params` → 可选 `reset_estimator_and_wait` → `onboard_controller` → `pose_ready` → `health_ready` → `trajectory_prepare`。

`PreflightRunner` 当前会检查：

- 至少 4 个 leaders、至少 1 个 follower
- leader affine span 是否有效
- trajectory 启用时已 upload/define 且内存可容纳
- pose snapshot 存在且 fresh、health sample 存在且新鲜
- 无 disconnected drone
- 起飞前位置未越界
- `min_vbat > 0` 时电量达阈值
- `estimator_variance_threshold > 0` 时 Kalman 方差低于阈值

### 3. 两种启动模式

- `auto`：leader 按任务参考运行；启用 trajectory 时在启动阶段 upload/define，运行阶段启动。
- `manual_leader`：键盘实时改变 leader 共时结构参考；follower 仍保留在线闭环。键盘输入适配器基于 Windows `msvcrt`。

### 4. 运行时约束

核心约束：

- 没有新 pose，不重算 follower 控制
- control 计算与无线发包分离
- follower 发包受频率限制与 deadband 限制
- follower / parked hold 的限流按 `radio_group` 拆分
- 同一 `radio_group` 内发包串行，跨 `radio_group` 的 follower、hold、takeoff、land、trajectory start 等动作通过 `GroupExecutorPool` 并行
- safety 动作为 `EXECUTE / HOLD / ABORT`
- 主循环每帧只做一次 `safety.evaluate`；ABORT 级前置拦截由 `SafetyManager.fast_gate_decision(snapshot)` / `fast_gate(snapshot)` 承担（只扫 `disconnected_ids` 与 `boundary`）
- HOLD 只能被 full-safety `EXECUTE` 清除；`hold_auto_land_timeout` 超时后自动降落
- follower velocity stream 带 watchdog；动作可选 `telemetry / hold / degrade`
- 部分 radio group 掉线默认降级为 parked hold；所有 group 掉线或越界仍走 ABORT / emergency land
- `comm.reconnect_enabled=true` 时，纯 disconnect ABORT 前会尝试有限次 reconnect；成功后会重新挂 `ConsoleTap` 与 `LighthousePoseSource` 的 log callbacks

**`velocity_stream_watchdog_action` 行为：**

| 取值         | 行为                                                                |
| ------------ | ------------------------------------------------------------------- |
| `telemetry`  | 只记录事件，不改变调度。                                             |
| `hold`       | 记录事件后立即把超时 follower 切到 HOLD。                             |
| `degrade`    | 记录事件后把超时 follower 降级为 parked hold，其余 follower 不受影响。 |

### 5. Telemetry 与离线分析

真机运行输出 `telemetry/run_real_YYYYMMDD_HHMMSS.jsonl`，使用 `schema_version=2` JSONL 流结构：

- 第 1 行 `kind=header`：schema_version、config_fingerprint、readiness、fleet 摘要
- 后续混合 `kind=event`（结构化事件）与 `kind=record`（每次新 pose 的运行期快照）
- JSON 序列化 + 写盘在 daemon writer 线程上；主循环只 `queue.put`，不阻塞

`record` 常见字段：`measured_positions / leader_ref / follower_ref / frame_validity / condition_number / safety_action / scheduler_diagnostics / follower_command_norms / radio_link_quality / startup_mode / trajectory_state` 等。

离线工具集中在 `src/app/`：

- replay summary — `replay_analysis.py`
- offline reference visualization — `offline_reference_viz.py`
- thesis-style trajectory comparison — `trajectory_comparison.py`
- multi-run comparison — `trajectory_compare_runs.py`

事件码表（`CONNECT_*` / `RUNTIME_WATCHDOG_*` / `RUNTIME_EXECUTOR_*` / `RUNTIME_LINK_RECONNECT_*`）与离线摘要新增字段见 [doc/communication_hardening.md](doc/communication_hardening.md#稳定事件码表)。

### 6. mission_error 三类

- `connection`：链路建立和设备连接阶段失败
- `readiness`：启动、readiness、preflight 与起飞验证阶段失败
- `runtime`：进入主循环之后的运行期异常

与 watchdog / executor 事件共享 `category / code / stage / outcome` 四元语义。

### 7. 工程优化与通信稳定性

当前已沉淀的工程优化：

- **PR1 – PR9 运行时优化**：主循环 fast_gate、Affine SVD 合并、follower controller 向量化、PoseBus 预分配、telemetry 后台写线程、FleetModel 只读 tuple、AFCModel ndarray 变体。详见 [doc/runtime_optimization.md](doc/runtime_optimization.md)。
- **PR10 – PR11 通信稳定性**：`LinkQualityBus` + `LinkStateBus` + `GroupExecutorPool`、跨 radio group 并行、基于 link_quality 的可选 tx 降频、部分组掉线降级、disconnect 前可选有限次自动重连。link-quality 自适应与 reconnect 默认关闭，部分组掉线降级默认开启。详见 [doc/communication_hardening.md](doc/communication_hardening.md)。
- **近期 cflib 对齐优化**：reconnect 成功后重挂 Lighthouse log configs、preflight 检查 Kalman variance、关闭默认诊断日志流（attitude / motor）以降低 log 带宽、测试集增加 `slow` marker 以缩短日常回归。

---

## 仓库结构

```text
config/          # yaml 配置（fleet / mission / comm / safety / startup）
src/
  adapters/      # cflib link/transport、pose source、group executor pool 等
  app/           # bootstrap、run_real、cli、preflight、replay/viz/compare 入口
  config/        # ConfigLoader
  domain/        # 编队模型、AFC、reference
  runtime/       # pose bus、controller、scheduler、safety、telemetry 等
  tests/         # contract tests
telemetry/       # 真机 JSONL 记录
artifacts/       # 离线分析、对比图和运行产物
doc/             # 项目设计与专题说明
docs/            # Sphinx 文档目录
archive/         # 历史脚本、旧实验
scripts/         # 离线扫参与 ablation 脚本
main.py          # 兼容旧入口
```

---

## 常用命令

### 统一 CLI 入口（推荐）

```bash
python -m src.app.cli --help
python -m src.app.cli run --help
python -m src.app.cli compare-runs --help
python -m src.app.cli web --help
```

支持的子命令：

| 子命令          | 作用                                |
| --------------- | ----------------------------------- |
| `run`           | 真机任务入口                        |
| `budget`        | trajectory 内存预算 dry-run（不连接真机） |
| `replay`        | telemetry replay summary            |
| `viz`           | 离线参考轨迹可视化                  |
| `compare`       | 单次 run 理想轨迹对比               |
| `compare-runs`  | 多次 run 汇总与回归检查             |
| `sim`           | 最小离线 smoke test                 |
| `web`           | 本地离线 telemetry / artifacts 浏览界面 |

> `main.py` 保留旧入口兼容行为；`python main.py`、`python main.py --startup-mode ...`、`python main.py --trajectory-budget` 仍可使用。

### 测试

```bash
python -m pytest src/tests -m "not slow" -q   # 日常快测
python -m pytest src/tests -m slow -q         # CLI / trajectory / offline 慢测
python -m pytest src/tests -q                 # 全量
```

日常开发优先跑 `not slow`；`slow` 覆盖 CLI subprocess、`run_sim`、trajectory/offline ablation、baseline sweep 等长耗时测试，适合提交前或改到离线分析/轨迹路径时单独跑。全量 `pytest src/tests` 会包含 slow 测试，可能需要十几分钟。

### 真机主入口

```bash
python -m src.app.cli run
python -m src.app.cli run --startup-mode auto
python -m src.app.cli run --startup-mode manual_leader
python -m src.app.cli run --skip-confirm
```

行为：读取 `config/` → 组装 `build_app("config")` → 等待 Enter → 启动。需要真实 Crazyflie、radio 与 Lighthouse 环境就绪。

**启动状态显示：**

- 默认使用 rich TUI 展示启动阶段（连接 / 等参数 / 重置估计器 / onboard controller / 定位 / 健康 / 轨迹上传 / preflight / takeoff+settle+align；条件阶段按 config 启用）。
- `--verbose` / `-v` 展开每台无人机的 per-step 细节；root logger 切 DEBUG。
- rich 未安装、非 TTY（重定向）、或 `AFC_NO_RICH=1` 时自动降级为 `[HH:MM:SS] [i/N] 阶段名 ...` 滚动文本。
- 每个阶段 begin/ok/fail 以 `startup_phase` 事件写入 telemetry。

### trajectory 预算 dry-run（不连接真机）

```bash
python -m src.app.cli budget --config-dir config
```

输出每个 leader 的 piece 数、预计字节数、起始地址与是否 fit memory；摘要带出 `config_dir` 与 `startup_mode`。

### telemetry replay summary

```bash
python -m src.app.cli replay telemetry/run_real_YYYYMMDD_HHMMSS.jsonl
```

真机默认写入**带时间戳**的 jsonl，建议显式传入路径。replay / comparison 产物保留 `config_fingerprint` 以便确认多 run 是否同一套配置。

### 离线参考轨迹可视化

```bash
python -m src.app.cli viz \
  --config-dir config \
  --output-dir artifacts/offline_reference_viz
```

默认输出 `swarm_reference.png` 与 `swarm_reference.gif`。支持 `--dt / --fps / --trail / --total-time`。

### 单次 run 对理想轨迹对比

```bash
python -m src.app.cli compare telemetry/run_real_YYYYMMDD_HHMMSS.jsonl
python -m src.app.cli compare telemetry/run_real_YYYYMMDD_HHMMSS.jsonl --include-all-phases
```

默认输出到 `artifacts/<telemetry_stem>/`，生成 `trajectory_comparison_summary.json` 与 overlay / tracking error 图；不传路径时自动选最新 `run_real_*.jsonl`。默认统计 `formation_run`，加 `--include-all-phases` 后图表和指标使用全任务记录。

### 多次 run 摘要对比

```bash
python -m src.app.cli compare-runs \
  artifacts/run_real_20260401_235152 \
  artifacts/run_real_20260402_020853 \
  --output artifacts/compare_runs.json
```

汇总 formation / leader / follower / communication 指标；额外生成 `compare_overview.png / compare_roles.png / compare_communication.png`。

### 回归阈值检查

```bash
python -m src.app.cli compare-runs \
  artifacts/run_real_20260401_235152 \
  artifacts/run_real_20260402_020853 \
  --output artifacts/compare_runs_regression.json \
  --formation-rmse-threshold 0.11 \
  --frame-valid-threshold 0.82
```

支持 `--formation-rmse-threshold / --frame-valid-threshold / --leader-rmse-threshold / --follower-rmse-threshold`；结果新增 `regression_checked / regression_thresholds / failing_runs` 与每 run 各指标的 `passed / failed` 状态。

### 离线 smoke test

```bash
python -m src.app.cli sim --config-dir config --dt 0.25 --total-time 10
python -m src.app.cli sim --config-dir config --dt 0.25 --total-time 10 --output artifacts/offline_smoke.json
```

最小离线 smoke runner，用于快速检查配置装载、leader/follower 参考链路、affine frame 与 follower validity、mission phase 与 leader mode。**不是成熟仿真器**。

### 本地 Web 分析界面

```bash
python -m src.app.cli web --host 127.0.0.1 --port 8000 \
  --telemetry-dir telemetry \
  --artifacts-dir artifacts
```

Web 模块基于 FastAPI / Jinja2，面向离线 telemetry 与 artifacts 浏览。当前定位是分析辅助入口，不替代 CLI replay / compare-runs 主流程。

### 离线调参与 ablation 脚本

`scripts/` 下保留离线扫参与 ablation 脚本；专题结论若当前 checkout 带有 `doc/offline_tuning_findings.md`，可结合该文档阅读：

- `generate_baseline_sweep.py` — Phase 1A 一阶基线
- `generate_delay_compensation_ablation.py` — Phase 1B delay compensation
- `generate_trajectory_condition_ablation.py` — Phase 1C condition penalty
- `generate_second_order_baseline_sweep.py` — Phase 2A 二阶基线
- `generate_model_order_ablation.py` — Phase 2B 一阶 vs 二阶

### 部分测试示例

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

## `manual_leader` 模式键位

| 键     | 作用                         |
| ------ | ---------------------------- |
| `W/S`  | `x +/-`                     |
| `A/D`  | `y +/-`                     |
| `R/F`  | `z +/-`                     |
| `Q/E`  | 缩放 `+/-`                   |
| `Z/X`  | 绕当前轴旋转 `-/+`            |
| `C`    | 切换旋转轴                   |
| `V`    | 切换控制目标（整体结构/单个 leader） |

步长与限制位于 `config/startup.yaml`。

---

## 关键配置文件

### `config/fleet.yaml`

`drones[]`（id / uri / role / radio_group）+ 共享 `control` 参数（gain、max_velocity、feedforward、damping、时滞补偿等）。

### `config/mission.yaml`

`duration / formation_type / nominal_positions / leader_motion / phases / trajectory 配置（sample dt、time scale、id、start addr 等）`。

### `config/comm.yaml`

pose / leader / follower 更新频率、follower deadband、readiness 开关、连接分组并行、trajectory upload 分组并行、telemetry 队列参数（`telemetry_queue_max / telemetry_flush_every_n`）、Lighthouse 诊断日志开关（`attitude_log_enabled / motor_log_enabled`）、PR10/PR11 连接与链路质量字段（`connect_pace_s / connect_timeout_s / link_quality_enabled / radio_driver / link_quality_soft_floor / reconnect_*` 等）。当前频率是**全局阈值**，但 scheduler 内部按 `radio_group` 维护独立发送状态，不再共享全局发送时间戳。

### `config/safety.yaml`

`boundary / pose_timeout / max_condition_number / max_command_norm / estimator_variance_threshold / hold_auto_land_timeout / velocity_stream_watchdog_action / min_vbat / fast_gate_group_degrade_enabled`。

### `config/startup.yaml`

`startup.mode` 与 `manual_leader` 下的平移、缩放、旋转步长与限制。

### `ConfigLoader` 校验项

drone id 唯一、leader 数 ≥ 4、`nominal_positions` 与 drone 数一致、phase 连续按时间排序且从 `t=0` 起、各类频率为正、boundary 合法、trajectory 参数合法、`manual_leader` 配置合法、`velocity_stream_watchdog_action ∈ {telemetry, hold, degrade}`、telemetry 队列参数合法、PR10/PR11 comm 字段合法、`full_state` 模式要求二阶控制并建议 onboard controller 使用 Mellinger / INDI。

---

## 环境说明

仓库通过 `requirements.txt` 维护运行依赖范围，并通过 `pyproject.toml` 收口 pytest 配置。主要依赖：

- `cflib>=0.1.30,<0.2`（代码使用 0.1.x 公开 API；部分注释/测试覆盖到 0.1.31 行为）
- `numpy`
- `scipy`
- `cvxpy`
- `PyYAML`
- `matplotlib`
- `fastapi` / `uvicorn` / `jinja2`（离线分析 Web UI）

可选：

- `cflinkcpp`（`comm.radio_driver=cpp` 时）
- `Sphinx` / `sphinx-rtd-theme`（构建 `docs/` 时）
- `rich`（真机 `run` 启动阶段 TUI；未安装时自动降级为滚动文本）

---

## 当前限制与诚实描述

当前仓库应被描述为：

> **一个已具备真实平台 integration、trajectory 准备、在线 follower 闭环、结构化 telemetry 与离线分析能力的 Crazyflie AFC 实验基线。**

**不应被夸大为：**

- fully flight-proven swarm stack
- production-ready system
- 成熟的仿真平台
- 已完全收口的 transport / batching / watchdog 架构

**已知限制：**

- `main.py` / `src/app/cli.py run` 都是真机入口，会等待用户确认并连接硬件
- `src/app/run_sim.py` 是最小离线 smoke 入口，不是成熟仿真器
- 自动 reconnect 默认关闭；开启前建议先单独做 ablation，确认 Lighthouse log 重挂、pose freshness 与 recovery telemetry 符合预期
- 默认 `min_vbat = 3.15`，电池阈值保护默认开启
- `config/fleet.yaml` 的 control 默认值**不等于** Phase 1A Baseline 最佳参数
- 历史文档和旧产物中可能保留旧编队规模或旧说法
- PR1 – PR11 与近期 cflib 对齐优化主要靠 contract tests 覆盖；**尚未在真机上做基线 vs 优化版 benchmark 对比**，声称的 CPU / IO 节省是基于代码路径推断
- Telemetry 后台 writer 队列上限默认 4096（`comm.telemetry_queue_max`），默认每 50 条 record flush 一次（`comm.telemetry_flush_every_n`）；超出后 `record` 会被丢弃计入 `records_dropped`，`event` 始终阻塞 put 保证不丢
- Executor 连续失败阈值默认 2（`safety.executor_group_failure_streak`）
- `attitude_log_enabled=false`、`motor_log_enabled=false` 是默认保守带宽配置；需要姿态 / 电机诊断时可在 `comm.yaml` 显式开启

---

## 相关文档

**项目专题：**

当前分支可能包含本地 `doc/` 专题文档（例如运行时优化、通信稳定性、离线调参结论、历史设计文档）。如果你的 checkout 没有带这些文件，以本 README、`config/*.yaml`、`src/` 实现和 `src/tests/` 契约为准；避免把未随分支分发的 `doc/` 文件当成唯一入口。

**代码入口速查：**

- [main.py](main.py) — 兼容旧入口
- [src/app/cli.py](src/app/cli.py) — 统一 CLI
- [src/app/bootstrap.py](src/app/bootstrap.py) — `build_app`
- [src/app/run_real.py](src/app/run_real.py) — 真机主循环
- [src/app/preflight.py](src/app/preflight.py) — preflight 检查
- [src/app/trajectory_budget_summary.py](src/app/trajectory_budget_summary.py)
- [src/app/offline_reference_viz.py](src/app/offline_reference_viz.py)
- [src/app/trajectory_comparison.py](src/app/trajectory_comparison.py)
