# Crazyflie AFC Swarm

> 2026-04 状态更新。
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
  - pose bus、affine frame estimator、follower controller、scheduler、safety、telemetry、health
- `src/adapters/`
  - cflib link/transport、Lighthouse pose source、leader/follower executor、键盘手动输入
- `src/app/`
  - bootstrap、真机主循环、preflight、replay/visualization/comparison CLI

### 2. readiness / preflight

当前启动流程已经包含：

- `connect_all`
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
- safety 动作为 `EXECUTE / HOLD / ABORT`
- `HOLD` 持续超时后会自动降落（`hold_auto_land_timeout`）

### 5. Telemetry 与离线分析

真机运行会输出：

- `telemetry/run_real_YYYYMMDD_HHMMSS.jsonl`

当前 telemetry 已记录的内容包括：

- readiness / health / phase events
- `mission_error` 结构化错误事件，包含稳定的 `category / code / stage`
- snapshot 序号与测量时间
- measured positions / leader reference / follower reference
- frame validity / condition number
- safety action / reason codes
- scheduler diagnostics
- leader / follower action count
- follower command norms
- startup mode、manual axis、trajectory state、config fingerprint 等上下文

当前 `mission_error` 的错误定义已经按三类集中管理：

- `connection`：链路建立和设备连接阶段失败
- `readiness`：启动、readiness、preflight 与起飞验证阶段失败
- `runtime`：进入主循环之后的运行期异常

仓库内已提供的离线工具包括：

- replay summary
- offline reference visualization
- thesis-style trajectory comparison
- multi-run comparison

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
generate_docs.py
system_diagnosis.py
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

### `config/safety.yaml`

定义：

- boundary
- pose timeout
- max condition number
- max command norm
- estimator variance threshold
- battery threshold
- hold auto-land timeout（当前已在 `config/safety.yaml` 中显式配置）

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

