# 启动进度可视化设计（rich TUI + 自动降级）

> 2026-04-20 · 针对 `python -m src.app.cli run` 真机主流程

## 背景与问题

用户反馈：真机入口按下 Enter 后，终端长时间一片黑，看不出系统是否在正常启动。

实际根因（见 `src/app/cli.py:_run_real` 与 `src/app/run_real.py:RealMissionApp.start`）：

1. 仓库中无任何 `logging.basicConfig` / `dictConfig` / root logger 配置，Python 默认 root level = WARNING 且无 handler，因此 `start()` 中所有 `logger.info(...)` 均不显示。
2. `_run_real` 仅在按 Enter 前 `print` 4 行常量字符串，之后进入 `app.start()` 到 `app.run()` 期间完全沉默。
3. `connect_all` 的 `on_group_start / on_group_result` 回调只写入 telemetry JSONL，不回显终端。10 机分组连接、`wait_for_params`、`reset_estimator`、`trajectory_upload` 每步均可能 sleep 数秒。

本设计引入一个启动阶段进度 UI，默认使用 rich TUI，在非 TTY / rich 未安装 / 显式 opt-out 时自动降级到普通滚动日志；并通过一个统一的 `configure_logging` 入口修复 logger 初始化问题。

## 目标

- 真机 `run` 主流程在 `start()` 的每个启动阶段均有可见反馈：阶段名、进度（n/total）、用时、结果（OK / WARN / FAIL）。
- 支持两档粒度：默认阶段级（每阶段 1 行 / 1 Live 行），`--verbose` 切到每机级。
- rich 为可选依赖；未安装 / 非 TTY / `AFC_NO_RICH=1` 时自动降级到滚动文本，不报错、不影响启动。
- 启动阶段事件 mirror 到现有 telemetry（新增 `startup_phase` 事件），复盘与 web 端将来消费免费获得。
- 不改变现有 telemetry schema（不删除、不重命名任何字段）；不改控制流；不改 preflight / bootstrap / 其他子命令。

## 非目标

- 不覆盖 `run()` 主循环（进入主循环前面板关闭，主循环仍按现有 telemetry 行为运行）。
- 不做 rich 视觉回归测试、不做真机端 e2e 测试。
- 不向 `src/web/` 扩展本设计的 UI 层。未来 web 端实时启动进度展示基于 telemetry 的 `startup_phase` 事件自行实现。
- 不重做 `cli.py` 其他子命令（`budget / replay / viz / compare / compare-runs / sim / web`）。

## 架构

### 新增文件

- `src/app/startup_progress.py` — 进度 reporter 抽象与两种实现
- `src/app/log_setup.py` — 单一 logging 初始化入口

### 修改文件

- `src/app/cli.py` — `_run_real` 先建 reporter，再 `configure_logging(verbose, reporter)`，把 reporter 传入 `RealMissionApp`
- `src/app/run_real.py` — `RealMissionApp.__init__` 接收可选 reporter；`start()` 用 `with progress.phase(...)` 包住每一段；阶段 mirror 一条 `startup_phase` telemetry 事件

### 不改文件

- `src/app/bootstrap.py`、`src/app/preflight.py`、`src/runtime/*`、`src/adapters/*`、`src/web/*`、所有 `config/*.yaml`
- `src/runtime/telemetry.py` 的 schema（仅新增一种 event name，走现有 `record_event` 路径）

### 组件

```
cli._run_real
  │
  ├── make_reporter(verbose)         ── src/app/startup_progress.py
  │     ├── RichProgressReporter      （Live + Table + Progress）
  │     └── TextProgressReporter      （logger.info 滚动文本）
  │
  ├── configure_logging(verbose, reporter)  ── src/app/log_setup.py
  │     ├── Rich 分支 → RichHandler(console=reporter.console)
  │     └── Text 分支 → StreamHandler(sys.stdout)
  │
  └── RealMissionApp(components, progress=reporter)
        └── start() 使用 with self._phase(key, title): ...
              └── _phase helper 同时驱动 reporter 与 telemetry.record_event("startup_phase", ...)
```

### Reporter 接口（`StartupProgressReporter` 协议）

```python
class StartupProgressReporter(Protocol):
    @contextmanager
    def phase(self, key: str, title: str, total: int | None = None): ...
    def step(self, done: int, total: int, detail: str | None = None) -> None: ...
    def warn(self, msg: str) -> None: ...
    # phase context 内部：正常退出调用自身 ok(duration)；
    # 异常退出调用自身 fail(reason) 后让异常冒泡。
    # 外部代码只负责 phase + step + warn，不直接调 ok/fail。

    @property
    def console(self) -> Any: ...   # rich.console.Console 或 None
    def close(self) -> None: ...
```

### 职责边界：reporter vs telemetry

reporter 只负责**显示**，不调 telemetry。telemetry 事件由 `RealMissionApp` 的 private helper 统一发出：

```python
# RealMissionApp 中新增
@contextmanager
def _phase(self, key: str, title: str, total: int | None = None):
    self.telemetry.record_event("startup_phase", phase=key, status="begin")
    start = time.monotonic()
    try:
        with self._progress.phase(key, title, total) as ctx:
            yield ctx
    except Exception as exc:
        self.telemetry.record_event("startup_phase", phase=key, status="fail",
                                     duration_s=time.monotonic() - start,
                                     detail=str(exc))
        raise
    else:
        self.telemetry.record_event("startup_phase", phase=key, status="ok",
                                     duration_s=time.monotonic() - start)
```

`start()` 用 `with self._phase(...)` 包每段，职责清晰：UI 走 reporter、事件走 telemetry、一次包裹双得。

## 降级决策

`make_reporter(verbose)` 按以下顺序决定实现类：

1. `os.getenv("AFC_NO_RICH") == "1"` → `TextProgressReporter`
2. `try: import rich` 失败 → `TextProgressReporter`，首行打印提示 "rich 未安装，降级滚动文本（可 `pip install rich` 启用 TUI）"
3. `sys.stdout.isatty()` 为 False（重定向、CI、`> log.txt`）→ `TextProgressReporter`
4. 否则 → `RichProgressReporter`

`configure_logging(verbose, reporter)`：

- Rich 分支：`RichHandler(console=reporter.console, rich_tracebacks=True, show_path=False)`
- Text 分支：`StreamHandler(sys.stdout)` + 格式 `[%(asctime)s] %(message)s`（`%H:%M:%S`）
- 都将 root logger level 设为 `INFO`，`verbose=True` 时设为 `DEBUG`

两个分支共享同一 console，保证任意 `logger.info(...)` 调用不会撕裂 Live 面板。

## 启动阶段定义

与 `RealMissionApp.start()` 实际流程一对一：

| # | key                   | 来源（run_real.py）      | total                          | verbose 额外信息                       |
|---|-----------------------|--------------------------|--------------------------------|----------------------------------------|
| 1 | `connect`             | `connect_all` L171-195   | radio_group 数                 | 每组 start/result 一次 step            |
| 2 | `wait_for_params`     | L197-227（条件）         | drone 数                       | 每台 drone done 一次 step              |
| 3 | `reset_estimator`     | L229-244（条件）         | drone 数                       | 每台 drone done 一次 step              |
| 4 | `onboard_controller`  | L246-302                 | drone 数                       | 每台 drone done 一次 step              |
| 5 | `pose_source`         | L305-339                 | drone 数                       | fresh_mask 覆盖数                      |
| 6 | `health_ready`        | L341-358                 | drone 数                       | 已上报 drone 数                        |
| 7 | `trajectory_upload`   | L360-456（auto+轨迹启用）| leader 数                      | per-leader piece count / bytes         |
| 8 | `preflight`           | L464-488                 | None                           | 失败时列 `failed_codes`                |
| 9 | `takeoff_settle_align`| L491-598                 | None                           | 起飞→settle→align 三小步合并显示状态    |

条件阶段（2、3、7）根据 config 决定是否出现。出现时计入总阶段数（面板头部标 `N/N`，N 为本次实际启用的阶段数，范围 6-9）；不出现时不计入。示例中的 `[1/9]` 等编号表示"本次运行启用全部 9 个阶段"。

## 阶段 telemetry 事件

每个阶段开始/结束由 `RealMissionApp._phase` helper 调用：

```python
telemetry.record_event("startup_phase",
    phase=key,
    status="begin" | "ok" | "warn" | "fail",
    duration_s=<float, ok/fail 时有>,
    detail=<str 可选>,
)
```

- 复用现有 `record_event` 路径，零 schema 改动（事件名是字符串字段）
- web 端（`src/web/data_access/`）若要消费，等价于现有其他 event 的读取方式
- reporter 不直接 import telemetry；mission 代码通过 `_phase` helper 同时驱动两者，职责清晰

## 错误处理

三种异常路径必须干净交回终端：

- **阶段失败**：`run_real._fail_start` 已有 `logger.error + record_error_event`。`with phase(...)` 捕获异常后：标 FAIL、关闭 Live（`finally: progress.close()`）、异常继续上抛。关键：**Live 必须在异常冒泡到 traceback 打印前停止**，否则 traceback 会被 Live 面板遮挡。
- **KeyboardInterrupt**：`cli._run_real` 的 `try/finally` 已有处理；reporter 的 `close()` 必须幂等。
- **preflight 失败**：`preflight` 阶段的异常已由 `_fail_start` 记录到 telemetry。reporter 在面板 FAIL 行附加 `failed_codes`（最多列 3 条，超出显示 `(+N more)`）。

## verbose 语义

- 默认：阶段级；Rich 显示 9 行面板 + 当前 active 阶段的子进度条；Text 每阶段 2 行（begin / ok）
- `--verbose` / `-v`：每机级；Rich 下 active 阶段展开子表列每台 drone 状态；Text 下每台 drone 一行
- logging level：默认 `INFO`，verbose 下 `DEBUG`

`AFC_NO_RICH=1` 只强制 Text reporter，不改变 verbose 语义。

## Text 降级输出示例

```
[14:22:31] 构建系统...
[14:22:33] 系统构建完成
[14:22:35] 按Enter启动（需要真机连接）
[14:22:42] [1/9] 连接 Crazyflie ...
[14:22:42]        group=0 drones=[1,2,3,4] 开始
[14:22:45]        group=0 drones=[1,2,3,4] OK (3.1s)
[14:22:45]        group=1 drones=[5,6,7] 开始
[14:22:50] [1/9] 连接 Crazyflie OK (8.3s)
[14:22:50] [2/9] 等待参数同步 ...
[14:22:55] [2/9] 等待参数同步 OK (4.7s, 10/10)
...
```

## Rich 默认面板示例

```
Crazyflie AFC Swarm · 启动 (mode=auto, drones=10, leaders=[1,4,7,8])
  [1/9] 连接 Crazyflie           OK  (8.3s)
  [2/9] 等待参数同步             OK  (4.7s, 10/10)
  [3/9] 重置估计器               OK  (2.1s, 10/10)
  [4/9] 设置 onboard controller  ████████░░ 8/10  (1.9s)
  [5/9] 定位就绪                 ...
  [6/9] 健康数据就绪             ...
  [7/9] 轨迹上传                 ...
  [8/9] preflight                ...
  [9/9] takeoff / settle / align ...
```

阶段失败该行变红 + `✗ reason`；阶段 warn（如 `onboard_controller` fallback）该行变黄 + 提示。

## CLI 与环境变量

- `python -m src.app.cli run`：Rich 默认（如可用）
- `python -m src.app.cli run --verbose`：每机级
- `AFC_NO_RICH=1 python -m src.app.cli run`：强制 Text
- `python -m src.app.cli run > startup.log 2>&1`：自动降级 Text（非 TTY）

新增的 `--verbose` / `-v` 加到 `_build_run_parent`；兼容旧入口 `main.py`。

## 测试策略

### 单元测试

- **`src/tests/test_startup_progress_reporter.py`**
  - `TextProgressReporter` 捕获 logger 输出，断言 9 阶段按序、`step(done, total)` 格式、`fail` 路径附带 reason
  - `RichProgressReporter` 注入 `rich.console.Console(file=StringIO(), force_terminal=True)`，断言输出含阶段 key 与 OK/✗（不做像素级比对）
  - `make_reporter` 4 条降级分支各一条 case（monkey-patch `sys.stdout.isatty` 与 `AFC_NO_RICH`，另一条 `try: import rich` 失败用 `sys.modules` mock）
- **`src/tests/test_startup_progress_integration.py`**
  - 在 `run_real.py` 提取 `_with_phase(progress, key, title, fn)` helper 后，针对 helper 测 phase→ok、phase→exception、重复 close 幂等
- **`src/tests/test_log_setup.py`**
  - 断言 verbose 切换 level
  - 断言 Rich 分支下 RichHandler 与 reporter 共享同一 console 实例

### 契约测试不动

现有 36 条 contract tests 基于 `build_core_app`，不经过 `cli.py`，与本设计无交集。零改动。

### 手动验证

接真机前：

- `AFC_NO_RICH=1 python -m src.app.cli run > startup.log 2>&1` 对比 Rich / Text 输出
- 不接真机直接跑 → `connect_all` 应报错，reporter 应把第 1 阶段标 FAIL、Live 关闭、traceback 正常打印

## 依赖

- `rich >= 13`（可选）。未安装时自动降级，不触发 ImportError。
- README 的"环境说明 → 可选"小节追加一行：`rich`（启动 TUI，缺省时自动降级滚动文本）。

## 回滚策略

- 所有改动集中在 `src/app/cli.py`、`src/app/run_real.py` 与两个新增文件。
- 回滚只需删除两个新增文件、还原 `_run_real` 的 4 行 print 与 `RealMissionApp.start` 的 `with` 包裹。
- telemetry 的 `startup_phase` 事件若下游已消费，回滚前需要 web 端先撤消费代码；web 端暂未消费，因此无后向兼容负担。

## 开放问题

- 无（所有澄清问题已在 brainstorm 阶段回答：入口=真机 run、粒度=两档可切、依赖=rich 可选+自动降级、生命周期=仅启动阶段、opt-out=`AFC_NO_RICH`）。
