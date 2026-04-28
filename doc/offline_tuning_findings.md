# 离线调参结论（Phase 1 / Phase 2）

> 2026-04-19 记录。本文汇总一阶与二阶基线的离线扫参结论，以及三条 ablation 的定量结果。
>
> 所有数据来自 `artifacts/` 下的 json 文件，扫描脚本集中在 `scripts/`。

---

## 快速导航

| 阶段       | 关注点                                                                                | 对应产物                                            |
| ---------- | ------------------------------------------------------------------------------------- | --------------------------------------------------- |
| Phase 1A   | 一阶 follower controller 的 `gain_xy / gain_z / max_velocity` baseline                  | `artifacts/baseline_results.json`                   |
| Phase 1B   | reference 层一阶纯时滞补偿 on/off ablation                                             | `artifacts/delay_compensation_ablation.json`        |
| Phase 1C   | Leader trajectory 条件数惩罚 on/off（含 `condition_stress_*` 压测轨迹）                | `artifacts/trajectory_condition_ablation.json`      |
| Phase 2A   | 二阶 follower 内部动力学的 baseline 参数                                                | `artifacts/second_order_baseline_results.json`      |
| Phase 2B   | 一阶 vs 二阶 tuned-vs-tuned 公平对比                                                    | `artifacts/model_order_ablation_tuned.json`         |

---

## Phase 1 结论

### Phase 1A Baseline

当前一阶闭环离线基线的最佳参数为：

| 参数            | 值      |
| --------------- | ------- |
| `gain_xy`       | `1.4`   |
| `gain_z`        | `0.6`   |
| `max_velocity`  | `0.55`  |

对应基线指标：

- `formation_rmse ≈ 0.00463`
- `follower_rmse ≈ 0.00801`
- `formation_p95 ≈ 0.01000`
- `frame_valid_rate = 1.0`

> 注意：`config/fleet.yaml` 默认 control 参数**不等于**这套最佳基线，只记录在 Phase findings 里，需要显式应用。

扫参脚本：

```bash
python scripts/generate_baseline_sweep.py \
  --grid p-limit \
  --dt 0.25 \
  --formation-rmse-threshold 0.05 \
  --output artifacts/baseline_results.json
```

支持 `--grid quick` 与 `--limit-trials N` 做快速试跑。

### Phase 1B Delay Compensation

在上述 Baseline 参数集上开启 reference 层的一阶纯时滞补偿：

| 指标              | off      | on       | delta     |
| ----------------- | -------- | -------- | --------- |
| `formation_rmse`  | `0.00463` | `0.00422` | `-0.00041` |
| `follower_rmse`   | `0.00801` | `0.00734` | `-0.00067` |
| `frame_valid_rate`| `1.0`    | `1.0`    | `0`       |

结论：当前 delay compensation 在第一版 Baseline 上带来稳定正收益，可视为后续实机验证候选能力。

相关 `fleet.yaml` 字段（默认关闭）：

- `time_delay_compensation_enabled`
- `estimated_total_delay_ms`
- `delay_prediction_gain`

实现位置：`FollowerReferenceGenerator`；逻辑是基于历史 target velocity 做一阶线性预测，不改变现有一阶 follower controller 结构。

扫参脚本：

```bash
python scripts/generate_delay_compensation_ablation.py \
  --output artifacts/delay_compensation_ablation.json
```

### Phase 1C Leader Trajectory Condition Penalty

**默认任务（`affine_rotation`）下：**

- Leader 几何条件数本来就稳定在约 `3.04`，`cond_penalty_on` 不会触发，也不会改变 RMSE。

**为了验证第三条线是否真实生效，额外引入了 `condition_stress_*` 压测轨迹参数：**

- `condition_stress_enabled`
- `condition_stress_axis`
- `condition_stress_min_scale`
- `condition_stress_period`

**Stress trajectory 下：**

- `raw_condition_number_max` 从默认良态抬高到 `≈ 4.09`
- `cond_penalty_on` 可将 `condition_number_max` 压回到 `≈ 2.34`
- `penalized_samples = 9`
- 当前离线 RMSE 仍未改善

结论：第三条线能在**坏轨迹**上真实降低几何条件数，但当前离线一阶模型下，这种几何修正尚未传导为 RMSE 收益。更像**几何鲁棒性保护**，而不是主要性能增益来源。

相关 `mission.yaml` 字段（默认关闭）：

- `condition_penalty_enabled`
- `condition_soft_limit`
- `condition_penalty_scale`

扫参脚本：

```bash
python scripts/generate_trajectory_condition_ablation.py \
  --output artifacts/trajectory_condition_ablation.json
```

---

## Phase 2 结论

### Phase 2A Second-order Baseline

当前二阶离线基线的最佳参数为：

| 参数                             | 值      |
| -------------------------------- | ------- |
| `gain_xy`                        | `1.4`   |
| `gain_z`                         | `0.5`   |
| `max_velocity`                   | `0.65`  |
| `velocity_feedback_gain`         | `1.2`   |
| `acceleration_feedforward_gain`  | `0.5`   |
| `damping_coeff`                  | `0.05`  |

对应基线指标：

- `formation_rmse ≈ 0.00347`
- `follower_rmse ≈ 0.00603`
- `formation_p95 ≈ 0.00848`
- `frame_valid_rate = 1.0`

扫参脚本：

```bash
python scripts/generate_second_order_baseline_sweep.py \
  --grid quick \
  --output artifacts/second_order_baseline_results.json
```

支持 `--grid quick`、`--limit-trials N`。长时运行时会打印 trial 进度条、当前 rmse、best rmse、elapsed 和 eta。

### Phase 2B Tuned-vs-Tuned Model-order Ablation

使用一阶最优基线与二阶最优基线分别配置：

| 阶次   | `formation_rmse` | `follower_rmse` | `frame_valid_rate` |
| ------ | ---------------- | --------------- | ------------------ |
| 一阶   | `≈ 0.00470`      | `≈ 0.00808`     | `1.0`              |
| 二阶   | `≈ 0.00347`      | `≈ 0.00603`     | `1.0`              |
| delta  | `-0.00123`       | `-0.00205`      | `0.0`              |

结论：在当前 `dt = 5.0`、`total_time = 10.0` 的离线窗口内，二阶 internal-dynamics follower path 带来稳定跟踪收益，未牺牲有效帧率。

扫参脚本：

```bash
python scripts/generate_model_order_ablation.py \
  --baseline artifacts/baseline_results.json \
  --second-order artifacts/second_order_baseline_results.json \
  --output artifacts/model_order_ablation.json
```

结果中会额外写出 `applied_control`，明确每个 trial 实际应用的控制参数。

### Phase 2C Engineering Note

- `scripts/generate_model_order_ablation.py` 当前会优先保留 `second_order_baseline_results.json` 中的 `velocity_feedback_gain` 与 `acceleration_feedforward_gain`，再补默认项。
- 这是为了确保 tuned-vs-tuned 公平对比真正使用二阶最优参数，不被脚本默认值回写覆盖。
- 当前证据仍来自短窗口 smoke-style 配置；是否在更长时域、更激进 leader 轨迹下继续保持优势，建议后续再做 `30s ~ 40s` 验证。

---

## 工程结论

综合 Phase 1 与 Phase 2 的当前证据：

1. **Baseline 参数集可直接作为第一版一阶基线**。
2. **Delay compensation 值得继续做实机验证**——离线上是稳定正收益，但需要在真机链路抖动存在的条件下验证是否仍为正。
3. **Condition penalty 应保留为轨迹质量保护机制**，但不建议仅凭当前默认任务将其作为主收益项宣传。
4. **二阶 follower 在短窗口上有稳定收益**，但长窗口实机验证尚未完成；参数扫参脚本已具备。
5. 下一步建议集中在：**最优参数与最优工况的固化、各项改动的消融分析、不同机动强度/场地条件下的边界验证、将已有经验整理成可复现的实验结论**。

---

## 相关脚本速查

| 脚本                                              | 作用                                                            |
| ------------------------------------------------- | --------------------------------------------------------------- |
| `scripts/generate_baseline_sweep.py`              | Phase 1A 一阶基线扫参                                            |
| `scripts/generate_delay_compensation_ablation.py` | Phase 1B delay compensation on/off 对比                          |
| `scripts/generate_trajectory_condition_ablation.py` | Phase 1C leader trajectory 条件数惩罚 on/off 对比             |
| `scripts/generate_second_order_baseline_sweep.py` | Phase 2A 二阶基线扫参                                            |
| `scripts/generate_model_order_ablation.py`        | Phase 2B 一阶 vs 二阶 tuned-vs-tuned 对比                        |

---

## 边界

- 所有扫参结果基于短窗口（`dt = 0.25`、`total_time = 10.0` 量级）的离线 smoke 配置，未在真机上做对应验证。
- `model_order_ablation` 在参数读取顺序上对二阶字段有特殊优先级，跨脚本组合使用时需留意。
- Condition penalty 收益只能在坏轨迹上显现；默认良态轨迹下无行为差异，这是预期而非 bug。
