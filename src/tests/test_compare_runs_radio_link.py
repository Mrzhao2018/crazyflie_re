"""compare_run_summaries 扩展：radio_link_summary 需要被提取到每行汇总里。"""

import json
import tempfile
from pathlib import Path

from src.app.trajectory_compare_runs import compare_run_summaries


with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    run_a = root / "run_a"
    run_b = root / "run_b"
    run_a.mkdir()
    run_b.mkdir()

    summary_a = {
        "alignment_time_base": "mission_elapsed",
        "formation_run_summary": {
            "config_fingerprint": {"config_sha256": "sha-a"},
            "record_count": 100,
            "frame_valid_rate": 1.0,
            "formation_error": {"mean": 0.1, "rmse": 0.1, "p95": 0.1, "max": 0.1},
            "role_tracking_error": {
                "leader": {"rmse": 0.1, "p95": 0.1},
                "follower": {"rmse": 0.1, "p95": 0.1},
            },
            "watchdog_summary": {"total": 0, "by_mode": {}},
            "executor_failure_summary": {
                "total": 0,
                "by_action": {},
                "by_group": {},
                "retryable_counts": {},
            },
            "radio_link_summary": {
                "overall": {"count": 400, "min": 52.0, "mean": 85.5, "p5": 60.0, "max": 99.0},
                "per_drone": {
                    "1": {"count": 100, "min": 70.0, "mean": 92.0, "p5": 75.0, "max": 99.0},
                    "2": {"count": 100, "min": 52.0, "mean": 78.0, "p5": 55.0, "max": 95.0},
                },
            },
        },
    }
    summary_b = {
        "alignment_time_base": "mission_elapsed",
        "formation_run_summary": {
            "config_fingerprint": {"config_sha256": "sha-b"},
            "record_count": 100,
            "frame_valid_rate": 1.0,
            "formation_error": {"mean": 0.1, "rmse": 0.1, "p95": 0.1, "max": 0.1},
            "role_tracking_error": {
                "leader": {"rmse": 0.1, "p95": 0.1},
                "follower": {"rmse": 0.1, "p95": 0.1},
            },
            "watchdog_summary": {"total": 0, "by_mode": {}},
            "executor_failure_summary": {
                "total": 0,
                "by_action": {},
                "by_group": {},
                "retryable_counts": {},
            },
            # 某些旧 run 不含 radio_link_summary —— 不应让整条管道崩
        },
    }

    (run_a / "trajectory_comparison_summary.json").write_text(
        json.dumps(summary_a, ensure_ascii=False), encoding="utf-8"
    )
    (run_b / "trajectory_comparison_summary.json").write_text(
        json.dumps(summary_b, ensure_ascii=False), encoding="utf-8"
    )

    result = compare_run_summaries([str(run_a), str(run_b)])
    row_a = result["runs"][0]
    row_b = result["runs"][1]

    assert row_a["radio_link_overall_min"] == 52.0
    assert row_a["radio_link_overall_mean"] == 85.5
    assert row_a["radio_link_overall_p5"] == 60.0
    assert row_a["radio_link_sample_count"] == 400
    # drone 级最低：min_per_drone 取所有 drone 最小的 min
    assert row_a["radio_link_worst_drone_min"] == 52.0

    # 缺字段的 run 应安全降级到 None/0
    assert row_b["radio_link_overall_min"] is None
    assert row_b["radio_link_sample_count"] == 0

print("[OK] compare_run_summaries.radio_link extraction verified")
