"""Offline reference visualization smoke test."""

from pathlib import Path
import tempfile

from src.app.offline_reference_viz import generate_reference_visualizations


with tempfile.TemporaryDirectory() as tmp_dir:
    output_dir = Path(tmp_dir) / "viz"
    outputs = generate_reference_visualizations(
        config_dir="config",
        output_dir=output_dir,
        dt=10.0,
        fps=4,
        trail=4,
        total_time=20.0,
    )

    png_path = outputs.png_path
    gif_path = outputs.gif_path
    replay = outputs.replay

    assert png_path.exists()
    assert gif_path.exists()
    assert png_path.stat().st_size > 0
    assert gif_path.stat().st_size > 0
    assert replay.times[0] == 0.0
    assert replay.times[-1] == 20.0
    assert len(replay.drone_ids) == 10
    assert replay.roles[1] == "leader"
    assert replay.roles[4] == "leader"
    assert replay.roles[7] == "leader"
    assert replay.roles[8] == "leader"
    assert replay.roles[10] == "follower"

print("[OK] Offline reference visualization smoke path verified")
