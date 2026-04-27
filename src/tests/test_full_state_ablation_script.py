"""Full-state/Mellinger ablation matrix contracts."""

from scripts.generate_full_state_ablation import build_experiment_matrix


matrix = build_experiment_matrix(repeats=3)
profiles = {item["profile"] for item in matrix["experiments"]}

assert matrix["repeat_count"] == 3
assert profiles == {"velocity_pid_safe", "full_state_mellinger"}

velocity = next(
    item for item in matrix["experiments"] if item["profile"] == "velocity_pid_safe"
)
full_state = next(
    item for item in matrix["experiments"] if item["profile"] == "full_state_mellinger"
)

assert velocity["control_overrides"]["output_mode"] == "velocity"
assert velocity["control_overrides"]["onboard_controller"] == "pid"
assert full_state["control_overrides"]["output_mode"] == "full_state"
assert full_state["control_overrides"]["onboard_controller"] == "mellinger"
assert full_state["control_overrides"]["dynamics_model_order"] == 2
assert "radio_link_summary" in matrix["metrics"]
assert "safety_counts" in matrix["metrics"]

print("[OK] full-state ablation matrix contracts verified")
