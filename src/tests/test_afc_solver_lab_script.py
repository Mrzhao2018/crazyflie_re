"""AFC solver lab ablation matrix contracts."""

from scripts.generate_afc_solver_lab_ablation import build_afc_ablation_matrix


matrix = build_afc_ablation_matrix()
assert matrix["solver_modes"] == ["dense_current", "sparse_convex"]
assert "delay_compensation" in matrix["axes"]
assert "event_triggered_tx" in matrix["axes"]
assert "formation_error" in matrix["metrics"]
assert "radio_link_summary" in matrix["metrics"]
assert any(
    trial["solver_mode"] == "sparse_convex"
    and trial["event_triggered_tx"]["enabled"] is True
    for trial in matrix["trials"]
)

print("[OK] AFC solver lab ablation contracts verified")
