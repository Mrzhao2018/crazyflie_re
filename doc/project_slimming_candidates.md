# Project Slimming Candidates

This document is a review list, not a deletion list. It separates likely cleanup candidates from modules that look confusing but are still part of maintained behavior.

## Summary

The repository feels bulky for five main reasons:

1. `src/app/run_real.py` concentrates startup, real mission loop, safety handling, telemetry payloads, reconnect, landing, shutdown, and manual mode helpers in one large class.
2. `src/app/cli.py` exposes real flight, simulation, replay, visualization, comparison, probing, and web browsing through one command surface.
3. `src/runtime/` has several small bus/state modules with similar names. The split is reasonable, but the mental model is not obvious to a new reader.
4. `scripts/` mixes maintained experiment generators with local maintenance helpers.
5. `src/tests/` intentionally supports both historical script-style tests and normal pytest tests.

The safest cleanup order is:

1. Document the map and command surfaces.
2. Classify scripts and historical docs.
3. Consolidate repeated script helpers.
4. Split `run_real.py` in small behavior-preserving steps.
5. Only then consider renames or deletions.

## Candidate Classification

### Removed In Initial Cleanup

These items had no obvious runtime or test references during the cleanup scan and were removed from the source tree.

- `scripts/system_diagnosis.py`
  - Why removed: self-contained diagnostic helper with stale assumptions about fleet size and no obvious imports.

- `scripts/generate_docs.py`
  - Why removed: undocumented Sphinx config generator with no obvious project workflow references.

- `scripts/build_docs.sh`
  - Why removed: undocumented Sphinx wrapper with no obvious project workflow references.

- `plans/2026-04-20-startup-progress-ui.md`
  - Why removed: completed historical implementation plan; the corresponding design record remains in `specs/2026-04-20-startup-progress-ui-design.md`.

### Keep, But Explain Better

These areas look like duplication at first glance, but should not be deleted without a targeted refactor.

- `scripts/generate_*_ablation.py`
  - Why it looks bulky: several files have similar sweep/ablation shapes.
  - Why to keep: tests import several of them directly, including baseline, model-order, delay-compensation, trajectory-condition, full-state, and AFC solver lab paths.
  - Better cleanup: extract shared sweep helpers while preserving public functions used by tests.

- `src/runtime/follower_controller.py`, `src/runtime/follower_controller_v2.py`, and `src/runtime/follower_controller_base.py`
  - Why it looks bulky: v1/v2 naming is not self-explanatory.
  - Why to keep: `src/app/bootstrap.py` selects V2 for `full_state` and second-order dynamics.
  - Better cleanup: document controller selection in the project map or add a short module comment; do not merge blindly.

- `src/app/trajectory_comparison.py` and `src/app/trajectory_compare_runs.py`
  - Why it looks duplicated: names are very similar.
  - Why to keep: one compares a single run to an ideal trajectory, the other summarizes multiple runs.
  - Better cleanup: clarify CLI help text and docs before considering a rename.

- `src/runtime/health_bus.py`, `src/runtime/link_state_bus.py`, and `src/runtime/link_quality_bus.py`
  - Why it looks noisy: several bus modules sit next to each other.
  - Why to keep: they track different runtime concerns: drone health samples, connection state events, and radio quality metrics.
  - Better cleanup: document producer/consumer relationships instead of collapsing them.

### High Complexity, Not Deletion Candidates

- `src/app/run_real.py`
  - Issue: one large orchestration class owns too many phases.
  - Why not delete or rewrite: it is the real mission path and has the highest regression risk.
  - Better cleanup: split by phase with tests after every step.

- `src/app/bootstrap.py`
  - Issue: composition root handles core, real hardware, and Crazyswarm sim assembly.
  - Why not delete or split first: it is currently the best place to see the object graph.
  - Better cleanup: leave it intact until `run_real.py` is easier to navigate, then consider separating sim assembly.

- `src/app/cli.py`
  - Issue: many commands in one parser make the project feel larger than the core mission path.
  - Why not split first: it is still short enough to read, and command handlers are mostly thin.
  - Better cleanup: document command categories first, then consider subparser builder modules only if CLI keeps growing.

## Proposed `run_real.py` Refactor Outline

Do this only after the docs and cleanup list are reviewed. Each step should preserve behavior and keep public command usage unchanged.

### Step 1: Extract Telemetry Payload Helpers

Move pure payload-building helpers out of `RealMissionApp` into a small module such as `src/app/run_real_telemetry.py`.

Candidates:

- `_build_config_fingerprint`
- `_fleet_meta`
- `_measured_positions`
- `_leader_reference_positions`
- `_follower_reference_positions`
- `_trajectory_entry_start_positions`
- `_radio_link_quality_payload`

Why first: these helpers are comparatively pure and easier to test.

Verification:

- `python -m pytest src/tests/test_telemetry.py src/tests/test_telemetry_async.py src/tests/test_replay_radio_link_summary.py -q`
- `python -m pytest src/tests/test_run_real.py src/tests/test_run_real_follower_regressions.py -q`

### Step 2: Slim Landing And Shutdown Wrappers

`src/runtime/landing_flow.py` already owns most landing/shutdown orchestration through `LandingFlow`. The next cleanup should not duplicate that extraction. Instead, review the remaining `RealMissionApp` wrapper methods and decide whether they still improve call-site readability or can be replaced with direct `landing_flow` calls.

Candidates:

- `_graceful_shutdown_land`
- `_orderly_land`
- `_flush_terminal_telemetry`
- `_emergency_land`
- `_leader_land_action`

Why second: the heavy landing logic is already outside `run_real.py`, so this is a bounded wrapper-reduction pass rather than a risky behavior move.

Verification:

- `python -m pytest src/tests/test_run_real.py src/tests/test_real_mission_app_phase.py src/tests/test_run_real_watchdog.py -q`
- `python -m pytest src/tests/test_group_executor_pool.py src/tests/test_follower_executor_group_parallel.py src/tests/test_leader_executor_group_parallel.py -q`

### Step 3: Extract Startup Phase Helpers

Split startup/readiness orchestration into a module such as `src/app/run_real_startup.py`.

Candidates:

- `_start_impl`
- `_fail_start`
- `_last_connect_report`
- `_warmup_full_state_followers`
- `_initialize_manual_mode`

Why third: startup touches many hardware-adapter interactions and should be split only after the easier helpers are out.

Verification:

- `python -m pytest src/tests/test_startup_flow.py src/tests/test_startup_progress.py src/tests/test_preflight.py -q`
- `python -m pytest src/tests/test_run_real_reset_estimator_progress.py src/tests/test_config_loader_connect_fields.py -q`

### Step 4: Slim Failure Policy And Runtime Safety Glue

`src/runtime/failure_policy.py` already owns watchdog/degrade/hold/reconnect policy state through `FailurePolicy`. The remaining work is to reduce glue code in `RealMissionApp`, identify policy helpers that still live in the app class, and leave the main loop with clearer high-level calls.

Candidates:

- `_check_velocity_stream_watchdog`
- `_apply_watchdog_degrade`
- `_clear_watchdog_degrade`
- `_apply_follower_failure_policy`
- `_split_degraded_commands`
- `_enter_hold_mode`
- `_check_hold_timeout`
- `_clear_hold_tracking`
- `_record_link_state_events`
- `_try_reconnect_on_disconnect`

Why fourth: this is behavior-sensitive and already partially extracted, so the right next step is consolidation and wrapper pruning, not a second extraction of the same policy.

Verification:

- `python -m pytest src/tests/test_run_real_watchdog.py src/tests/test_safety_fast_gate.py src/tests/test_safety_fast_gate_group.py -q`
- `python -m pytest src/tests/test_link_state_bus.py src/tests/test_link_quality_bus.py src/tests/test_link_manager_reconnect.py -q`

### Step 5: Isolate One Mission Loop Step

After helpers are extracted, consider turning one iteration of `RealMissionApp.run()` into an explicit function or small object that receives snapshot, references, safety state, scheduler, and executors.

Why last: this is the highest-risk piece because it controls real-time behavior and command dispatch ordering.

Verification:

- `python -m pytest src/tests/test_run_real.py src/tests/test_run_real_follower_regressions.py src/tests/test_scheduler.py src/tests/test_scheduler_group_fast_path.py -q`
- `python -m pytest src/tests -m "not slow" -q`

## Deletion Gates

Before deleting or moving any file:

1. Search for imports, CLI references, README references, and tests.
2. If it is a script, run the directly affected script tests.
3. If it is a command path, run `python -m src.app.cli --help` and the command-specific help.
4. If behavior changes, update `README.md` and this document in the same change.
5. Prefer archiving over deletion when the only evidence is "no imports found" but manual lab usage is plausible.

## Recommended Next Actions

1. Review this list and mark each likely archive/delete candidate as `keep`, `archive`, or `delete`.
2. Archive or remove only the confirmed low-risk candidates.
3. Consolidate shared ablation script helpers without changing tested public functions.
4. Start the `run_real.py` split with telemetry payload helpers.
