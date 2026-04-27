"""Landing/shutdown orchestration helpers.

Wraps the ``_orderly_land`` / ``_emergency_land`` / ``_graceful_shutdown_land``
/ ``_flush_terminal_telemetry`` choreography so the RealMissionApp does not
carry all of this orchestration inline.
"""

from __future__ import annotations

import logging
import time

from ..app.mission_errors import MissionErrorDefinition
from .command_plan import HoldAction
from .mission_fsm import MissionState
from .telemetry import TelemetryRecord


logger = logging.getLogger(__name__)


class LandingFlow:
    """Terminal-state landing + telemetry flush helpers for RealMissionApp."""

    def __init__(self, app):
        self.app = app

    def _active_follower_ids(self) -> list[int]:
        active_fn = getattr(self.app, "_active_follower_ids", None)
        if callable(active_fn):
            return list(active_fn())
        return list(self.app.fleet.follower_ids())

    def _notify_streaming_setpoint_stop(self, *, reason: str) -> dict:
        """Stop low-level streaming before returning control to high-level commander."""
        app = self.app
        follower_ids = self._active_follower_ids()
        app.telemetry.record_event(
            "commander_mode_switch",
            code="COMMANDER_MODE_SWITCH",
            from_mode="streaming_setpoint",
            to_mode="high_level_commander",
            reason=reason,
            drone_ids=follower_ids,
        )
        try:
            stop_result = app.comp["follower_executor"].stop_velocity_mode(follower_ids)
        except Exception as exc:
            logger.exception("Failed to notify setpoint stop before high-level command")
            stop_result = {
                "kind": "notify_stop",
                "successes": [],
                "failures": [
                    {
                        "drone_ids": follower_ids,
                        "failure_category": "notify_setpoint_stop",
                        "reason": str(exc),
                    }
                ],
            }
        app.telemetry.record_event(
            "setpoint_stop_notify",
            code="SETPOINT_STOP_NOTIFY",
            ok=not bool(stop_result.get("failures")),
            reason=reason,
            drone_ids=follower_ids,
            result=stop_result,
        )
        app.telemetry_reporter.record_executor_summary(
            "follower_stop_velocity_execution", [stop_result]
        )
        return stop_result

    def _brake_streaming_followers(
        self, *, reason: str, repeats: int = 3, interval_s: float = 0.05
    ) -> None:
        """Send a short zero-velocity burst before handing followers to HLC land."""
        app = self.app
        follower_ids = self._active_follower_ids()
        if not follower_ids:
            return

        actions = [HoldAction(drone_id=drone_id) for drone_id in follower_ids]
        for attempt in range(max(1, int(repeats))):
            try:
                result = app.comp["follower_executor"].execute_hold(actions)
                app.telemetry_reporter.record_executor_summary(
                    "follower_brake_execution", [result]
                )
                app.telemetry.record_event(
                    "follower_brake",
                    code="FOLLOWER_BRAKE",
                    ok=not bool(result.get("failures")),
                    reason=reason,
                    attempt=attempt + 1,
                    drone_ids=follower_ids,
                    result=result,
                )
            except Exception as exc:
                logger.exception("Failed to brake followers before landing")
                app.telemetry.record_event(
                    "follower_brake",
                    code="FOLLOWER_BRAKE",
                    ok=False,
                    reason=reason,
                    attempt=attempt + 1,
                    drone_ids=follower_ids,
                    error=str(exc),
                )
            time.sleep(max(0.0, float(interval_s)))

    # ---- graceful shutdown / orderly land --------------------------------

    def graceful_shutdown_land(self) -> None:
        app = self.app
        state = app.fsm.state()
        if state in {MissionState.INIT, MissionState.CONNECT, MissionState.ABORT}:
            return

        self.orderly_land(
            reason_event="manual_shutdown_land",
            safety_action="SHUTDOWN",
            safety_reasons=["manual_shutdown"],
            safety_reason_codes=["MANUAL_SHUTDOWN"],
            scheduler_reason="shutdown",
            scheduler_diagnostics={"shutdown": True},
            trajectory_terminal_reason="shutdown",
        )

    def orderly_land(
        self,
        *,
        reason_event: str,
        safety_action: str,
        safety_reasons: list[str],
        safety_reason_codes: list[str],
        scheduler_reason: str,
        scheduler_diagnostics: dict,
        trajectory_terminal_reason: str,
    ) -> None:
        app = self.app
        if app._terminal_land_executed:
            app._set_trajectory_state("terminated", trajectory_terminal_reason)
            self.flush_terminal_telemetry(
                safety_action=safety_action,
                safety_reasons=safety_reasons,
                safety_reason_codes=safety_reason_codes,
                scheduler_reason=scheduler_reason,
                scheduler_diagnostics=scheduler_diagnostics,
            )
            return

        try:
            if app.fsm.state() != MissionState.LAND:
                app._safe_transition(MissionState.LAND)
        except Exception:
            logger.exception("Failed to enter LAND during shutdown")

        try:
            app.telemetry.record_event(reason_event, ok=True)
            self._brake_streaming_followers(reason=reason_event)
            self._notify_streaming_setpoint_stop(reason=reason_event)
            time.sleep(0.2)
            leader_land_results = app.comp["leader_executor"].execute(
                [app._leader_land_action(app.fleet.leader_ids())]
            )
            app.telemetry_reporter.record_executor_summary(
                "leader_land_execution", leader_land_results
            )
            follower_land_result = app.comp["follower_executor"].land(
                self._active_follower_ids(), duration=4.0
            )
            app.telemetry_reporter.record_executor_summary(
                "follower_land_execution", [follower_land_result]
            )
            app._terminal_land_executed = True
            app._set_trajectory_state("terminated", trajectory_terminal_reason)
            time.sleep(3.0)
        except Exception:
            logger.exception("Failed graceful land during shutdown")
            app._set_trajectory_state("failed", trajectory_terminal_reason)

        self.flush_terminal_telemetry(
            safety_action=safety_action,
            safety_reasons=safety_reasons,
            safety_reason_codes=safety_reason_codes,
            scheduler_reason=scheduler_reason,
            scheduler_diagnostics=scheduler_diagnostics,
        )

    # ---- emergency land ---------------------------------------------------

    def emergency_land(
        self,
        *,
        trigger_error: MissionErrorDefinition | None = None,
    ) -> None:
        """紧急降落"""
        app = self.app
        logger.error("=== 紧急降落 ===")
        event_details: dict[str, object] = {"ok": True}
        if trigger_error is not None:
            event_details["trigger_code"] = trigger_error.code
            event_details["trigger_category"] = trigger_error.category
            event_details["trigger_stage"] = trigger_error.stage
        app.telemetry.record_event("emergency_land", **event_details)
        app.fsm.force_abort()
        app._set_trajectory_state("terminated", "abort")

        self._brake_streaming_followers(reason="emergency_land")
        self._notify_streaming_setpoint_stop(reason="emergency_land")
        time.sleep(0.2)

        try:
            app.comp["leader_executor"].execute(
                [app._leader_land_action(app.fleet.leader_ids())]
            )
            app.comp["follower_executor"].land(
                self._active_follower_ids(), duration=4.0
            )
            app._terminal_land_executed = True
        except Exception as exc:
            logger.error("Failed to land swarm: %s", exc)

        self.flush_terminal_telemetry(
            safety_action="ABORT",
            safety_reasons=["emergency_land"],
            safety_reason_codes=["EMERGENCY_LAND"],
            scheduler_reason="emergency_land",
            scheduler_diagnostics={"emergency_land": True},
        )

        time.sleep(3.0)

    # ---- terminal telemetry flush ----------------------------------------

    def flush_terminal_telemetry(
        self,
        *,
        safety_action: str,
        safety_reasons: list[str],
        safety_reason_codes: list[str],
        scheduler_reason: str,
        scheduler_diagnostics: dict,
    ) -> None:
        app = self.app
        if app._shutdown_flushed or app.telemetry is None:
            return
        telemetry = app.telemetry
        pose_bus = app.comp.get("pose_bus")
        if pose_bus is None:
            return
        snapshot = pose_bus.latest()
        if snapshot is None:
            return

        t_elapsed = 0.0
        measured_positions = app._measured_positions(snapshot)
        leader_ref = app.comp["leader_ref_gen"].reference_at(t_elapsed)
        fleet = app.fleet
        telemetry.log(
            TelemetryRecord(
                t_wall=time.time(),
                mission_state=app.fsm.state().value,
                startup_mode=app.comp.get("startup_mode"),
                mission_elapsed=t_elapsed,
                trajectory_state=app._trajectory_state,
                trajectory_terminal_reason=app._trajectory_terminal_reason,
                snapshot_seq=snapshot.seq,
                snapshot_t_meas=snapshot.t_meas,
                measured_positions=measured_positions,
                fresh_mask={
                    drone_id: bool(
                        snapshot.fresh_mask[fleet.id_to_index(drone_id)]
                    )
                    for drone_id in fleet.all_ids()
                },
                disconnected_ids=list(snapshot.disconnected_ids),
                health={
                    drone_id: sample.values
                    for drone_id, sample in app.comp["health_bus"].latest().items()
                },
                frame_valid=None,
                frame_condition_number=None,
                phase_label=app._phase_label(t_elapsed),
                leader_mode=getattr(leader_ref, "mode", None),
                leader_reference_positions=app._leader_reference_positions(leader_ref),
                follower_reference_positions={},
                safety_action=safety_action,
                safety_reasons=safety_reasons,
                safety_reason_codes=safety_reason_codes,
                scheduler_reason=scheduler_reason,
                scheduler_diagnostics=scheduler_diagnostics,
                leader_reference_source=(
                    type(app.comp["leader_ref_gen"]).__name__
                    if "leader_ref_gen" in app.comp
                    else None
                ),
                manual_axis=app._manual_axis(),
                manual_input_age=app._manual_input_age(),
                leader_action_count=0,
                follower_action_count=0,
                follower_command_norms={},
            )
        )
        app._shutdown_flushed = True
