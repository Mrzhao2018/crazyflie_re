"""Mission telemetry event helpers.

Encapsulates the structured-event builders that used to live on
``RealMissionApp``. ``run_real.py`` now delegates connect/summary/error
event emission to this class so the application layer can focus on
orchestration.
"""

from __future__ import annotations

from typing import Any

from ..app.mission_errors import MissionErrorDefinition, MissionErrors


class MissionTelemetryReporter:
    """Build and emit mission telemetry events through the telemetry recorder."""

    def __init__(self, telemetry, fleet):
        self.telemetry = telemetry
        self.fleet = fleet

    # ---- static classification helpers ------------------------------------

    @staticmethod
    def connect_group_outcome(group_result: dict[str, object]) -> str:
        failures = group_result.get("failures", [])
        connected = group_result.get("connected", [])
        if not failures:
            return "success"
        if connected:
            return "partial_failure"
        return "failed"

    @classmethod
    def connect_group_definition(
        cls, group_result: dict[str, object] | None = None
    ) -> MissionErrorDefinition:
        if group_result is None:
            return MissionErrors.Connection.CONNECT_GROUP_START
        outcome = cls.connect_group_outcome(group_result)
        if outcome == "success":
            return MissionErrors.Connection.CONNECT_GROUP_SUCCESS
        if outcome == "partial_failure":
            return MissionErrors.Connection.CONNECT_GROUP_PARTIAL_FAILURE
        return MissionErrors.Connection.CONNECT_GROUP_FAILED

    @classmethod
    def connect_all_outcome(cls, report: dict[str, object], ok: bool) -> str:
        if ok:
            return "success"
        if report.get("connected"):
            return "partial_failure"
        return "failed"

    @staticmethod
    def failed_connect_group_ids(report: dict[str, object]) -> list[int]:
        radio_groups = report.get("radio_groups", {})
        if not isinstance(radio_groups, dict):
            return []
        return sorted(
            int(group_id)
            for group_id, group_result in radio_groups.items()
            if isinstance(group_result, dict) and not group_result.get("ok", False)
        )

    # ---- connect phase ----------------------------------------------------

    def record_connect_group_start(self, group_event: dict[str, object]) -> None:
        if self.telemetry is None:
            return
        definition = self.connect_group_definition()
        self.telemetry.record_event(
            "connect_group_start",
            ok=True,
            category=definition.category,
            code=definition.code,
            stage=definition.stage,
            outcome="start",
            group_id=group_event.get("group_id"),
            drone_ids=group_event.get("drone_ids", []),
        )

    def record_connect_group_result(self, group_result: dict[str, object]) -> None:
        if self.telemetry is None:
            return
        definition = self.connect_group_definition(group_result)
        outcome = self.connect_group_outcome(group_result)
        connected_value = group_result.get("connected", [])
        failures_value = group_result.get("failures", [])
        connected = connected_value if isinstance(connected_value, list) else []
        failures = failures_value if isinstance(failures_value, list) else []
        self.telemetry.record_event(
            "connect_group_result",
            ok=group_result.get("ok", False),
            category=definition.category,
            code=definition.code,
            stage=definition.stage,
            outcome=outcome,
            group_id=group_result.get("group_id"),
            drone_ids=group_result.get("drone_ids", []),
            connected=connected,
            connected_count=len(connected),
            failures=failures,
            failure_count=len(failures),
            failure_drone_ids=[
                item.get("drone_id") for item in failures if isinstance(item, dict)
            ],
            attempted_count=len(connected) + len(failures),
            duration_s=group_result.get("duration_s"),
        )

    def record_connect_all(
        self,
        *,
        ok: bool,
        report: dict[str, object],
        error: str | None = None,
    ) -> None:
        if self.telemetry is None:
            return
        definition = (
            MissionErrors.Connection.CONNECT_ALL_OK
            if ok
            else MissionErrors.Connection.CONNECT_ALL_FAILED
        )
        outcome = self.connect_all_outcome(report, ok)
        failed_group_ids = self.failed_connect_group_ids(report)
        connected_value = report.get("connected", [])
        failures_value = report.get("failures", [])
        connected = connected_value if isinstance(connected_value, list) else []
        failures = failures_value if isinstance(failures_value, list) else []

        details: dict[str, Any] = {
            "ok": ok,
            "category": definition.category,
            "code": definition.code,
            "stage": definition.stage,
            "outcome": outcome,
            "connected": connected,
            "connected_count": len(connected),
            "failures": failures,
            "failure_count": len(failures),
            "radio_groups": report.get("radio_groups", {}),
            "failed_group_ids": failed_group_ids,
            "duration_s": report.get("duration_s"),
        }
        if error is not None:
            details["error"] = error
        self.telemetry.record_event("connect_all", **details)

    # ---- radio-group helpers ----------------------------------------------

    def radio_group_summary(
        self, drone_ids: list[int]
    ) -> dict[int, dict[str, list[int]]]:
        if self.fleet is None:
            return {}
        groups: dict[int, dict[str, list[int]]] = {}
        for drone_id in drone_ids:
            group_id = self.fleet.get_radio_group(drone_id)
            entry = groups.setdefault(group_id, {"drone_ids": []})
            entry["drone_ids"].append(drone_id)
        return groups

    def radio_group_item_summary(
        self,
        items: list[dict],
        *,
        drone_key: str = "drone_id",
        item_key: str = "items",
    ) -> dict[int, dict[str, list[object]]]:
        if self.fleet is None:
            return {}
        groups: dict[int, dict[str, list[object]]] = {}
        for item in items:
            drone_id = item.get(drone_key)
            if drone_id is None:
                continue
            group_id = self.fleet.get_radio_group(drone_id)
            entry = groups.setdefault(group_id, {"drone_ids": [], item_key: []})
            entry["drone_ids"].append(drone_id)
            entry[item_key].append(item)
        return groups

    # ---- error / executor summaries ---------------------------------------

    def record_error(
        self,
        *,
        definition: MissionErrorDefinition,
        message: str,
        mission_state: str | None = None,
        exception: Exception | None = None,
        **details,
    ) -> None:
        if self.telemetry is None:
            return

        payload: dict[str, Any] = {
            "ok": False,
            "category": definition.category,
            "code": definition.code,
            "stage": definition.stage,
            "message": message,
            "mission_state": mission_state,
        }
        if exception is not None:
            payload["exception_type"] = type(exception).__name__
            payload["exception_message"] = str(exception)
        payload.update(details)
        self.telemetry.record_event("mission_error", **payload)

    def record_executor_summary(self, event_name: str, results: list[dict]) -> None:
        if self.telemetry is None or self.fleet is None:
            return

        total_successes: list[int] = []
        total_failures: list[dict] = []
        groups: dict[int, dict[str, list[object]]] = {}
        for result in results:
            for drone_id in result.get("successes", []):
                total_successes.append(drone_id)
                group_id = self.fleet.get_radio_group(drone_id)
                group_entry = groups.setdefault(
                    group_id,
                    {"drone_ids": [], "successes": [], "failures": []},
                )
                group_entry["drone_ids"].append(drone_id)
                group_entry["successes"].append(drone_id)
            for failure in result.get("failures", []):
                total_failures.append(failure)
                drone_id = failure.get("drone_id")
                if drone_id is None:
                    continue
                group_id = self.fleet.get_radio_group(drone_id)
                group_entry = groups.setdefault(
                    group_id,
                    {"drone_ids": [], "successes": [], "failures": []},
                )
                group_entry["drone_ids"].append(drone_id)
                group_entry["failures"].append(failure)

        self.telemetry.record_event(
            event_name,
            ok=not total_failures,
            successes=total_successes,
            failures=total_failures,
            radio_groups=groups,
            batch_count=len(results),
        )
