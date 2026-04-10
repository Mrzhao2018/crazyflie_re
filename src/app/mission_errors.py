"""Structured mission error definitions for real-flight telemetry."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MissionErrorDefinition:
    category: str
    code: str
    stage: str


class MissionErrors:
    class Connection:
        """用于标识链路建立与设备连接阶段失败。"""

        CONNECT_ALL_FAILED = MissionErrorDefinition(
            category="connection",
            code="CONNECT_ALL_FAILED",
            stage="connect_all",
        )

    class Readiness:
        """用于标识启动、readiness、preflight 与起飞验证阶段失败。"""

        STARTUP_FAILED = MissionErrorDefinition(
            category="readiness",
            code="STARTUP_FAILED",
            stage="startup",
        )
        FSM_CONNECT_TRANSITION_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_FSM_CONNECT_TRANSITION_FAILED",
            stage="fsm_connect",
        )
        WAIT_FOR_PARAMS_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_WAIT_FOR_PARAMS_FAILED",
            stage="wait_for_params",
        )
        RESET_ESTIMATOR_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_RESET_ESTIMATOR_FAILED",
            stage="reset_estimator",
        )
        POSE_SOURCE_START_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_POSE_SOURCE_START_FAILED",
            stage="pose_source_start",
        )
        POSE_TIMEOUT = MissionErrorDefinition(
            category="readiness",
            code="READINESS_POSE_TIMEOUT",
            stage="pose_ready",
        )
        HEALTH_TIMEOUT = MissionErrorDefinition(
            category="readiness",
            code="READINESS_HEALTH_TIMEOUT",
            stage="health_ready",
        )
        TRAJECTORY_PREPARE_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_TRAJECTORY_PREPARE_FAILED",
            stage="trajectory_prepare",
        )
        FSM_POSE_READY_TRANSITION_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_FSM_POSE_READY_TRANSITION_FAILED",
            stage="fsm_pose_ready",
        )
        FSM_PREFLIGHT_TRANSITION_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_FSM_PREFLIGHT_TRANSITION_FAILED",
            stage="fsm_preflight",
        )
        PREFLIGHT_EXCEPTION = MissionErrorDefinition(
            category="readiness",
            code="READINESS_PREFLIGHT_EXCEPTION",
            stage="preflight",
        )
        PREFLIGHT_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_PREFLIGHT_FAILED",
            stage="preflight",
        )
        FSM_TAKEOFF_TRANSITION_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_FSM_TAKEOFF_TRANSITION_FAILED",
            stage="fsm_takeoff",
        )
        FSM_SETTLE_TRANSITION_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_FSM_SETTLE_TRANSITION_FAILED",
            stage="fsm_settle",
        )
        TAKEOFF_VALIDATION_FAILED = MissionErrorDefinition(
            category="readiness",
            code="READINESS_TAKEOFF_VALIDATION_FAILED",
            stage="takeoff_validation",
        )

    class Runtime:
        """用于标识进入主循环之后的运行期异常。"""

        RUN_LOOP_EXCEPTION = MissionErrorDefinition(
            category="runtime",
            code="RUN_LOOP_EXCEPTION",
            stage="run_loop",
        )