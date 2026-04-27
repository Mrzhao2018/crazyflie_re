"""起飞前检查"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class PreflightCheck:
    code: str
    passed: bool
    message: str
    details: dict = field(default_factory=dict)


@dataclass
class PreflightReport:
    ok: bool
    reasons: list[str]
    checks: list[PreflightCheck] = field(default_factory=list)
    readiness: dict = field(default_factory=dict)

    @property
    def failed_codes(self) -> list[str]:
        return [check.code for check in self.checks if not check.passed]


class PreflightRunner:
    def __init__(self, components: dict):
        self.comp = components

    def run(self) -> PreflightReport:
        checks: list[PreflightCheck] = []

        def add_check(code: str, passed: bool, message: str, **details):
            checks.append(
                PreflightCheck(
                    code=code,
                    passed=passed,
                    message=message,
                    details=details,
                )
            )

        fleet = self.comp["fleet"]
        formation = self.comp["formation"]
        pose_bus = self.comp["pose_bus"]
        health_bus = self.comp["health_bus"]
        config = self.comp["config"]
        readiness = self.comp.get("readiness_report", {})
        is_crazyswarm_sim = self.comp.get("runtime_backend") == "crazyswarm2_sim"

        add_check(
            "LEADER_COUNT",
            len(fleet.leader_ids()) >= 4,
            "Affine formation requires at least 4 leaders",
            actual=len(fleet.leader_ids()),
        )

        add_check(
            "FOLLOWER_COUNT",
            len(fleet.follower_ids()) >= 1,
            "V2 requires at least 1 follower",
            actual=len(fleet.follower_ids()),
        )

        span = formation.check_affine_span(fleet.leader_ids())
        add_check(
            "AFFINE_SPAN",
            span.valid,
            f"Leader affine span valid (rank={span.rank})",
            rank=span.rank,
            condition_number=span.condition_number,
        )

        trajectory_prepare = readiness.get("trajectory_prepare", {})
        if config.mission.leader_motion.trajectory_enabled:
            leaders_ready = bool(trajectory_prepare) and all(
                item.get("uploaded")
                and item.get("defined")
                and item.get("fits_memory", True)
                for item in trajectory_prepare.values()
            )
            add_check(
                "TRAJECTORY_READY",
                leaders_ready,
                "Trajectory leaders prepared and fit memory",
                leaders=trajectory_prepare,
            )

        snapshot = pose_bus.latest()
        health_samples = health_bus.latest()
        if snapshot is None:
            add_check("POSE_AVAILABLE", False, "No pose snapshot available")
        else:
            add_check(
                "POSE_AVAILABLE", True, "Pose snapshot available", seq=snapshot.seq
            )
            add_check(
                "POSE_FRESH",
                bool(all(snapshot.fresh_mask)),
                "All drone poses are fresh",
                fresh_mask=snapshot.fresh_mask.tolist(),
            )
            add_check(
                "DISCONNECTED",
                not bool(snapshot.disconnected_ids),
                "No disconnected drones",
                disconnected_ids=snapshot.disconnected_ids,
            )
            pose_window = {}
            recent_pose_fn = getattr(pose_bus, "recent_samples", None)
            if callable(recent_pose_fn):
                pose_window = recent_pose_fn(config.safety.estimator_variance_window_s)
            if pose_window:
                sample_counts = {
                    int(drone_id): len(samples)
                    for drone_id, samples in pose_window.items()
                }
                add_check(
                    "POSE_WINDOW_SAMPLES",
                    all(
                        sample_counts.get(drone_id, 0) >= 2
                        for drone_id in fleet.all_ids()
                    ),
                    "Pose window has enough samples for jitter check",
                    window_s=config.safety.estimator_variance_window_s,
                    sample_counts=sample_counts,
                )
                if is_crazyswarm_sim and config.safety.pose_jitter_threshold > 0:
                    add_check(
                        "POSE_JITTER_SIM_SKIPPED",
                        True,
                        "Crazyswarm2 sim skips Lighthouse pose jitter preflight",
                        threshold=config.safety.pose_jitter_threshold,
                    )
                elif config.safety.pose_jitter_threshold > 0:
                    for drone_id in fleet.all_ids():
                        samples = pose_window.get(drone_id, [])
                        if len(samples) < 2:
                            continue
                        positions = np.array([pos for _t, pos in samples], dtype=float)
                        center = np.mean(positions, axis=0)
                        jitter = float(np.max(np.linalg.norm(positions - center, axis=1)))
                        add_check(
                            f"POSE_JITTER_DRONE_{drone_id}",
                            jitter <= config.safety.pose_jitter_threshold,
                            f"Drone {drone_id} pose jitter too high",
                            drone_id=drone_id,
                            jitter=jitter,
                            threshold=config.safety.pose_jitter_threshold,
                            sample_count=len(samples),
                        )
            if is_crazyswarm_sim and config.safety.min_inter_drone_distance > 0:
                add_check(
                    "FORMATION_SPACING_SIM_SKIPPED",
                    True,
                    "Crazyswarm2 sim skips ground-spawn spacing preflight",
                    threshold=config.safety.min_inter_drone_distance,
                )
            elif config.safety.min_inter_drone_distance > 0:
                min_pair = None
                min_distance = None
                ids = fleet.all_ids()
                for i, left_id in enumerate(ids):
                    left_pos = snapshot.positions[fleet.id_to_index(left_id)]
                    for right_id in ids[i + 1:]:
                        right_pos = snapshot.positions[fleet.id_to_index(right_id)]
                        distance = float(np.linalg.norm(left_pos - right_pos))
                        if min_distance is None or distance < min_distance:
                            min_distance = distance
                            min_pair = [left_id, right_id]
                add_check(
                    "FORMATION_SPACING",
                    min_distance is None
                    or min_distance >= config.safety.min_inter_drone_distance,
                    "Formation inter-drone spacing below safety margin",
                    min_distance=min_distance,
                    pair=min_pair,
                    threshold=config.safety.min_inter_drone_distance,
                )

            for drone_id in fleet.all_ids():
                idx = fleet.id_to_index(drone_id)
                pos = snapshot.positions[idx]
                in_bounds = not any(
                    pos[i] < config.safety.boundary_min[i]
                    or pos[i] > config.safety.boundary_max[i]
                    for i in range(3)
                )
                add_check(
                    f"BOUNDARY_DRONE_{drone_id}",
                    in_bounds,
                    f"Drone {drone_id} outside boundary before takeoff",
                    drone_id=drone_id,
                    position=pos.tolist(),
                )

                sample = health_samples.get(drone_id)
                has_health = sample is not None and "pm.vbat" in sample.values
                safety_t_meas = (
                    getattr(sample, "safety_t_meas", sample.t_meas)
                    if sample is not None
                    else None
                )
                health_fresh = bool(
                    has_health
                    and snapshot is not None
                    and safety_t_meas is not None
                    and safety_t_meas >= snapshot.t_meas - config.safety.pose_timeout
                )
                add_check(
                    f"HEALTH_DRONE_{drone_id}",
                    has_health,
                    f"Drone {drone_id} health sample missing",
                    drone_id=drone_id,
                )
                if has_health:
                    add_check(
                        f"HEALTH_FRESH_DRONE_{drone_id}",
                        health_fresh,
                        f"Drone {drone_id} health sample stale",
                        drone_id=drone_id,
                        t_meas=safety_t_meas,
                        sample_t_meas=sample.t_meas,
                        snapshot_t_meas=snapshot.t_meas,
                    )
                    if config.safety.min_vbat > 0:
                        add_check(
                            f"VBAT_DRONE_{drone_id}",
                            float(sample.values["pm.vbat"]) >= config.safety.min_vbat,
                            f"Drone {drone_id} battery below threshold",
                            drone_id=drone_id,
                            vbat=float(sample.values["pm.vbat"]),
                            threshold=config.safety.min_vbat,
                        )
                    if config.safety.estimator_variance_threshold > 0:
                        variance_values = [
                            sample.values.get("kalman.varPX"),
                            sample.values.get("kalman.varPY"),
                            sample.values.get("kalman.varPZ"),
                        ]
                        present = [
                            float(value)
                            for value in variance_values
                            if value is not None
                        ]
                        if present:
                            variance_max = max(present)
                            add_check(
                                f"ESTIMATOR_VARIANCE_DRONE_{drone_id}",
                                variance_max <= config.safety.estimator_variance_threshold,
                                f"Drone {drone_id} estimator variance too high",
                                drone_id=drone_id,
                                variance=variance_max,
                                threshold=config.safety.estimator_variance_threshold,
                            )
                    required_method = config.safety.lighthouse_required_method
                    if required_method is not None:
                        method = sample.values.get("lighthouse.method")
                        add_check(
                            f"LIGHTHOUSE_METHOD_DRONE_{drone_id}",
                            method == required_method,
                            f"Drone {drone_id} Lighthouse method mismatch",
                            drone_id=drone_id,
                            method=method,
                            required_method=required_method,
                        )

            recent_health_fn = getattr(health_bus, "recent_samples", None)
            if callable(recent_health_fn) and config.safety.estimator_variance_threshold > 0:
                health_window = recent_health_fn(config.safety.estimator_variance_window_s)
                for drone_id in fleet.all_ids():
                    samples = health_window.get(drone_id, [])
                    variance_values = []
                    for sample in samples:
                        for key in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):
                            value = sample.values.get(key)
                            if value is not None:
                                variance_values.append(float(value))
                    if variance_values:
                        variance_max = max(variance_values)
                        add_check(
                            f"ESTIMATOR_VARIANCE_WINDOW_DRONE_{drone_id}",
                            variance_max <= config.safety.estimator_variance_threshold,
                            f"Drone {drone_id} estimator variance window too high",
                            drone_id=drone_id,
                            variance=variance_max,
                            threshold=config.safety.estimator_variance_threshold,
                            sample_count=len(samples),
                            window_s=config.safety.estimator_variance_window_s,
                        )

        reasons = [check.message for check in checks if not check.passed]
        return PreflightReport(
            ok=not reasons,
            reasons=reasons,
            checks=checks,
            readiness=readiness,
        )
