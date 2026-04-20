"""起飞前检查"""

from dataclasses import dataclass, field


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

        reasons = [check.message for check in checks if not check.passed]
        return PreflightReport(
            ok=not reasons,
            reasons=reasons,
            checks=checks,
            readiness=readiness,
        )
