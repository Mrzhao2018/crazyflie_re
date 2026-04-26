"""CflibLinkManager ↔ LinkStateBus ↔ SafetyManager contracts."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np

from src.adapters import cflib_link_manager as link_manager_module
from src.adapters.cflib_link_manager import CflibLinkManager
from src.app.mission_errors import MissionErrors
from src.app.run_real import RealMissionApp
from src.config.loader import ConfigLoader
from src.runtime.link_state_bus import LinkStateBus
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyManager, SafetyDecision
from src.tests.run_real_fixtures import build_components, make_snapshot


class FakeCaller:
    def __init__(self):
        self.callbacks = []

    def add_callback(self, cb):
        self.callbacks.append(cb)

    def call(self, *args):
        for cb in list(self.callbacks):
            cb(*args)


class FakeCf:
    def __init__(self):
        self.connection_lost = FakeCaller()
        self.disconnected = FakeCaller()
        self.link_statistics = object()

    def is_connected(self):
        return True


class FakeScf:
    def __init__(self, uri, cf):
        self.uri = uri
        self.cf = cf
        self.opened = False

    def open_link(self):
        self.opened = True

    def close_link(self):
        self.opened = False


class FakeFleet:
    def __init__(self):
        self._uris = {1: "radio://0/60/2M/E7E7E7E701"}

    def all_ids(self):
        return [1]

    def get_uri(self, drone_id):
        return self._uris[drone_id]

    def get_radio_group(self, drone_id):
        return 0

    def id_to_index(self, drone_id):
        return 0


fleet = FakeFleet()
link_state_bus = LinkStateBus()

with patch.object(link_manager_module, "cflib", MagicMock()), patch.object(
    link_manager_module, "Crazyflie", side_effect=lambda **kwargs: FakeCf()
), patch.object(
    link_manager_module, "SyncCrazyflie", side_effect=lambda uri, cf: FakeScf(uri, cf)
), patch.object(link_manager_module, "select_radio_driver", MagicMock()):
    manager = CflibLinkManager(
        fleet,
        link_state_bus=link_state_bus,
        connect_pace_s=0.0,
        connect_timeout_s=0.1,
    )
    report = manager.connect_all()
    assert report["ok"] is True
    assert link_state_bus.latest()[1].state == "connected"

    scf = manager.get(1)
    scf.cf.connection_lost.call(scf.uri, "radio lost")
    latest = link_state_bus.latest()
    assert latest[1].state == "disconnected"
    assert latest[1].error == "radio lost"
    assert link_state_bus.disconnected_ids() == [1]


config = ConfigLoader.load("config")
safety = SafetyManager(config.safety, fleet, link_state_bus=link_state_bus)
snapshot = PoseSnapshot(
    seq=1,
    t_meas=0.0,
    positions=np.array([[0.0, 0.0, 0.5]], dtype=float),
    fresh_mask=np.array([True], dtype=bool),
    disconnected_ids=[],
)

blocked, reasons = safety.fast_gate(snapshot)
assert blocked is True
assert reasons == ["DISCONNECTED:[1]"]

decision = safety.fast_gate_decision(snapshot)
assert decision.action == "ABORT"
assert decision.reason_codes == ["DISCONNECTED:[1]"]

components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
)
components["link_state_bus"] = link_state_bus
app = RealMissionApp(components)
app._record_link_state_events()

connected_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "link_state_change"
    and event["details"]["state"] == "connected"
)
assert connected_event["details"]["ok"] is True
assert connected_event["details"]["code"] == MissionErrors.Runtime.LINK_STATE_CHANGE.code

disconnected_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "link_state_change"
    and event["details"]["state"] == "disconnected"
)
assert disconnected_event["details"]["ok"] is False
assert disconnected_event["details"]["drone_id"] == 1
assert disconnected_event["details"]["error"] == "radio lost"
assert disconnected_event["details"]["code"] == MissionErrors.Runtime.LINK_STATE_CHANGE.code

print("[OK] CflibLinkManager connection_lost <-> LinkStateBus contracts verified")
