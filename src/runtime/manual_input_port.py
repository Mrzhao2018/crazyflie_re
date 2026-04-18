"""手动输入端口抽象"""

from dataclasses import dataclass
from typing import Protocol


@dataclass
class ManualLeaderIntent:
    """一次手动输入意图"""

    translation_delta: tuple[float, float, float] = (0.0, 0.0, 0.0)
    scale_delta: float = 0.0
    rotation_delta_deg: float = 0.0
    axis_switch: bool = False
    target_switch: bool = False


class ManualInputPort(Protocol):
    """手动输入端口"""

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def poll(self) -> ManualLeaderIntent | None: ...
