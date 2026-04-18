"""键盘手动输入适配器"""

from collections.abc import Callable

from ..runtime.manual_input_port import ManualInputPort, ManualLeaderIntent


class KeyboardManualInputSource(ManualInputPort):
    """Windows 非阻塞键盘输入适配器。

    输入映射：
    - WASD: xy 平移
    - R/F: z 平移
    - Q/E: 缩放
    - Z/X: 绕当前轴旋转
    - C: 切换旋转轴
    - V: 切换控制目标（集群 / leader1 / leader2 / leader3 / leader4）
    """

    def __init__(
        self,
        translation_step: float,
        vertical_step: float,
        scale_step: float,
        rotation_step_deg: float,
        key_reader: Callable[[], str | None] | None = None,
    ):
        self._started = False
        self._translation_step = float(translation_step)
        self._vertical_step = float(vertical_step)
        self._scale_step = float(scale_step)
        self._rotation_step_deg = float(rotation_step_deg)
        self._key_reader = key_reader or self._read_key_nonblocking

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def poll(self) -> ManualLeaderIntent | None:
        if not self._started:
            return None

        key = self._key_reader()
        if key is None:
            return None
        return self._intent_for_key(key)

    def _read_key_nonblocking(self) -> str | None:
        import msvcrt

        if not msvcrt.kbhit():
            return None

        ch = msvcrt.getwch()
        if ch in {"\x00", "\xe0"}:
            # 丢弃扩展键的第二个字节，避免污染输入流
            if msvcrt.kbhit():
                msvcrt.getwch()
            return None
        return ch

    def _intent_for_key(self, key: str) -> ManualLeaderIntent | None:
        k = key.lower()
        step = self._translation_step
        z_step = self._vertical_step

        if k == "w":
            return ManualLeaderIntent(translation_delta=(step, 0.0, 0.0))
        if k == "s":
            return ManualLeaderIntent(translation_delta=(-step, 0.0, 0.0))
        if k == "a":
            return ManualLeaderIntent(translation_delta=(0.0, step, 0.0))
        if k == "d":
            return ManualLeaderIntent(translation_delta=(0.0, -step, 0.0))
        if k == "r":
            return ManualLeaderIntent(translation_delta=(0.0, 0.0, z_step))
        if k == "f":
            return ManualLeaderIntent(translation_delta=(0.0, 0.0, -z_step))
        if k == "q":
            return ManualLeaderIntent(scale_delta=self._scale_step)
        if k == "e":
            return ManualLeaderIntent(scale_delta=-self._scale_step)
        if k == "z":
            return ManualLeaderIntent(rotation_delta_deg=-self._rotation_step_deg)
        if k == "x":
            return ManualLeaderIntent(rotation_delta_deg=self._rotation_step_deg)
        if k == "c":
            return ManualLeaderIntent(axis_switch=True)
        if k == "v":
            return ManualLeaderIntent(target_switch=True)
        return None
