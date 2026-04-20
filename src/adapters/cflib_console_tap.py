"""Onboard console tap —— 捕获 Crazyflie firmware `consolePrintf` 输出。

cflib 暴露 `scf.cf.console.receivedChar.add_callback(cb)`，每次收到字符块触发回调。
字符块**不保证按行分段**，所以按 drone 维护一个 buffer，遇到 `\n` 再把整行推到
telemetry 作为 `onboard_console` event。

用法：
    tap = ConsoleTap(link_manager, telemetry)
    tap.start()   # 内部对每架 drone 注册 receivedChar 回调
    tap.stop()    # 清理

事件载荷：
    {"drone_id": int, "line": str}

适用于捕获 onboard assert / panic / EKF warning 等 firmware 侧诊断信息，
这些信息不经过 log block，只能走 console port。
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable

logger = logging.getLogger(__name__)


class ConsoleTap:
    def __init__(
        self,
        link_manager,
        on_line: Callable[[int, str], None] | None = None,
    ):
        self.link_manager = link_manager
        self._on_line = on_line
        self._buffers: dict[int, str] = {}
        self._callbacks: dict[int, Callable[[str], None]] = {}
        self._attached_scfs: dict[int, object] = {}
        self._lock = threading.Lock()
        self._running = False

    def start(self) -> None:
        """对每架已连接的 drone 注册 console 回调"""
        self._running = True
        for drone_id in list(self.link_manager._scfs.keys()):
            self._attach(drone_id)

    def stop(self) -> None:
        """断开回调（cflib Caller 的 remove_callback 需要 original cb 引用）。"""
        self._running = False
        for drone_id in list(self._callbacks):
            self._detach(drone_id)
        self._attached_scfs.clear()
        self._callbacks.clear()
        self._buffers.clear()

    def reattach_drone(self, drone_id: int) -> bool:
        """Reconnect 成功后对新的 SyncCrazyflie 重新挂 console tap。

        对同一个 ``scf`` 重复调用时应保持幂等；当底层 link object 已替换时，
        会先从旧对象移除 callback，再挂到新对象上。
        """

        if not self._running:
            return False
        return self._attach(drone_id)

    def _detach(self, drone_id: int) -> bool:
        cb = self._callbacks.pop(drone_id, None)
        scf = self._attached_scfs.pop(drone_id, None)
        if cb is None or scf is None:
            return False
        try:
            scf.cf.console.receivedChar.remove_callback(cb)
        except Exception:
            logger.exception("Failed to remove console callback for drone %s", drone_id)
            return False
        return True

    def _attach(self, drone_id: int) -> bool:
        scf = self.link_manager._scfs.get(drone_id)
        if scf is None:
            return False
        if self._attached_scfs.get(drone_id) is scf and drone_id in self._callbacks:
            return False

        self._detach(drone_id)
        self._buffers.pop(drone_id, None)

        def _on_char(text: str, _drone_id: int = drone_id) -> None:
            if not self._running:
                return
            self._on_chunk(_drone_id, text)

        try:
            scf.cf.console.receivedChar.add_callback(_on_char)
        except Exception:
            logger.exception("Failed to attach console tap for drone %s", drone_id)
            return False
        self._callbacks[drone_id] = _on_char
        self._attached_scfs[drone_id] = scf
        return True

    def _on_chunk(self, drone_id: int, text: Any) -> None:
        if not isinstance(text, str):
            try:
                text = text.decode("utf-8", errors="replace")
            except Exception:
                text = str(text)

        with self._lock:
            buf = self._buffers.get(drone_id, "") + text
            lines: list[str] = []
            while "\n" in buf:
                line, _, buf = buf.partition("\n")
                line = line.rstrip("\r")
                if line:
                    lines.append(line)
            self._buffers[drone_id] = buf

        if lines and self._on_line is not None:
            for line in lines:
                try:
                    self._on_line(drone_id, line)
                except Exception:
                    logger.exception("Console tap on_line callback failed")
