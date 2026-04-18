"""radio_driver 选择器。

cflib 通过 module-global ``CLASSES`` 列表决定驱动注册顺序；其 ``init_drivers``
根据 ``USE_CFLINK`` 环境变量在 Python ``RadioDriver`` 与 C++ ``CfLinkCppDriver``
之间做互斥选择。本模块把这套约定包一层，给项目提供：

* ``auto``：不碰 ``USE_CFLINK``，沿用 cflib 默认；
* ``python``：明确把 ``USE_CFLINK`` 清掉，走 Python 驱动；
* ``cpp``：设 ``USE_CFLINK=cpp``，走 C++ 驱动（需要 ``cflinkcpp`` 包）。

每次调用都会先清空 ``cflib.crtp.CLASSES``，避免多次初始化累加。
"""

from __future__ import annotations

import importlib.util
import os
from enum import Enum

import cflib.crtp


class RadioDriverMode(str, Enum):
    AUTO = "auto"
    PYTHON = "python"
    CPP = "cpp"


_VALID_MODES = {mode.value for mode in RadioDriverMode}


def cflinkcpp_available() -> bool:
    """``cflinkcpp`` 包是否已安装。"""
    return importlib.util.find_spec("cflinkcpp") is not None


def select_radio_driver(mode: RadioDriverMode | str) -> None:
    """按配置初始化 cflib radio 驱动。"""

    if isinstance(mode, RadioDriverMode):
        resolved = mode.value
    else:
        resolved = str(mode).lower()

    if resolved not in _VALID_MODES:
        raise ValueError(
            f"未知 radio_driver={mode!r}；合法取值：auto / python / cpp"
        )

    if resolved == RadioDriverMode.CPP.value:
        if not cflinkcpp_available():
            raise RuntimeError(
                "radio_driver='cpp' 需要 cflinkcpp 包，请先 pip install cflinkcpp"
            )
        os.environ["USE_CFLINK"] = "cpp"
    elif resolved == RadioDriverMode.PYTHON.value:
        os.environ.pop("USE_CFLINK", None)
    # auto：不动 USE_CFLINK

    cflib.crtp.CLASSES.clear()
    cflib.crtp.init_drivers()
