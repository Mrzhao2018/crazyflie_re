"""radio_driver 选择器契约（cflinkcpp 可选依赖）。"""

import os

import cflib.crtp
from cflib.crtp.radiodriver import RadioDriver

from src.adapters.radio_driver_select import (
    RadioDriverMode,
    cflinkcpp_available,
    select_radio_driver,
)


# ---- "python" ----------------------------------------------------------

select_radio_driver(RadioDriverMode.PYTHON)
assert any(cls is RadioDriver for cls in cflib.crtp.CLASSES), (
    "python 模式下 RadioDriver 必须在 CLASSES 中"
)
assert os.environ.get("USE_CFLINK") != "cpp"

# ---- "cpp" -------------------------------------------------------------

if cflinkcpp_available():
    from cflib.crtp.cflinkcppdriver import CfLinkCppDriver

    select_radio_driver(RadioDriverMode.CPP)
    assert any(cls is CfLinkCppDriver for cls in cflib.crtp.CLASSES)
    assert os.environ.get("USE_CFLINK") == "cpp"
else:
    # 未安装 cflinkcpp 包：必须抛 RuntimeError，提示用户安装或换模式
    try:
        select_radio_driver(RadioDriverMode.CPP)
    except RuntimeError as exc:
        assert "cflinkcpp" in str(exc).lower()
    else:
        raise AssertionError("缺失 cflinkcpp 时 cpp 模式必须抛 RuntimeError")

# ---- "auto" -------------------------------------------------------------

os.environ.pop("USE_CFLINK", None)
select_radio_driver(RadioDriverMode.AUTO)
assert "USE_CFLINK" not in os.environ, "auto 模式不应主动设置 USE_CFLINK"
assert any(cls is RadioDriver for cls in cflib.crtp.CLASSES)

# ---- 字符串入口 --------------------------------------------------------

select_radio_driver("python")
assert any(cls is RadioDriver for cls in cflib.crtp.CLASSES)

# ---- 非法输入 ---------------------------------------------------------

try:
    select_radio_driver("bogus")
except ValueError as exc:
    assert "bogus" in str(exc)
else:
    raise AssertionError("未知 radio_driver 应当抛 ValueError")

# 清理副作用
os.environ.pop("USE_CFLINK", None)
select_radio_driver(RadioDriverMode.PYTHON)

print("[OK] radio_driver selector contracts verified")
