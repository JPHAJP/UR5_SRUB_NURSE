import sys
import os
import types
import math

# Ensure repository root is on sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Stub out heavy optional dependencies before importing UR
# Create simple placeholder modules for heavy dependencies
for mod in ["pyrealsense2", "cv2", "rtde_control", "rtde_receive", "rtde_io"]:
    sys.modules.setdefault(mod, types.ModuleType(mod))

# "activacion_voz" must provide ActivacionVoz attribute
sys.modules.setdefault("activacion_voz", types.SimpleNamespace(ActivacionVoz=lambda *a, **k: None))
# ultralytics.YOLO is used but not needed for these tests
sys.modules.setdefault("ultralytics", types.SimpleNamespace(YOLO=lambda *a, **k: None))

from UR import mapValue, transformCoordinates


def test_map_value_scaling():
    assert mapValue(50, 0, 100, 0, 1) == 0.5
    assert mapValue(25, 0, 100, -1, 1) == -0.5


def test_transform_coordinates_basic():
    x, y = transformCoordinates(1, 0, 0, 0, 0)
    assert math.isclose(x, 0.0, abs_tol=1e-6)
    assert math.isclose(y, -1.0, abs_tol=1e-6)

    x, y = transformCoordinates(1, 0, 1, 2, 0)
    assert math.isclose(x, 1.0, abs_tol=1e-6)
    assert math.isclose(y, 1.0, abs_tol=1e-6)

    x, y = transformCoordinates(1, 0, 0, 0, math.pi / 2)
    assert math.isclose(x, 1.0, abs_tol=1e-6)
    assert math.isclose(y, 0.0, abs_tol=1e-6)
