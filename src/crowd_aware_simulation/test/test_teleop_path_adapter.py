import importlib.util
import math
from pathlib import Path

import pytest


path = Path(__file__).resolve().parents[1] / 'scripts' / 'teleop_path_adapter.py'
spec = importlib.util.spec_from_file_location('teleop_path_adapter', path)
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def test_straight_projection_starts_at_robot_pose():
    points = adapter.integrate_twist(1.0, 2.0, math.pi / 2.0,
                                     0.2, 0.0, 2.0, 0.1)
    assert points[0] == pytest.approx((1.0, 2.0, math.pi / 2.0))
    assert points[-1] == pytest.approx((1.0, 2.4, math.pi / 2.0))


def test_arc_projection_uses_differential_drive_kinematics():
    points = adapter.integrate_twist(0.0, 0.0, 0.0, 0.2, 0.4, 2.0, 0.1)
    radius = 0.2 / 0.4
    assert points[-1] == pytest.approx((
        radius * math.sin(0.8),
        radius * (1.0 - math.cos(0.8)),
        0.8))


def test_rotation_projection_has_no_lateral_translation():
    points = adapter.integrate_twist(1.0, -1.0, 0.3, 0.0, -0.5, 2.0, 0.1)
    assert all(x == pytest.approx(1.0) and y == pytest.approx(-1.0)
               for x, y, _yaw in points)
    assert points[-1][2] == pytest.approx(-0.7)
