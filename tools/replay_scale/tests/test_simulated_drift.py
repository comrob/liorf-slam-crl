"""Injecting a known error into the complementary odometry.

The point of the feature is that the error is *known*, so these check the
injected displacement is exactly alpha x distance along the chosen body axis --
otherwise "we put in alpha, did we get it back" proves nothing.
"""

import numpy as np
import pytest

from replay_scale.core.model import Frame
from replay_scale.core.odom_source import DRIFT_AXES, apply_complementary_drift


def _frame(twist=(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), dt=0.1, has_complementary=True):
    f = Frame()
    f.time = 0.0
    f.dt_scan = dt
    f.dt_complementary = dt
    f.has_complementary = has_complementary
    f.complementary_twist = np.array(twist, dtype=float)
    return f


def _displacement(f):
    dt = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
    return f.complementary_twist[:3] * dt


def test_drift_adds_alpha_times_distance_along_x():
    f = _frame(twist=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0), dt=0.5)   # 1.0 m along x
    before = _displacement(f).copy()
    assert apply_complementary_drift([f], 0.1, "x") == 1
    np.testing.assert_allclose(_displacement(f), before + [0.1, 0.0, 0.0], atol=1e-12)


def test_drift_is_proportional_to_distance_not_constant():
    short, long = _frame(twist=(1.0, 0, 0, 0, 0, 0)), _frame(twist=(4.0, 0, 0, 0, 0, 0))
    b_short, b_long = _displacement(short).copy(), _displacement(long).copy()
    apply_complementary_drift([short, long], 0.25, "x")
    added_short = _displacement(short) - b_short
    added_long = _displacement(long) - b_long
    assert np.linalg.norm(added_long) == pytest.approx(4.0 * np.linalg.norm(added_short))


def test_drift_along_x_is_a_pure_scale_error_when_travel_is_along_x():
    """alpha = 0.1 must make the odometry report exactly 10% too far."""
    f = _frame(twist=(3.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    before = np.linalg.norm(_displacement(f))
    apply_complementary_drift([f], 0.1, "x")
    assert np.linalg.norm(_displacement(f)) == pytest.approx(1.1 * before)


def test_drift_uses_the_full_distance_not_just_the_axis_component():
    # Travelling purely along y, a drift on x is still sized by |displacement|.
    f = _frame(twist=(0.0, 2.0, 0.0, 0.0, 0.0, 0.0), dt=0.5)   # 1.0 m along y
    apply_complementary_drift([f], 0.3, "x")
    np.testing.assert_allclose(_displacement(f), [0.3, 1.0, 0.0], atol=1e-12)


@pytest.mark.parametrize("axis,expected", [
    ("x", [0.1, 1.0, 0.0]),
    ("y", [0.0, 1.1, 0.0]),
    ("z", [0.0, 1.0, 0.1]),
])
def test_drift_axis_selects_the_body_component(axis, expected):
    f = _frame(twist=(0.0, 2.0, 0.0, 0.0, 0.0, 0.0), dt=0.5)
    apply_complementary_drift([f], 0.1, axis)
    np.testing.assert_allclose(_displacement(f), expected, atol=1e-12)


def test_negative_alpha_shortens_the_displacement():
    f = _frame(twist=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0), dt=0.5)
    apply_complementary_drift([f], -0.2, "x")
    assert np.linalg.norm(_displacement(f)) == pytest.approx(0.8)


def test_angular_part_is_untouched():
    f = _frame(twist=(1.0, 0.0, 0.0, 0.01, 0.02, 0.03))
    apply_complementary_drift([f], 0.5, "x")
    np.testing.assert_allclose(f.complementary_twist[3:], [0.01, 0.02, 0.03], atol=1e-12)


def test_frames_without_complementary_odometry_are_skipped():
    f = _frame(has_complementary=False)
    assert apply_complementary_drift([f], 0.1, "x") == 0


def test_non_finite_twists_are_skipped():
    f = _frame(twist=(np.nan,) * 6)
    assert apply_complementary_drift([f], 0.1, "x") == 0


def test_stationary_frames_gain_nothing():
    f = _frame(twist=(0.0,) * 6)
    assert apply_complementary_drift([f], 0.1, "x") == 0
    np.testing.assert_allclose(_displacement(f), np.zeros(3), atol=1e-12)


def test_zero_alpha_is_a_no_op():
    f = _frame()
    before = f.complementary_twist.copy()
    assert apply_complementary_drift([f], 0.0, "x") == 0
    np.testing.assert_allclose(f.complementary_twist, before, atol=1e-12)


def test_unknown_axis_is_rejected():
    with pytest.raises(ValueError, match="drift axis"):
        apply_complementary_drift([_frame()], 0.1, "forward")
    assert DRIFT_AXES == ("x", "y", "z")


def test_drift_falls_back_to_the_scan_interval():
    """Mirrors the dt rule the reconstruction uses, or the injection is wrong."""
    f = _frame(dt=0.1)
    f.dt_complementary = 0.0          # below the 1e-5 gate -> use dt_scan
    f.dt_scan = 0.4
    f.complementary_twist = np.array([2.5, 0.0, 0.0, 0.0, 0.0, 0.0])  # 1.0 m
    apply_complementary_drift([f], 0.2, "x")
    np.testing.assert_allclose(_displacement(f), [1.2, 0.0, 0.0], atol=1e-12)
