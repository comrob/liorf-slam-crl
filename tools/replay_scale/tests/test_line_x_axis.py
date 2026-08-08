"""Reading the scale off where one frame's own degenerate line crosses y = 0.

The claim this method rests on: it is the node's ratio, measured on the picture
instead of in the node's algebra. In the plane with a single degenerate
direction they are the same number -- with normal ``n`` perpendicular to the
degenerate direction, the node computes ``|p.n| / |comp.n|`` and this computes
``(p.n) / (comp.n)``. Taking the norms first is the only difference, and it
costs the sign.
"""

import numpy as np
import pytest

from replay_scale.core.estimator import (
    build_additional_odom_scale_sample,
    line_x_axis_scale,
    own_normalized_line,
)


def _line(origin, direction):
    return np.array(origin, dtype=float), np.array(direction, dtype=float)


# ---------------------------------------------------------------------------
# The crossing itself
# ---------------------------------------------------------------------------

def test_a_line_through_the_axis_crosses_where_it_stands():
    """Straight up through (0.7, 0): the robot went 0.7 of what was claimed."""
    assert line_x_axis_scale(_line([0.7, 0.0], [0.0, 1.0])) == pytest.approx(0.7)


def test_the_crossing_is_followed_back_along_the_direction():
    # From (1.0, 0.5) going down-left at 45 degrees, y = 0 at x = 0.5.
    assert line_x_axis_scale(_line([1.0, 0.5], [1.0, 1.0])) == pytest.approx(0.5)


def test_the_direction_s_sign_does_not_matter():
    """A line is a line; which way its unit vector points is arbitrary."""
    forward = line_x_axis_scale(_line([1.0, 0.5], [1.0, 1.0]))
    backward = line_x_axis_scale(_line([1.0, 0.5], [-1.0, -1.0]))
    assert forward == pytest.approx(backward)


def test_a_line_along_the_axis_never_crosses_it():
    """The frame is saying nothing about how far along the odometry went."""
    assert np.isnan(line_x_axis_scale(_line([0.7, 0.3], [1.0, 0.0])))


def test_a_crossing_behind_the_anchor_is_not_a_scale():
    assert np.isnan(line_x_axis_scale(_line([-0.4, 0.2], [0.0, 1.0])))


def test_no_line_is_no_sample():
    assert np.isnan(line_x_axis_scale(None))


# ---------------------------------------------------------------------------
# Against the ratio, on the same geometry
# ---------------------------------------------------------------------------

def _sample(t_lidar, t_comp, degenerate_angle):
    axis = np.zeros(6, dtype=float)
    axis[:3] = [np.cos(degenerate_angle), np.sin(degenerate_angle), 0.0]
    T_anchor, T_latest = np.eye(4), np.eye(4)
    T_latest[:3, 3] = t_lidar
    T_lidar_rel, T_comp_rel = np.eye(4), np.eye(4)
    T_lidar_rel[:3, 3] = t_lidar
    T_comp_rel[:3, 3] = t_comp
    return build_additional_odom_scale_sample(
        T_anchor, T_latest, T_lidar_rel, T_comp_rel, [axis], True, 0.1, 0.0)


def test_it_is_the_ratio_wherever_the_crossing_is_positive():
    rng = np.random.default_rng(0)
    compared = 0
    for _ in range(500):
        t_lidar = np.array([*rng.normal(0.0, 1.0, 2), 0.0])
        t_comp = np.array([*rng.normal(0.0, 1.0, 2), 0.0])
        out = _sample(t_lidar, t_comp, rng.uniform(0.0, np.pi))
        crossing = line_x_axis_scale(out["own_line"])
        if np.isfinite(crossing):
            assert crossing == pytest.approx(out["scale_instant_raw"], abs=1e-9)
            compared += 1
    assert compared > 100, "the geometries were degenerate; nothing was compared"


def test_where_they_differ_the_ratio_is_reporting_a_magnitude():
    """The node folds a backwards crossing onto the positive side; this does not."""
    # LiDAR behind the anchor, degenerate direction across the odometry.
    out = _sample([-0.5, 0.0, 0.0], [1.0, 0.0, 0.0], np.pi / 2)
    assert out["scale_instant_raw"] == pytest.approx(0.5)
    assert np.isnan(line_x_axis_scale(out["own_line"]))


def test_it_reads_the_same_geometry_the_view_draws():
    """The line it crosses is the one the anchor tab puts on screen."""
    t_lidar, t_comp = [1.0, 0.5, 0.0], [2.0, 0.0, 0.0]
    axis = np.zeros(6, dtype=float)
    axis[:3] = [0.0, 1.0, 0.0]
    line = own_normalized_line(t_lidar, t_comp, [axis])
    # |lidar| along x is 1.0 in units of |comp| = 2, and the line runs along y,
    # so it crosses the axis right where it stands.
    assert line_x_axis_scale(line) == pytest.approx(0.5)
