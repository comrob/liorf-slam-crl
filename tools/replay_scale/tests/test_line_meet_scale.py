"""Taking the scale from where the degenerate lines meet.

The idea: express every frame the way the viewer does when normalized -- rotated
so its complementary displacement is +x, divided by that displacement's length --
and each frame's degenerate line becomes a statement about the same quantity.
Where they meet, the x coordinate *is* the scale: how far the robot really went
per unit the odometry claimed.
"""

import numpy as np
import pytest

from replay_scale.core.estimator import (
    line_meet_scale,
    own_normalized_line,
    strided_lines,
)

#: A basis twist whose linear part points along the given axis.
def _basis(axis):
    twist = np.zeros(6, dtype=float)
    twist[:3] = axis
    return [twist]


# ---------------------------------------------------------------------------
# One frame's line, in its own normalized geometry
# ---------------------------------------------------------------------------

def test_the_line_passes_through_the_lidar_position_over_comp():
    """|lidar| / |comp| along the complementary direction: the ratio itself."""
    origin, direction = own_normalized_line([1.0, 0.0, 0.0], [2.0, 0.0, 0.0],
                                            _basis([0.0, 1.0, 0.0]))
    np.testing.assert_allclose(origin, [0.5, 0.0], atol=1e-12)
    np.testing.assert_allclose(np.abs(direction), [0.0, 1.0], atol=1e-12)


def test_the_frame_is_rotated_so_comp_points_along_x():
    """A frame travelling along +y must look like one travelling along +x."""
    origin, direction = own_normalized_line([0.0, 1.0, 0.0], [0.0, 2.0, 0.0],
                                            _basis([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(origin, [0.5, 0.0], atol=1e-12)
    np.testing.assert_allclose(np.abs(direction), [0.0, 1.0], atol=1e-12)


def test_a_lateral_lidar_offset_survives_the_rotation():
    origin, _ = own_normalized_line([1.0, 0.5, 0.0], [2.0, 0.0, 0.0],
                                    _basis([0.0, 1.0, 0.0]))
    np.testing.assert_allclose(origin, [0.5, 0.25], atol=1e-12)


def test_a_frame_without_a_degenerate_basis_has_no_line():
    assert own_normalized_line([1.0, 0.0, 0.0], [2.0, 0.0, 0.0], []) is None


def test_a_frame_without_a_complementary_vector_has_no_line():
    """There is nothing to align to, and nothing to divide by."""
    assert own_normalized_line([1.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                               _basis([0.0, 1.0, 0.0])) is None


def test_a_purely_vertical_degenerate_axis_has_no_line():
    """Nothing in the plane to draw, so the frame constrains nothing here."""
    assert own_normalized_line([1.0, 0.0, 0.0], [2.0, 0.0, 0.0],
                               _basis([0.0, 0.0, 1.0])) is None


# ---------------------------------------------------------------------------
# The scale the lines agree on
# ---------------------------------------------------------------------------

def _line(origin, angle_rad):
    return (np.array(origin, dtype=float),
            np.array([np.cos(angle_rad), np.sin(angle_rad)]))


def test_the_scale_is_the_x_of_the_meeting_point():
    """Both lines pass through (0.6, 0): the robot went 0.6 of what was claimed."""
    lines = [_line([0.6, 0.3], np.pi / 2), _line([0.6 - 0.2, -0.2], np.pi / 4)]
    assert line_meet_scale(lines) == pytest.approx(0.6)


def test_the_lateral_component_is_discarded():
    """Only the along-complementary coordinate is a scale; y is disagreement."""
    lines = [_line([0.8, 0.5], np.pi / 2), _line([0.8, 0.5], 0.0)]
    assert line_meet_scale(lines) == pytest.approx(0.8)


def test_lines_that_stay_parallel_give_no_scale():
    """The straight-tunnel case: every frame says the same thing."""
    lines = [_line([0.5, 0.0], np.pi / 2), _line([0.5, 1.0], np.pi / 2)]
    assert np.isnan(line_meet_scale(lines))


def test_a_meeting_point_behind_the_anchor_is_not_a_scale():
    """A negative scale is the fit disagreeing with the direction of travel."""
    lines = [_line([-0.6, 0.3], np.pi / 2), _line([-0.6, -0.2], np.pi / 4)]
    assert np.isnan(line_meet_scale(lines))


def test_one_line_is_not_enough():
    assert np.isnan(line_meet_scale([_line([0.5, 0.0], np.pi / 2)]))


def test_the_l1_fit_ignores_one_wrong_line():
    """Nine lines agree on 0.5, one is badly wrong; l2 is dragged, l1 is not."""
    lines = [_line([0.5, 0.0], a) for a in np.linspace(0.4, 2.7, 9)]
    lines.append(_line([2.5, 0.0], np.pi / 2))
    assert line_meet_scale(lines, norm="l1") == pytest.approx(0.5, abs=1e-3)
    assert line_meet_scale(lines, norm="l2") > 0.6


def test_both_norms_agree_when_the_lines_do():
    lines = [_line([0.7, 0.0], a) for a in np.linspace(0.3, 2.8, 6)]
    assert line_meet_scale(lines, norm="l2") == pytest.approx(0.7, abs=1e-6)
    assert line_meet_scale(lines, norm="l1") == pytest.approx(0.7, abs=1e-6)


# ---------------------------------------------------------------------------
# Which earlier lines the fit sees
# ---------------------------------------------------------------------------

def test_the_stride_takes_every_nth_line_newest_first():
    history = list(range(10))          # 0 oldest .. 9 newest
    assert strided_lines(history, 3, 1) == [9, 8, 7]
    assert strided_lines(history, 3, 2) == [9, 7, 5]
    assert strided_lines(history, 3, 4) == [9, 5, 1]


def test_the_stride_buys_span_at_the_same_number_of_lines():
    """50 x 4 reaches back 200 frames while still fitting 50 lines."""
    history = list(range(400))
    strided = strided_lines(history, 50, 4)
    assert len(strided) == 50
    assert history[-1] - strided[-1] == 196      # a 200-frame span


def test_a_short_history_gives_what_there_is():
    assert strided_lines([7, 8], 5, 3) == [8]


def test_asking_for_no_lines_gives_none():
    assert strided_lines(list(range(10)), 0, 2) == []
    assert strided_lines([], 5, 2) == []


def test_a_step_below_one_is_treated_as_one():
    """The config rejects it; the helper still must not return an empty slice."""
    assert strided_lines([1, 2, 3], 2, 0) == [3, 2]
