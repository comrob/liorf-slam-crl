"""Where a set of degenerate lines agree.

Each frame's degenerate line says "the truth is somewhere along here". The
point closest to all of them is what those statements agree on -- a position
the LiDAR alone could not give, since the line direction rotates as the robot
turns. These pin the cases with known answers and the refusals.
"""

import numpy as np
import pytest

from replay_scale.core.lines import (
    MIN_LINE_SPREAD,
    closest_point_to_lines,
    covariance_ellipse,
    fit_lines,
)


def _dir(angle_rad):
    return np.array([np.cos(angle_rad), np.sin(angle_rad)])


def test_two_perpendicular_lines_meet_at_their_intersection():
    point = closest_point_to_lines([[0.0, 0.0], [2.0, -1.0]],
                                   [_dir(0.0), _dir(np.pi / 2)])
    np.testing.assert_allclose(point, [2.0, 0.0], atol=1e-12)


def test_lines_through_one_point_meet_there():
    point = closest_point_to_lines([[1.0, 1.0]] * 3,
                                   [_dir(0.0), _dir(1.0), _dir(2.0)])
    np.testing.assert_allclose(point, [1.0, 1.0], atol=1e-12)


def test_two_lines_always_meet_exactly_wherever_they_cross():
    """With two lines there is no least-squares compromise to make."""
    point = closest_point_to_lines([[0.0, 0.2], [0.0, -0.2]],
                                   [_dir(0.4), _dir(-0.4)])
    # The crossing sits where the first line reaches y = 0.
    t = -0.2 / np.sin(0.4)
    np.testing.assert_allclose(point, [t * np.cos(0.4), 0.0], atol=1e-12)


def test_a_symmetric_spread_averages_out():
    """Four lines whose errors cancel: the answer is the point they surround."""
    offsets = [[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]]
    directions = [_dir(np.pi / 2), _dir(np.pi / 2), _dir(0.0), _dir(0.0)]
    point = closest_point_to_lines(offsets, directions)
    np.testing.assert_allclose(point, [0.0, 0.0], atol=1e-12)


def test_the_fit_is_least_squares_not_just_a_pair():
    """A third line pulls the answer off the first two's crossing."""
    origins = [[0.0, 0.0], [0.0, 0.0], [0.0, 2.0]]
    directions = [_dir(0.0), _dir(np.pi / 2), _dir(0.0)]
    point = closest_point_to_lines(origins, directions)
    # Two lines want y = 0, one wants y = 2, all three want x = 0.
    np.testing.assert_allclose(point, [0.0, 1.0], atol=1e-12)


# ---------------------------------------------------------------------------
# When there is no answer worth giving
# ---------------------------------------------------------------------------

def test_parallel_lines_have_no_meeting_point():
    """The straight-tunnel case: they meet nowhere in particular."""
    assert closest_point_to_lines([[0.0, 0.0], [0.0, 1.0]],
                                  [_dir(0.0), _dir(0.0)]) is None


def test_nearly_parallel_lines_are_refused():
    """They do cross, but arbitrarily far away and wherever noise put them."""
    assert closest_point_to_lines([[0.0, 0.0], [0.0, 1.0]],
                                  [_dir(0.0), _dir(0.001)]) is None


def test_the_spread_threshold_is_where_the_refusal_happens():
    """Just inside the threshold answers, just outside refuses."""
    # For two lines the eigenvalue ratio is tan(theta/2)^2.
    theta = 2.0 * np.arctan(np.sqrt(MIN_LINE_SPREAD))
    assert closest_point_to_lines([[0.0, 0.0], [1.0, 0.0]],
                                  [_dir(0.0), _dir(theta * 1.2)]) is not None
    assert closest_point_to_lines([[0.0, 0.0], [1.0, 0.0]],
                                  [_dir(0.0), _dir(theta * 0.8)]) is None


def test_one_line_is_not_enough():
    assert closest_point_to_lines([[0.0, 0.0]], [_dir(0.0)]) is None
    assert closest_point_to_lines([], []) is None


def test_a_zero_length_direction_is_skipped():
    """A degenerate axis with no in-plane part carries no constraint."""
    point = closest_point_to_lines([[0.0, 0.0], [2.0, -1.0], [5.0, 5.0]],
                                   [_dir(0.0), _dir(np.pi / 2), np.zeros(2)])
    np.testing.assert_allclose(point, [2.0, 0.0], atol=1e-12)


def test_direction_sign_does_not_matter():
    """A line has no orientation; flipping its direction is the same line."""
    origins = [[0.0, 0.0], [2.0, -1.0]]
    forward = closest_point_to_lines(origins, [_dir(0.0), _dir(np.pi / 2)])
    backward = closest_point_to_lines(origins, [-_dir(0.0), -_dir(np.pi / 2)])
    np.testing.assert_allclose(forward, backward, atol=1e-12)


def test_directions_need_not_be_unit_length():
    origins = [[0.0, 0.0], [2.0, -1.0]]
    unit = closest_point_to_lines(origins, [_dir(0.0), _dir(np.pi / 2)])
    scaled = closest_point_to_lines(origins, [3.0 * _dir(0.0), 0.1 * _dir(np.pi / 2)])
    np.testing.assert_allclose(unit, scaled, atol=1e-12)


def test_three_dimensional_inputs_are_read_as_their_xy():
    """The view is 2D; a caller holding 3-vectors should not have to slice."""
    point = closest_point_to_lines([[0.0, 0.0, 9.0], [2.0, -1.0, -9.0]],
                                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    np.testing.assert_allclose(point, [2.0, 0.0], atol=1e-12)


# ---------------------------------------------------------------------------
# L1: one line, one vote
# ---------------------------------------------------------------------------

def test_l1_ignores_a_line_that_l2_is_dragged_by():
    """Ten lines through (1, 0) and one badly wrong: only l2 follows it."""
    angles = np.linspace(0.2, 2.9, 10)
    origins = [[1.0, 0.0]] * len(angles) + [[1.0, 3.0]]
    directions = [_dir(a) for a in angles] + [_dir(0.0)]

    l1 = fit_lines(origins, directions, norm="l1")
    l2 = fit_lines(origins, directions, norm="l2")
    np.testing.assert_allclose(l1.point, [1.0, 0.0], atol=1e-3)
    assert abs(l2.point[1]) > 0.3


def test_both_norms_agree_on_lines_that_agree():
    angles = np.linspace(0.2, 2.9, 6)
    origins = [[1.0, 0.5]] * len(angles)
    directions = [_dir(a) for a in angles]
    l1 = fit_lines(origins, directions, norm="l1")
    l2 = fit_lines(origins, directions, norm="l2")
    np.testing.assert_allclose(l1.point, l2.point, atol=1e-6)


def test_an_unknown_norm_is_rejected():
    with pytest.raises(ValueError, match="l2"):
        fit_lines([[0.0, 0.0], [1.0, 0.0]], [_dir(0.0), _dir(1.0)], norm="huber")


def test_the_fit_reports_how_many_lines_it_used():
    fit = fit_lines([[0.0, 0.0], [1.0, 0.0], [2.0, 2.0]],
                    [_dir(0.0), _dir(1.0), np.zeros(2)])
    assert fit.n_lines == 2


# ---------------------------------------------------------------------------
# The uncertainty ellipse
# ---------------------------------------------------------------------------

def test_lines_that_agree_give_a_smaller_ellipse_than_lines_that_scatter():
    angles = np.linspace(0.2, 2.9, 8)

    def scattered(spread):
        return fit_lines([[1.0, spread * (-1) ** k] for k in range(len(angles))],
                         [_dir(a) for a in angles])

    assert np.trace(scattered(0.1).covariance) > np.trace(scattered(0.001).covariance)


def test_lines_that_agree_exactly_leave_no_ellipse_worth_drawing():
    """Nothing missed the point, so there is nothing to size an ellipse from.

    Whether the residuals come out as exactly zero (no covariance at all) or as
    1e-18 (a covariance of 1e-35) depends on the linear algebra underneath, so
    both are accepted; what matters is that nothing visible is drawn.
    """
    angles = np.linspace(0.2, 2.9, 8)
    fit = fit_lines([[1.0, 0.0]] * len(angles), [_dir(a) for a in angles])
    ellipse = covariance_ellipse(fit.covariance)
    assert ellipse is None or float(np.abs(ellipse).max()) < 1e-9


def test_the_ellipse_is_widest_where_the_lines_pin_the_point_least():
    """Lines nearly along x pin y down and leave x free, so the ellipse is long
    in x -- which is exactly the shape a straight tunnel produces."""
    angles = [0.0, 0.05, -0.05, 0.1]
    fit = fit_lines([[1.0, 0.0], [1.0, 0.02], [1.0, -0.02], [1.0, 0.01]],
                    [_dir(a) for a in angles], min_spread=1e-6)
    ellipse = covariance_ellipse(fit.covariance)
    spread = ellipse.max(axis=0) - ellipse.min(axis=0)
    assert spread[0] > 10.0 * spread[1]


def test_the_ellipse_scales_with_sigma():
    angles = np.linspace(0.2, 2.9, 8)
    fit = fit_lines([[1.0, 0.05 * (-1) ** k] for k in range(len(angles))],
                    [_dir(a) for a in angles])
    one = covariance_ellipse(fit.covariance, n_sigma=1.0)
    two = covariance_ellipse(fit.covariance, n_sigma=2.0)
    np.testing.assert_allclose(2.0 * one, two, atol=1e-12)


def test_two_lines_have_no_ellipse():
    """They meet exactly, so nothing about the fit is left over to measure."""
    fit = fit_lines([[0.0, 0.0], [2.0, -1.0]], [_dir(0.0), _dir(np.pi / 2)])
    assert fit.covariance is None
    assert covariance_ellipse(None) is None


def test_lines_meeting_exactly_leave_nothing_to_draw():
    """Zero residual is zero uncertainty: no ellipse, or one too small to see."""
    fit = fit_lines([[1.0, 1.0]] * 4, [_dir(a) for a in (0.0, 0.7, 1.4, 2.1)])
    ellipse = covariance_ellipse(fit.covariance)
    assert ellipse is None or np.max(np.linalg.norm(ellipse, axis=1)) < 1e-9
