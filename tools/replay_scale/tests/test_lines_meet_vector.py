"""Correcting the complementary displacement with the whole meeting point.

``lines_meet_x`` reads one number off the point the degenerate lines agree on
and stretches the odometry's own arrow by it. ``lines_meet_xy`` moves the arrow
*onto* that point: in the frame where the displacement is ``(d, 0)`` the lines
put the robot at ``(d*cx, d*cy)``, so the correction turns the displacement as
well as stretching it. That is the only way a lateral odometry error can be
undone, since no scale can express one.

The geometry is checked here; the number the lines produce is checked in
test_line_meet_scale.py.
"""

import numpy as np
import pytest

from replay_scale.core.estimator import (
    apply_scale_correction,
    apply_similarity_correction,
    line_meet_point,
    reconstruct_with_estimator,
)
from replay_scale.core.model import Frame, ReplayParams


# ---------------------------------------------------------------------------
# The correction, on one displacement
# ---------------------------------------------------------------------------

def test_the_corrected_displacement_is_the_meeting_point_in_metres():
    """A 2 m step whose lines say (0.5, 0.25) really went (1.0, 0.5) m."""
    out = apply_similarity_correction([2.0, 0.0, 0.0], 0.5, 0.25)
    np.testing.assert_allclose(out, [1.0, 0.5, 0.0], atol=1e-12)


def test_the_frame_is_the_displacement_s_own_not_the_map_s():
    """Travelling along +y, a positive cy must point to the robot's left."""
    out = apply_similarity_correction([0.0, 3.0, 0.0], 1.0, 0.25)
    np.testing.assert_allclose(out, [-0.75, 3.0, 0.0], atol=1e-12)


def test_a_zero_lateral_is_the_scale_correction_in_the_plane():
    for t in ([2.0, 0.0, 0.0], [0.0, -1.5, 0.0], [0.7, 1.3, 0.0]):
        np.testing.assert_allclose(apply_similarity_correction(t, 0.6, 0.0),
                                   apply_scale_correction(t, 0.6), atol=1e-12)


def test_it_stretches_by_the_length_of_the_meeting_point():
    """|corrected| / |original| is |(cx, cy)|, not cx."""
    out = apply_similarity_correction([2.0, 0.0, 0.0], 0.6, 0.8)
    assert np.linalg.norm(out) == pytest.approx(2.0 * 1.0)      # |(0.6, 0.8)| = 1


def test_it_turns_the_displacement_by_the_angle_of_the_meeting_point():
    out = apply_similarity_correction([1.0, 0.0, 0.0], 1.0, np.tan(np.deg2rad(11.31)))
    assert np.degrees(np.arctan2(out[1], out[0])) == pytest.approx(11.31, abs=1e-2)


def test_z_is_left_alone_where_the_scale_would_have_scaled_it():
    """The fit is 2D and says nothing about the vertical; the scale is not."""
    assert apply_similarity_correction([2.0, 0.0, 0.9], 0.5, 0.0)[2] == pytest.approx(0.9)
    assert apply_scale_correction([2.0, 0.0, 0.9], 0.5)[2] == pytest.approx(0.45)


def test_a_displacement_with_no_direction_is_returned_unchanged():
    """Nothing to align to, so (cx, cy) has no frame to be read in."""
    np.testing.assert_allclose(
        apply_similarity_correction([0.0, 0.0, 0.4], 0.5, 0.3), [0.0, 0.0, 0.4])


def test_undoing_an_injected_lateral_error_recovers_the_true_displacement():
    """The correction is the exact inverse of the drift simulator's error.

    A drift of alpha along body y turns ``v`` into ``v + alpha*|v|*e_y``. The
    lines then meet at that displacement's own view of the truth, and applying
    it puts the arrow back where it started.
    """
    alpha, v = -0.2, np.array([2.0, 0.0, 0.0])
    drifted = v + alpha * np.linalg.norm(v) * np.array([0.0, 1.0, 0.0])
    # Where the truth sits in the drifted displacement's own normalized frame.
    d = np.linalg.norm(drifted[:2])
    e_x = drifted[:2] / d
    e_y = np.array([-e_x[1], e_x[0]])
    cx, cy = float(v[:2] @ e_x) / d, float(v[:2] @ e_y) / d

    np.testing.assert_allclose(apply_similarity_correction(drifted, cx, cy), v, atol=1e-12)


# ---------------------------------------------------------------------------
# Both coordinates come off the same fit
# ---------------------------------------------------------------------------

def _line(origin, angle_rad):
    return (np.array(origin, dtype=float),
            np.array([np.cos(angle_rad), np.sin(angle_rad)]))


def test_the_point_carries_the_lateral_the_scale_discards():
    lines = [_line([0.8, 0.5], np.pi / 2), _line([0.8, 0.5], 0.0)]
    np.testing.assert_allclose(line_meet_point(lines), [0.8, 0.5], atol=1e-9)


def test_parallel_lines_give_no_point_at_all():
    lines = [_line([0.5, 0.0], np.pi / 2), _line([0.5, 1.0], np.pi / 2)]
    assert line_meet_point(lines) is None


def test_a_point_behind_the_anchor_is_no_point_either():
    lines = [_line([-0.6, 0.3], np.pi / 2), _line([-0.6, -0.2], np.pi / 4)]
    assert line_meet_point(lines) is None


# ---------------------------------------------------------------------------
# What the estimator does with it over a run
# ---------------------------------------------------------------------------

def _frames(n=120, lateral=0.25, turn=0.02):
    """A synthetic run whose degenerate direction turns, so the lines meet.

    The robot really travels 1 m per frame along +x; the complementary odometry
    reports that with a fixed sideways error, which is what the lines can see
    and a scale cannot.
    """
    out = []
    for k in range(n):
        f = Frame()
        f.time = float(k) * 0.1
        f.lidar_prev_stamp = f.time - 0.1
        f.dt_scan = 0.1
        f.degeneracy_detected = True
        f.has_basis = True
        f.basis_size = 1
        angle = 0.6 + turn * k
        axis = np.zeros(6, dtype=float)
        axis[:3] = [np.cos(angle), np.sin(angle), 0.0]
        f.basis = [axis]
        f.has_complementary = True
        f.complementary_twist = np.array([10.0, 10.0 * lateral, 0, 0, 0, 0], dtype=float)
        f.dt_complementary = 0.1
        f.scale_applied = 1.0
        f.lidar_increment = np.array([10.0, 0, 0, 0, 0, 0], dtype=float)
        f.pose_prev = np.eye(4)
        pose = np.eye(4)
        pose[0, 3] = float(k + 1)
        f.pose_optimized = pose
        f.pose_effective = pose
        out.append(f)
    return out


def _params(correction, **overrides):
    fields = dict(
        complementary_correction=correction,
        scale_baseline_frame_lag=5,
        scale_min_nondegenerate_speed=0.0,
        scale_smoothing_window_size=20,
        scale_line_history=20,
        scale_line_history_step=1,
    )
    fields.update(overrides)
    return ReplayParams(**fields)


def _trace(correction, **kwargs):
    traj, scale_trace, _ = reconstruct_with_estimator(
        _frames(), _params(correction, **kwargs))
    return traj, scale_trace


def test_the_lateral_is_only_applied_by_the_xy_correction():
    _, x_only = _trace("lines_meet_x")
    _, xy = _trace("lines_meet_xy")

    assert all(np.isnan(s.lateral_applied) for s in x_only)
    assert any(np.isfinite(s.lateral_applied) for s in xy)


def test_both_corrections_read_the_same_along_track_coordinate():
    """Only what is done with the point differs, not where it is."""
    _, x_only = _trace("lines_meet_x")
    _, xy = _trace("lines_meet_xy")
    # The trajectories diverge once the lateral is applied, so compare the first
    # sample each produced -- taken before any correction had been applied.
    first_x = next(s for s in x_only if np.isfinite(s.scale_instant_raw))
    first_xy = next(s for s in xy if np.isfinite(s.scale_instant_raw))
    assert first_x.frame_idx == first_xy.frame_idx
    assert first_xy.scale_instant_raw == pytest.approx(first_x.scale_instant_raw)


def test_applying_the_lateral_changes_the_trajectory():
    x_traj, _ = _trace("lines_meet_x")
    xy_traj, _ = _trace("lines_meet_xy")
    moved = np.linalg.norm(x_traj[-1][1][:3, 3] - xy_traj[-1][1][:3, 3])
    assert moved > 1e-6


def test_the_lateral_bound_clamps_what_is_applied():
    _, unbounded = _trace("lines_meet_xy")
    _, bounded = _trace("lines_meet_xy", scale_lateral_max=0.01)

    applied = np.array([s.lateral_applied for s in bounded], dtype=float)
    applied = applied[np.isfinite(applied)]
    assert applied.size
    assert np.all(np.abs(applied) <= 0.01 + 1e-12)
    # The raw sample is reported as measured either way; only the filter is fed
    # a clamped one.
    raw = np.array([s.lateral_instant_raw for s in bounded], dtype=float)
    raw_unbounded = np.array([s.lateral_instant_raw for s in unbounded], dtype=float)
    assert np.nanmax(np.abs(raw)) > 0.01
    assert np.isfinite(raw_unbounded).sum() > 0


def test_a_frame_without_a_sample_keeps_applying_the_last_pair():
    """The parallel-lines case: hold, rather than fall back to no correction."""
    _, trace = _trace("lines_meet_xy")
    unobservable = [s for s in trace if not s.gate_observable
                    and np.isfinite(s.lateral_applied)]
    assert unobservable, "expected frames that produced no sample of their own"
    for s in unobservable:
        # Applied values come from the window, which such a frame never touched.
        assert np.isfinite(s.scale_applied)


def test_the_ratio_correction_still_reports_where_the_lines_met():
    """The trail the viewer draws does not depend on what is being applied."""
    traj, _, vectors = reconstruct_with_estimator(
        _frames(), _params("ratio"), collect_vectors=True)
    met = [vf for vf in vectors if np.all(np.isfinite(vf.meet_point))]
    assert met
    assert all(np.isnan(vf.lateral_applied) for vf in vectors)


def test_no_line_history_means_no_meeting_point_to_report():
    _, _, vectors = reconstruct_with_estimator(
        _frames(), _params("ratio", scale_line_history=0), collect_vectors=True)
    assert all(not np.any(np.isfinite(vf.meet_point)) for vf in vectors)
