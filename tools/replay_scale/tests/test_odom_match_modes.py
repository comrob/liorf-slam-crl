"""Matching an odometry source onto the LiDAR frame stamps.

``nearest`` is node parity and is checked as such. ``interpolate`` exists
because the scale estimate is a *ratio* of a displacement measured between two
LiDAR stamps: quantizing those stamps onto a slower source's sample grid
changes the window the displacement spans, which is an error in the ratio and
not merely in the timing. So these check the window, not just that a twist came
back out.
"""

import numpy as np
import pytest

from replay_scale.core.model import Frame
from replay_scale.core.odom_source import MATCH_MODES, sync_odom_to_frames
from replay_scale.core.se3 import exp_map

IDENTITY = np.eye(4)


def _frame(time, prev_stamp, dt_scan=0.1):
    f = Frame()
    f.time = time
    f.lidar_prev_stamp = prev_stamp
    f.dt_scan = dt_scan
    return f


def _pose(x=0.0, y=0.0, yaw=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = [x, y, 0.0]
    return T


def _straight_stream(rate_hz, speed=1.0, n=11):
    """Constant speed along +x, sampled at ``rate_hz``."""
    step = 1.0 / rate_hz
    return [(k * step, _pose(x=speed * k * step)) for k in range(n)]


def _sync(stream, frames, mode, max_match_dt_s=0.25):
    return sync_odom_to_frames(stream, frames, IDENTITY,
                               max_match_dt_s=max_match_dt_s, match_mode=mode)


# ---------------------------------------------------------------------------
# The window each mode measures over
# ---------------------------------------------------------------------------

def test_nearest_measures_over_the_odom_pair_interval():
    """The 10 Hz LiDAR window is reported as the 5 Hz source's own interval."""
    frames = [_frame(0.25, 0.15)]
    (_, dt), = _sync(_straight_stream(5.0), frames, "nearest")
    assert dt == pytest.approx(0.2)


def test_interpolate_measures_over_the_lidar_interval():
    frames = [_frame(0.25, 0.15)]
    (_, dt), = _sync(_straight_stream(5.0), frames, "interpolate")
    assert dt == pytest.approx(0.1)


def test_interpolate_recovers_the_true_velocity_of_a_constant_speed_source():
    """A source moving at 1 m/s must read as 1 m/s on every frame, off-grid or not."""
    stream = _straight_stream(5.0, speed=1.0)
    frames = [_frame(round(0.05 + 0.1 * k, 6), round(0.1 * k - 0.05, 6))
              for k in range(1, 15)]
    for twist, dt in _sync(stream, frames, "interpolate"):
        assert twist is not None
        np.testing.assert_allclose(twist, [1.0, 0, 0, 0, 0, 0], atol=1e-9)
        assert dt == pytest.approx(0.1)


def test_interpolated_displacement_is_the_window_it_claims():
    """twist x dt is what gets integrated downstream, so it is what must be right."""
    frames = [_frame(0.37, 0.27)]
    (twist, dt), = _sync(_straight_stream(5.0, speed=2.0), frames, "interpolate")
    np.testing.assert_allclose(twist[:3] * dt, [0.2, 0.0, 0.0], atol=1e-9)


# ---------------------------------------------------------------------------
# How the pose between two samples is built
# ---------------------------------------------------------------------------

def test_interpolation_follows_the_arc_not_the_chord():
    """Constant twist between the samples, so a turn interpolates along its arc.

    Interpolating position on a straight line instead would put the halfway pose
    on the chord at (0, 1) -- inside the arc -- and under-report the distance
    travelled on every turning frame.
    """
    turn = [(0.0, _pose(0.0, 0.0, 0.0)), (2.0, _pose(0.0, 2.0, np.pi / 2))]
    # A one-second window, so twist x dt inverts the log exactly: matrix_to_twist
    # and exp_map only agree for unit time (both carry the node's scaling of the
    # left Jacobian, which is parity, not something to work around here).
    frames = [_frame(1.0, 0.0, dt_scan=1.0)]
    (twist, dt), = _sync(turn, frames, "interpolate", max_match_dt_s=2.5)
    assert dt == pytest.approx(1.0)

    # The twist is a screw velocity, so the pose it produces is what to compare;
    # exp_map is what the reconstruction integrates it with downstream.
    T_delta = exp_map(twist, dt)
    np.testing.assert_allclose(T_delta[:3, 3], [np.sqrt(2.0) - 1.0, 1.0, 0.0], atol=1e-9)
    assert np.arctan2(T_delta[1, 0], T_delta[0, 0]) == pytest.approx(np.pi / 4)


def test_a_stamp_on_a_sample_interpolates_to_that_sample():
    """u = 0 and u = 1 must be exact, or every on-grid frame gains a bias."""
    stream = _straight_stream(5.0)
    frames = [_frame(0.4, 0.2)]                  # both stamps are samples
    (twist, dt), = _sync(stream, frames, "interpolate")
    np.testing.assert_allclose(twist[:3] * dt, [0.2, 0.0, 0.0], atol=1e-12)


# ---------------------------------------------------------------------------
# Gates
# ---------------------------------------------------------------------------

def test_interpolate_rejects_a_gap_wider_than_the_gate():
    """max_match_dt_s caps how much trajectory constant twist may stand in for."""
    frames = [_frame(0.25, 0.15)]
    (twist, dt), = _sync(_straight_stream(5.0), frames, "interpolate", max_match_dt_s=0.1)
    assert twist is None
    assert np.isnan(dt)


def test_interpolate_accepts_a_gap_at_the_gate():
    frames = [_frame(0.25, 0.15)]
    (twist, _), = _sync(_straight_stream(5.0), frames, "interpolate", max_match_dt_s=0.2)
    assert twist is not None


def test_stamps_outside_the_stream_are_not_extrapolated():
    """A source covering part of the run replays that part and no more."""
    stream = _straight_stream(5.0, n=6)          # ends at t = 1.0
    frames = [_frame(0.5, 0.4), _frame(1.5, 1.4), _frame(0.05, -0.05)]
    results = _sync(stream, frames, "interpolate")
    assert results[0][0] is not None
    assert results[1][0] is None
    assert results[2][0] is None


def test_unknown_match_mode_is_rejected():
    with pytest.raises(ValueError, match="match_mode"):
        _sync(_straight_stream(5.0), [_frame(0.25, 0.15)], "linear")


def test_both_modes_agree_on_a_source_sampled_at_the_lidar_stamps():
    """No grid error to remove: the modes may only differ when the source is off-grid."""
    stream = [(round(0.1 * k, 6), _pose(x=0.1 * k)) for k in range(11)]
    frames = [_frame(0.5, 0.4)]
    (near, dt_near), = _sync(stream, frames, "nearest")
    (interp, dt_interp), = _sync(stream, frames, "interpolate")
    np.testing.assert_allclose(near, interp, atol=1e-9)
    assert dt_near == pytest.approx(dt_interp)


def test_nearest_stays_the_default():
    """Parity with the node is what an unconfigured replay must reproduce."""
    assert MATCH_MODES[0] == "nearest"
    frames = [_frame(0.25, 0.15)]
    (_, dt), = sync_odom_to_frames(_straight_stream(5.0), frames, IDENTITY)
    assert dt == pytest.approx(0.2)
