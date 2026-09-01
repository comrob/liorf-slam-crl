"""Gating of the lines' meeting point: when it counts as a sample.

A fitted point is only a scale if the lines actually determined it. Two
independent tests decide that -- the fit's own uncertainty along the scale
axis, and whether the line directions span more than one heading -- and these
cover both, their interaction, and their off switches.
"""

import math

import numpy as np
import pytest

from replay_scale.core.estimator import (
    line_meet_accepted,
    line_meet_fit,
    line_meet_point,
    meet_scale_sigma,
)
from replay_scale.core.lines import leg_split


def _direction(angle_deg):
    theta = math.radians(angle_deg)
    return np.array([math.cos(theta), math.sin(theta)], dtype=float)


def _through(target, angle_deg, offset=0.0):
    """A line at ``angle_deg`` passing ``offset`` to the side of ``target``."""
    theta = math.radians(angle_deg)
    normal = np.array([-math.sin(theta), math.cos(theta)])
    return (np.asarray(target, dtype=float) + offset * normal, _direction(angle_deg))


def _fan(span_deg, *, n=9, noise=0.0, target=(0.9, 0.0), seed=0):
    """``n`` lines through ``target``, spread over ``span_deg`` and jittered.

    The picture the estimator actually sees: every line claims to pass through
    where the robot was, and misses by a little. How well they pin it down is
    then a question of how far their directions spread.
    """
    angles = np.linspace(-span_deg / 2.0, span_deg / 2.0, n)
    offsets = np.random.default_rng(seed).normal(0.0, noise, n)
    return [_through(target, a, o) for a, o in zip(angles, offsets)]


# ---------------------------------------------------------------------------
# leg_split
# ---------------------------------------------------------------------------

def test_two_legs_are_counted_separately():
    dirs = [_direction(a) for a in [20] * 4 + [-20] * 7]
    split = leg_split(dirs, min_separation=math.radians(10))
    assert (split.weaker, split.stronger) == (4, 7)
    assert split.separation == pytest.approx(math.radians(40))


def test_one_heading_has_no_weaker_leg():
    dirs = [_direction(30 + 0.01 * i) for i in range(20)]
    split = leg_split(dirs, min_separation=math.radians(10))
    assert split.weaker == 0
    assert math.isnan(split.separation)


def test_lines_are_undirected():
    """A direction and its opposite are the same line, so they are one group."""
    dirs = [_direction(30) for _ in range(5)]
    dirs += [-d for d in dirs]
    split = leg_split(dirs, min_separation=math.radians(10))
    assert split.weaker == 0


def test_separation_deadband_excludes_the_middle():
    """Half the required separation either side, so legs closer than it vanish."""
    dirs = [_direction(a) for a in [4] * 5 + [-4] * 5]
    assert leg_split(dirs, min_separation=math.radians(6)).weaker == 5
    assert leg_split(dirs, min_separation=math.radians(10)).weaker == 0


def test_separation_is_at_least_what_was_asked_for():
    """The deadband guarantees it, which is why callers need only the counts."""
    rng = np.random.default_rng(3)
    for _ in range(50):
        split = leg_split([_direction(a) for a in rng.uniform(-90.0, 90.0, size=12)],
                          min_separation=math.radians(25))
        if split.weaker:
            assert split.separation >= math.radians(25) - 1e-9


def test_a_continuous_spread_splits_at_its_own_middle():
    """No cluster structure is assumed: what is measured is the span covered."""
    dirs = [_direction(a) for a in np.linspace(-15.0, 15.0, 11)]
    assert leg_split(dirs, min_separation=math.radians(10)).weaker == 4
    assert leg_split(dirs, min_separation=math.radians(40)).weaker == 0


def test_no_directions_gives_a_null_split():
    assert leg_split([], min_separation=0.1).weaker == 0
    assert leg_split([np.zeros(2)], min_separation=0.1).stronger == 0


# ---------------------------------------------------------------------------
# meet_scale_sigma
# ---------------------------------------------------------------------------

def test_sigma_is_zero_when_the_lines_agree_exactly():
    assert meet_scale_sigma(line_meet_fit(_fan(80.0))) == pytest.approx(0.0, abs=1e-9)


def test_sigma_grows_with_the_lines_disagreement():
    tight = meet_scale_sigma(line_meet_fit(_fan(80.0, noise=0.002)))
    loose = meet_scale_sigma(line_meet_fit(_fan(80.0, noise=0.02)))
    assert loose > tight


def test_sigma_grows_as_the_lines_cross_more_shallowly():
    """Same disagreement, less spread in heading: the point is held less firmly."""
    wide = meet_scale_sigma(line_meet_fit(_fan(60.0, noise=0.005)))
    narrow = meet_scale_sigma(line_meet_fit(_fan(8.0, noise=0.005)))
    assert narrow > wide


def test_sigma_is_nan_without_a_covariance():
    """Two lines meet exactly, and exactness there says nothing about where."""
    fit = line_meet_fit([_through((0.9, 0.0), -30), _through((0.9, 0.0), 30)])
    assert fit.covariance is None
    assert math.isnan(meet_scale_sigma(fit))
    assert math.isnan(meet_scale_sigma(None))


# ---------------------------------------------------------------------------
# line_meet_accepted
# ---------------------------------------------------------------------------

def test_ungated_by_default():
    lines = _fan(8.0, noise=0.02)
    assert line_meet_accepted(line_meet_fit(lines), lines)


def test_nothing_is_accepted_without_a_fit():
    assert not line_meet_accepted(None, [])


def test_sigma_gate_rejects_the_point_a_shallow_fan_gets_wrong():
    """The case the gate is for, with the same lines told apart by their spread.

    Identical jitter about the same truth: over 8 degrees of heading it drags
    the answer 13% off and reports a sigma that says so, over 60 it barely moves
    it at all.
    """
    shallow = _fan(8.0, noise=0.02, seed=2)
    wide = _fan(60.0, noise=0.02, seed=2)
    shallow_fit, wide_fit = line_meet_fit(shallow), line_meet_fit(wide)

    assert abs(float(shallow_fit.point[0]) - 0.9) > 0.1
    assert abs(float(wide_fit.point[0]) - 0.9) < 0.03

    assert not line_meet_accepted(shallow_fit, shallow, max_scale_sigma=0.05)
    assert line_meet_accepted(wide_fit, wide, max_scale_sigma=0.05)


def test_sigma_gate_rejects_a_missing_covariance():
    """Too few lines to have missed by anything is not evidence of a good fit."""
    lines = [_through((0.9, 0.0), -30), _through((0.9, 0.0), 30)]
    fit = line_meet_fit(lines)
    assert not line_meet_accepted(fit, lines, max_scale_sigma=0.05)
    assert line_meet_accepted(fit, lines, max_scale_sigma=float("inf"))


def test_leg_gate_rejects_a_single_heading():
    lines = _fan(8.0, noise=0.002)
    fit = line_meet_fit(lines)
    assert not line_meet_accepted(fit, lines, min_leg_lines=3,
                                  min_leg_separation_rad=math.radians(20))
    assert line_meet_accepted(fit, lines, min_leg_lines=0)


def test_leg_gate_accepts_two_legs():
    lines = ([_through((0.9, 0.0), 20 + 0.2 * i) for i in range(4)]
             + [_through((0.9, 0.0), -20 - 0.2 * i) for i in range(4)])
    assert line_meet_accepted(line_meet_fit(lines), lines, min_leg_lines=3,
                              min_leg_separation_rad=math.radians(10))


def test_leg_gate_counts_the_weaker_side():
    """One line from the other leg is not two legs, however many the first has."""
    lines = [_through((0.9, 0.0), 20 + 0.2 * i) for i in range(20)]
    lines.append(_through((0.9, 0.0), -20))
    fit = line_meet_fit(lines)
    assert not line_meet_accepted(fit, lines, min_leg_lines=3,
                                  min_leg_separation_rad=math.radians(10))
    assert line_meet_accepted(fit, lines, min_leg_lines=0)


def test_leg_gate_accepts_lopsided_legs():
    """A window mostly on the current leg still answers, as long as the other
    one is represented: what is counted is the sparser side, not the balance."""
    lines = ([_through((0.9, 0.0), 20 + 0.05 * i) for i in range(20)]
             + [_through((0.9, 0.0), -20 - 0.05 * i) for i in range(5)])
    assert line_meet_accepted(line_meet_fit(lines), lines, min_leg_lines=3,
                              min_leg_separation_rad=math.radians(10))


def test_a_bundle_narrower_than_the_separation_never_passes():
    """The guarantee the deadband buys: no spread, no sample, at any count."""
    rng = np.random.default_rng(11)
    for _ in range(30):
        centre, width = rng.uniform(-90.0, 90.0), rng.uniform(0.0, 9.0)
        dirs = [_direction(centre + w) for w in rng.uniform(-width / 2, width / 2, 40)]
        assert leg_split(dirs, min_separation=math.radians(10)).weaker == 0


def test_leg_gate_catches_what_a_small_sigma_misses():
    """The case the leg rule is for: agreeing closely, crossing shallowly.

    Near-parallel lines that sit almost on top of each other leave tiny
    residuals, so the fit reports a small sigma while barely determining the
    point at all -- moving one line by a centimetre shifts the scale ten times
    as far as it does in a wide fan. Counting the headings catches that, because
    it looks at the conditioning rather than at the residuals.
    """
    narrow, wide = _fan(8.0, noise=2e-4), _fan(80.0, noise=2e-4)
    narrow_fit, wide_fit = line_meet_fit(narrow), line_meet_fit(wide)
    assert meet_scale_sigma(narrow_fit) < 0.05

    def _nudged(lines):
        moved = list(lines)
        moved[0] = (lines[0][0] + 0.01 * np.array([-lines[0][1][1], lines[0][1][0]]),
                    lines[0][1])
        return abs(float(line_meet_fit(moved).point[0]) - float(line_meet_fit(lines).point[0]))

    assert _nudged(narrow) > 5.0 * _nudged(wide)

    assert line_meet_accepted(narrow_fit, narrow, max_scale_sigma=0.05)
    assert not line_meet_accepted(narrow_fit, narrow, max_scale_sigma=0.05,
                                  min_leg_lines=2,
                                  min_leg_separation_rad=math.radians(20))
    assert line_meet_accepted(wide_fit, wide, max_scale_sigma=0.05, min_leg_lines=2,
                              min_leg_separation_rad=math.radians(20))


def test_gates_are_independent():
    """Each can reject on its own; passing one does not excuse failing the other."""
    lines = _fan(60.0, noise=0.02)
    fit = line_meet_fit(lines)
    assert line_meet_accepted(fit, lines, min_leg_lines=2,
                              min_leg_separation_rad=math.radians(10))
    assert not line_meet_accepted(fit, lines, max_scale_sigma=1e-6, min_leg_lines=2,
                                  min_leg_separation_rad=math.radians(10))


def test_a_rejected_point_is_still_a_point():
    """Gating decides whether the fit is used, not whether it exists."""
    lines = _fan(8.0, noise=0.002)
    assert line_meet_point(lines) is not None
    assert not line_meet_accepted(line_meet_fit(lines), lines, min_leg_lines=3,
                                  min_leg_separation_rad=math.radians(20))


# ---------------------------------------------------------------------------
# Wiring: what the estimator does with a rejected point
# ---------------------------------------------------------------------------

def _turning_run(n=60, dt=0.1, turn=0.15, noise=0.01):
    """A robot driving and turning, its odometry over-reporting by 10%.

    The degenerate direction is held fixed in the world -- a wall the scan slides
    along -- so turning swings it relative to the robot's own heading, and it is
    that relative swing, not the turn itself, that fans the normalized lines out
    and lets them meet anywhere.
    """
    from replay_scale.core.model import Frame
    from replay_scale.core.se3 import exp_map, matrix_to_twist

    step = exp_map(np.array([0.2, 0.0, 0.0, 0.0, 0.0, turn]), dt)
    # Jittered, so the lines miss each other by a little and the fit has a
    # residual to form a sigma from: a noiseless run meets exactly, which is a
    # geometry the gate has nothing to say about.
    rng = np.random.default_rng(5)
    frames, T = [], np.eye(4)
    for k in range(n):
        comp = exp_map(np.array([0.22 + rng.normal(0.0, noise), 0.0, 0.0,
                                 0.0, 0.0, turn]), dt)
        # The world's y axis, read in the body frame the basis is logged in.
        wall = T[:3, :3].T @ np.array([0.0, 1.0, 0.0])
        f = Frame()
        f.time = (k + 1) * dt
        f.lidar_prev_stamp = f.time - dt
        f.dt_scan = f.dt_complementary = dt
        f.degeneracy_detected = f.has_basis = f.has_complementary = True
        f.scale_applied = 1.0
        f.basis = [np.concatenate([wall, np.zeros(3)])]
        f.basis_size = 1
        f.lidar_increment = matrix_to_twist(step, 1.0)
        f.complementary_twist = matrix_to_twist(comp, dt)
        f.pose_prev, T = T, T @ step
        f.pose_optimized = f.pose_effective = T
        frames.append(f)
    return frames


def _samples(**gates):
    from replay_scale.core.estimator import reconstruct_with_estimator
    from replay_scale.core.model import ReplayParams

    params = ReplayParams(complementary_correction="lines_meet_x",
                          scale_baseline_frame_lag=5, scale_line_history=40,
                          scale_line_history_step=1, scale_smoothing_window_size=10,
                          scale_min_nondegenerate_speed=0.1, **gates)
    _, trace, vectors = reconstruct_with_estimator(_turning_run(), params,
                                                   collect_vectors=True)
    return trace, vectors


def test_the_estimator_takes_samples_when_the_gates_pass():
    trace, _ = _samples(scale_line_max_scale_sigma=float("inf"), scale_line_min_leg_lines=0)
    assert sum(1 for t in trace if t.gate_observable) > 0


def test_an_unreachable_leg_separation_stops_every_sample():
    trace, _ = _samples(scale_line_max_scale_sigma=float("inf"),
                        scale_line_min_leg_lines=3,
                        scale_line_min_leg_separation_deg=179.0)
    assert not any(t.gate_observable for t in trace)


def test_an_unreachable_sigma_stops_every_sample():
    trace, _ = _samples(scale_line_max_scale_sigma=1e-12, scale_line_min_leg_lines=0)
    assert not any(t.gate_observable for t in trace)


def test_a_rejected_frame_keeps_applying_the_last_agreed_value():
    """No sample is not a scale of 1: the smoothing window is simply not fed."""
    open_trace, _ = _samples(scale_line_max_scale_sigma=float("inf"),
                             scale_line_min_leg_lines=0)
    shut_trace, _ = _samples(scale_line_max_scale_sigma=float("inf"),
                             scale_line_min_leg_lines=3,
                             scale_line_min_leg_separation_deg=179.0)
    answered = [t for t in open_trace if t.gate_observable]
    assert answered and all(math.isnan(t.scale_filtered) for t in shut_trace)
    # The window never fills, so the applied scale stays at the untouched
    # default rather than following a rejected fit.
    assert all(t.scale_applied == pytest.approx(1.0) for t in shut_trace)


def test_the_viewer_still_sees_a_rejected_point():
    """Rejection decides the sample, not the diagnostic: the trail stays drawn."""
    _, vectors = _samples(scale_line_max_scale_sigma=1e-12, scale_line_min_leg_lines=0)
    drawn = [v for v in vectors if np.all(np.isfinite(v.meet_point))]
    assert drawn


# ---------------------------------------------------------------------------
# What is admitted into the line history
# ---------------------------------------------------------------------------

def _vectors(**overrides):
    from replay_scale.core.estimator import reconstruct_with_estimator
    from replay_scale.core.model import ReplayParams

    settings = dict(complementary_correction="lines_meet_x",
                    scale_baseline_frame_lag=5, scale_line_history=40,
                    scale_line_history_step=1, scale_smoothing_window_size=10,
                    scale_min_nondegenerate_speed=0.1,
                    scale_line_max_scale_sigma=float("inf"),
                    scale_line_min_leg_lines=0)
    settings.update(overrides)
    _, _, vectors = reconstruct_with_estimator(_turning_run(),
                                               ReplayParams(**settings),
                                               collect_vectors=True)
    return vectors


def test_frames_below_the_speed_gate_contribute_no_line():
    """A line's angle comes from the LiDAR displacement's direction.

    Below the gate that displacement is short enough to be noise, so the angle
    is arbitrary -- and an arbitrary line is not a weak vote in the fit, it is
    a wrong one. With the gate unreachable nothing is admitted, so there is
    never a second line to meet the current one and no point is ever fitted.
    """
    vectors = _vectors(scale_min_nondegenerate_speed=1e6)
    assert not any(np.all(np.isfinite(v.meet_point)) for v in vectors)


def test_the_speed_gate_alone_admits_a_line_not_the_acceptance_tests():
    """The history cannot be gated on whether its own fit was accepted.

    line_meet_accepted reads the fit that the history feeds, so gating the
    history on it would be circular: an empty history is fewer than two lines,
    which never fits, which is never accepted, which appends nothing -- and the
    estimator would be latched off from the first frame. Rejecting every sample
    must therefore still leave the history filling and points being fitted.
    """
    rejected = _vectors(scale_line_max_scale_sigma=1e-12)
    assert not any(v.gate_observable for v in rejected)
    assert any(np.all(np.isfinite(v.meet_point)) for v in rejected)


def test_the_speed_gate_is_recorded_apart_from_the_narrowed_one():
    """Under a lines-meet correction the two answer different questions.

    gate_observable carries whether the *sample* was taken, which acceptance
    narrows; speed_gate_observable carries whether the frame's *line* was
    admitted. The viewer's overlay filters on the latter, which is what makes
    the meeting point it draws the one the estimate was read from.
    """
    vectors = _vectors(scale_line_max_scale_sigma=1e-12)
    assert any(v.speed_gate_observable for v in vectors)
    assert all(v.speed_gate_observable or not v.gate_observable for v in vectors)
