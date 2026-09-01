"""Running the estimation where the odometry actually is.

The odometry being corrected does not sit on the LiDAR. Between the two origins
is a lever arm, and a body rotation about anything but the LiDAR swings the
LiDAR around it: real motion, present in both displacements the estimate is a
ratio of, but not motion of the odometry sensor. Measured in the LiDAR frame it
is read as odometry error, and then a scale is applied to it -- to a mount whose
length is fixed by the robot's geometry.

``estimationFrame: complementary`` moves the measurement and the correction to
the sensor's own origin, where an in-place rotation has no translation at all.
These pin what that does and, as much as anything, what it must *not* do: the
poses going in and out stay LiDAR poses, and with nothing between the origins
nothing changes.
"""

import numpy as np
import pytest

from replay_scale.core.estimator import (
    build_additional_odom_scale_sample,
    corrected_comp_step,
    reconstruct_with_estimator,
)
from replay_scale.core.model import Frame, ReplayParams
from replay_scale.core.odom_source import apply_complementary_drift
from replay_scale.core.se3 import BodyFrame, adjoint, exp_map, matrix_to_twist, rpy_to_matrix
from replay_scale.settings import estimation_body_frame

#: The ANYmal mount as the runs record it: 0.35 m of lever arm and a 180 deg yaw.
ANYMAL_EXTRINSIC = rpy_to_matrix(-0.310, 0.0, 0.159, 0.0, 0.0, np.pi)

LEVER = BodyFrame(np.array([[1.0, 0, 0, -0.310],
                            [0, 1.0, 0, 0.0],
                            [0, 0, 1.0, 0.159],
                            [0, 0, 0, 1.0]]))


# ---------------------------------------------------------------------------
# The change of frame itself
# ---------------------------------------------------------------------------

def test_an_in_place_rotation_moves_the_lidar_but_not_the_sensor():
    """The whole premise: this displacement is geometry, not odometry error."""
    spin_at_the_sensor = exp_map(np.array([0.0, 0, 0, 0, 0, 0.5]), 1.0)
    at_the_lidar = LEVER.unbase(spin_at_the_sensor)

    assert np.linalg.norm(at_the_lidar[:3, 3]) > 0.15      # 15 cm of "motion"
    np.testing.assert_allclose(LEVER.rebase(at_the_lidar)[:3, 3], 0.0, atol=1e-12)


def test_rebase_and_unbase_are_inverses():
    T = exp_map(np.array([0.3, -0.1, 0.05, 0.02, -0.04, 0.2]), 1.0)
    frame = BodyFrame(ANYMAL_EXTRINSIC)
    np.testing.assert_allclose(frame.unbase(frame.rebase(T)), T, atol=1e-12)
    np.testing.assert_allclose(frame.rebase(frame.unbase(T)), T, atol=1e-12)


def test_a_twist_changes_frame_by_the_adjoint_not_by_a_rotation():
    """A twist's linear part is read at the origin, so the lever arm enters it."""
    xi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5])          # pure spin at the LiDAR
    moved = LEVER.twist(xi)
    np.testing.assert_allclose(moved[3:], xi[3:], atol=1e-12)   # rotation is the same
    np.testing.assert_allclose(moved[:3], -np.cross(LEVER.offset, xi[3:]), atol=1e-12)
    np.testing.assert_allclose(LEVER.untwist(moved), xi, atol=1e-12)


def test_the_adjoint_leaves_a_purely_translational_basis_alone():
    """Which is why the degenerate basis usually does not move at all."""
    for direction in np.eye(3):
        b = np.concatenate([direction, np.zeros(3)])
        np.testing.assert_allclose(LEVER.twist(b), b, atol=1e-12)


def test_the_adjoint_of_the_inverse_undoes_the_adjoint():
    frame = BodyFrame(ANYMAL_EXTRINSIC)
    np.testing.assert_allclose(adjoint(frame.T) @ adjoint(frame.T_inv), np.eye(6), atol=1e-12)


# ---------------------------------------------------------------------------
# What the frame is built from
# ---------------------------------------------------------------------------

def test_the_lidar_frame_ignores_the_extrinsic_entirely():
    frame = estimation_body_frame(ANYMAL_EXTRINSIC, ReplayParams(estimation_frame="lidar"))
    assert frame.is_identity


def test_the_complementary_frame_takes_the_origin_but_not_the_axes():
    frame = estimation_body_frame(ANYMAL_EXTRINSIC,
                                  ReplayParams(estimation_frame="complementary"))
    np.testing.assert_allclose(frame.offset, [-0.310, 0.0, 0.159], atol=1e-12)
    np.testing.assert_allclose(frame.T[:3, :3], np.eye(3), atol=1e-12)


def test_the_axes_come_too_when_asked():
    frame = estimation_body_frame(
        ANYMAL_EXTRINSIC,
        ReplayParams(estimation_frame="complementary", estimation_frame_use_extrinsic_rot=True))
    np.testing.assert_allclose(frame.T, ANYMAL_EXTRINSIC, atol=1e-12)


def test_a_run_with_no_extrinsic_falls_back_to_the_lidar_frame():
    assert estimation_body_frame(
        None, ReplayParams(estimation_frame="complementary")).is_identity


# ---------------------------------------------------------------------------
# The correction applied in that frame
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("scale", [0.5, 1.0, 2.0, 20.0])
@pytest.mark.parametrize("lateral", [None, 0.0, 0.4, -0.4])
def test_no_correction_survives_an_in_place_rotation(scale, lateral):
    """The property the frame exists for, for every correction it could apply.

    The lever arm carrying the LiDAR around the turn is fixed by the robot's
    geometry. Whatever the smoothing window currently believes, the step must
    come back untouched.
    """
    spin = LEVER.unbase(exp_map(np.array([0.0, 0, 0, 0, 0, 0.5]), 1.0))
    np.testing.assert_allclose(
        corrected_comp_step(spin, LEVER, scale, lateral), spin, atol=1e-12)


def test_the_lidar_frame_stretches_that_same_rotation():
    """What it does today, and why the test above is not vacuous."""
    spin = LEVER.unbase(exp_map(np.array([0.0, 0, 0, 0, 0, 0.5]), 1.0))
    stretched = corrected_comp_step(spin, BodyFrame(), 2.0)
    assert np.linalg.norm(stretched[:3, 3]) > 1.9 * np.linalg.norm(spin[:3, 3])


def test_travel_through_the_sensor_origin_is_scaled_as_before():
    """Moving the origin must not stop a real displacement being corrected."""
    step = np.eye(4)
    step[:3, 3] = [1.0, 0.0, 0.0]
    np.testing.assert_allclose(
        corrected_comp_step(step, LEVER, 1.5)[:3, 3], [1.5, 0.0, 0.0], atol=1e-12)


def test_an_identity_frame_is_the_correction_the_node_performs():
    step = exp_map(np.array([0.4, 0.1, 0.0, 0.0, 0.0, 0.05]), 1.0)
    expected = step.copy()
    expected[:3, 3] *= 1.3
    np.testing.assert_allclose(corrected_comp_step(step, BodyFrame(), 1.3), expected, atol=1e-12)


# ---------------------------------------------------------------------------
# End to end through the estimator
# ---------------------------------------------------------------------------

#: The degenerate direction a fixture frame carries, as a twist. Along travel
#: for the correction to substitute into; across it where the test needs the
#: *observable* component to be the one an injected error lands on.
DEGENERATE_X = np.array([1.0, 0, 0, 0, 0, 0])
DEGENERATE_Y = np.array([0, 1.0, 0, 0, 0, 0])


def _frame(step, T_prev, stamp, *, dt=0.1, comp_step=None, basis=DEGENERATE_X):
    """One frame moving the LiDAR by ``step``, with the odometry agreeing.

    The two increments are logged in different units, and mixing them up makes
    a fixture that quietly measures the wrong thing: ``lidar_increment`` is the
    twist of the step over unit time (the reconstruction integrates it with
    ``dt = 1``), while ``complementary_twist`` is a velocity over the frame's
    own interval.
    """
    f = Frame()
    f.time = stamp
    f.lidar_prev_stamp = stamp - dt
    f.dt_scan = dt
    f.dt_complementary = dt
    f.degeneracy_detected = basis is not None
    f.has_basis = basis is not None
    f.has_complementary = True
    f.scale_applied = 1.0
    f.basis = [np.asarray(basis, dtype=float)] if basis is not None else []
    f.basis_size = len(f.basis)
    f.lidar_increment = matrix_to_twist(step, 1.0)
    f.complementary_twist = matrix_to_twist(step if comp_step is None else comp_step, dt)
    T_next = T_prev @ step
    f.pose_prev = T_prev
    f.pose_optimized = T_next
    f.pose_effective = T_next
    return f, T_next


def _run(step, n, dt=0.1, basis=DEGENERATE_X):
    frames = []
    T = np.eye(4)
    for k in range(n):
        f, T = _frame(step, T, (k + 1) * dt, dt=dt, basis=basis)
        frames.append(f)
    return frames


def _spin_in_place_run(n=40, dt=0.1, omega=0.5):
    """A robot turning on the spot about the *sensor* origin, nothing else."""
    return _run(LEVER.unbase(exp_map(np.array([0.0, 0, 0, 0, 0, omega]), dt)), n, dt)


def _params(**kw):
    base = dict(complementary_correction="ratio", scale_baseline_frame_lag=5,
                scale_min_nondegenerate_speed=0.1, scale_smoothing_window_size=10,
                scale_line_history=0, ignore_dz=False)
    base.update(kw)
    return ReplayParams(**base)


def test_a_spin_in_place_produces_no_sample_at_the_sensor():
    """In the LiDAR frame the lever arm sails past the speed gate and is measured."""
    frames = _spin_in_place_run()

    _, lidar_trace, _ = reconstruct_with_estimator(frames, _params())
    _, sensor_trace, _ = reconstruct_with_estimator(frames, _params(), body_frame=LEVER)

    assert any(s.gate_observable for s in lidar_trace)
    assert not any(s.gate_observable for s in sensor_trace)


def test_the_trajectory_is_still_the_lidars():
    """The frame changes what the scale is a scale of, not what is tracked.

    Fixture note: ``dt = 1`` because the node's ``matrixToTwist`` builds its J
    from the total angle while scaling omega by ``1/time``, so a twist only
    round-trips exactly at unit time. This replay is asked to land on the pose
    it was built from, which needs that; the metre-scale claim -- LiDAR poses,
    not sensor poses 0.35 m away -- would survive either way.
    """
    frames = _spin_in_place_run(dt=1.0, omega=0.05)
    traj, _, _ = reconstruct_with_estimator(frames, _params(), body_frame=LEVER)
    for f, (_, pose) in zip(frames, traj):
        np.testing.assert_allclose(pose[:3, 3], f.pose_effective[:3, 3], atol=1e-9)


def test_an_identity_frame_reproduces_the_lidar_frame_replay():
    """Parity: passing a frame at all must not perturb anything by itself."""
    frames = _spin_in_place_run()
    a, trace_a, _ = reconstruct_with_estimator(frames, _params())
    b, trace_b, _ = reconstruct_with_estimator(frames, _params(), body_frame=BodyFrame())
    for (_, pa), (_, pb) in zip(a, b):
        np.testing.assert_array_equal(pa, pb)
    assert [s.scale_instant_raw for s in trace_a] == [s.scale_instant_raw for s in trace_b]


def test_the_window_rotation_is_recorded_and_frame_independent():
    frames = _spin_in_place_run(omega=0.5, dt=0.1)
    _, _, vectors = reconstruct_with_estimator(frames, _params(), collect_vectors=True)
    _, _, at_sensor = reconstruct_with_estimator(
        frames, _params(), collect_vectors=True, body_frame=LEVER)

    turned = [v.window_rotation_rad for v in vectors if np.isfinite(v.window_rotation_rad)]
    assert turned and max(turned) == pytest.approx(0.5 * 0.1 * 5, abs=1e-6)   # omega x dt x lag
    for a, b in zip(vectors, at_sensor):
        assert a.window_rotation_rad == pytest.approx(b.window_rotation_rad, abs=1e-12, nan_ok=True)


# ---------------------------------------------------------------------------
# A known error, injected and recovered in the same frame
# ---------------------------------------------------------------------------

def test_drift_is_injected_along_the_frames_own_axis():
    """The ANYmal yaw is 180 degrees, so the two frames disagree about +x."""
    f = Frame()
    f.time = 0.0
    f.dt_scan = f.dt_complementary = 0.5
    f.has_complementary = True
    f.complementary_twist = np.array([2.0, 0, 0, 0, 0, 0])       # 1 m along lidar x

    turned = BodyFrame(ANYMAL_EXTRINSIC)
    assert apply_complementary_drift([f], 0.1, "x", body_frame=turned) == 1
    # 1 m along the sensor's +x is 1 m along the LiDAR's -x, so the odometry
    # reported 10% *less* far in the frame the estimator will read it in.
    np.testing.assert_allclose(f.complementary_twist[:3] * 0.5, [0.9, 0.0, 0.0], atol=1e-12)


def test_a_pure_scale_error_is_recovered_in_the_frame_it_was_injected_in():
    """Put in 10% too far, get back the 1/1.1 that cancels it."""
    dt, n = 0.1, 120
    # Travelling along x with a slow turn, so the lever arm is in play
    # throughout. The degeneracy is across travel, which is what puts the
    # injected error on the component the ratio is measured from -- along an
    # unobservable axis there would be nothing to recover it from.
    frames = _run(exp_map(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.15]), dt), n, dt,
                  basis=DEGENERATE_Y)
    apply_complementary_drift(frames, 0.1, "x", body_frame=LEVER)

    _, trace, _ = reconstruct_with_estimator(
        frames, _params(scale_baseline_frame_lag=10), body_frame=LEVER)
    accepted = [s.scale_instant_raw for s in trace if s.gate_observable]
    assert len(accepted) > 20
    assert np.median(accepted) == pytest.approx(1.0 / 1.1, rel=0.02)


# ---------------------------------------------------------------------------
# Which odometry is being corrected, and whose mount places the frame
# ---------------------------------------------------------------------------
#
# Two odometries can be configured at once -- the one the node recorded into the
# CSV and an alternative stream replacing it -- and they are different sensors
# on different mounts. Neither mount may stand in for the other: a wrong one
# does not fail, it comes back out as a scale. These pin that they cannot leak.

def _run_dir(tmp_path, meta=True):
    csv_path = tmp_path / "scale_replay_frames.csv"
    csv_path.write_text("")
    if meta:
        (tmp_path / "complementary_odom_meta.yaml").write_text(
            "T_complementary_to_lidar:\n"
            "  translation: [-0.310, 0.0, 0.159]\n"
            "  rotation_quat_xyzw: [0.0, 0.0, 1.0, 0.0]\n")
    return str(csv_path)


VO_MOUNT = np.eye(4)
VO_MOUNT[:3, 3] = [0.05, 0.0, 0.4]


def _settings(*, source=None, recorded=None, **source_kw):
    from replay_scale.settings import (ComplementarySourceSettings,
                                       RecordedOdometrySettings, ReplayToolSettings)
    return ReplayToolSettings(
        recorded_odometry=RecordedOdometrySettings(extrinsic=recorded),
        complementary_source=ComplementarySourceSettings(extrinsic=source, **source_kw))


def _resolve(settings, csv_path):
    from replay_scale.pipeline import resolve_active_odometry
    return resolve_active_odometry(settings, csv_path)


def test_the_recorded_odometry_takes_the_mount_the_run_recorded(tmp_path):
    pytest.importorskip("yaml")
    active = _resolve(_settings(), _run_dir(tmp_path))
    assert active.kind == "recorded"
    np.testing.assert_allclose(active.extrinsic[:3, 3], [-0.310, 0.0, 0.159], atol=1e-9)
    assert "complementary_odom_meta" in active.origin


def test_a_configured_recorded_mount_wins_over_the_run_file(tmp_path):
    """For a run whose meta file is missing or wrong."""
    pytest.importorskip("yaml")
    active = _resolve(_settings(recorded=VO_MOUNT), _run_dir(tmp_path))
    np.testing.assert_array_equal(active.extrinsic, VO_MOUNT)
    assert active.origin == "recorded_odometry.extrinsic"


def test_a_sources_mount_is_never_read_for_the_recorded_odometry(tmp_path):
    """The leak this split exists to close.

    A mount configured for a stream that is not being replayed described a
    different sensor. Taking it would place the estimation frame on hardware
    this replay never reads -- silently, since a wrong mount comes back out as
    a scale -- so it is ignored, and said to be ignored.
    """
    pytest.importorskip("yaml")
    active = _resolve(_settings(source=VO_MOUNT), _run_dir(tmp_path))

    assert active.kind == "recorded"
    np.testing.assert_allclose(active.extrinsic[:3, 3], [-0.310, 0.0, 0.159], atol=1e-9)
    assert "ignored" in active.note and "recorded_odometry" in active.note


def test_a_source_uses_its_own_mount(tmp_path):
    active = _resolve(
        _settings(source=VO_MOUNT, path="/tmp/vo.tum"), _run_dir(tmp_path))
    assert active.kind == "source" and active.is_source
    np.testing.assert_array_equal(active.extrinsic, VO_MOUNT)


def test_a_source_may_say_it_shares_the_recorded_sensors_mount(tmp_path):
    """complementary_odom_stream.tum is the recorded sensor's own raw stream.

    Legitimate, and therefore worth being able to say -- but said in the file
    rather than inherited by default, which is the difference between a stated
    assumption and a silent one.
    """
    pytest.importorskip("yaml")
    active = _resolve(_settings(source="run", path="/tmp/stream.tum"), _run_dir(tmp_path))
    np.testing.assert_allclose(active.extrinsic[:3, 3], [-0.310, 0.0, 0.159], atol=1e-9)
    assert "shared with the recorded odometry" in active.origin


def test_a_source_may_declare_itself_already_in_the_lidar_frame(tmp_path):
    active = _resolve(_settings(source="identity", path="/tmp/vo.tum"), _run_dir(tmp_path))
    np.testing.assert_array_equal(active.extrinsic, np.eye(4))


def test_a_source_with_no_mount_is_refused(tmp_path):
    """Not inherited, not defaulted to identity: stated or nothing."""
    with pytest.raises(ValueError) as excinfo:
        _resolve(_settings(path="/tmp/vo.tum"), _run_dir(tmp_path))
    message = str(excinfo.value)
    for fix in ("extrinsicTrans", "'extrinsic: run'", "'extrinsic: identity'"):
        assert fix in message


def test_a_run_without_a_meta_file_falls_back_to_the_lidar_frame(tmp_path):
    active = _resolve(_settings(), _run_dir(tmp_path, meta=False))
    np.testing.assert_array_equal(active.extrinsic, np.eye(4))
    assert "identity" in active.origin


def test_the_recorded_block_may_not_carry_a_sentinel():
    """`run` there would mean "the run's own", which omitting it already means."""
    pytest.importorskip("yaml")
    from replay_scale.settings import replay_tool_settings_from_mapping
    with pytest.raises(ValueError):
        replay_tool_settings_from_mapping({"recorded_odometry": {"extrinsic": "run"}})
