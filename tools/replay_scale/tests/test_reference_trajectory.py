"""An external trajectory drawn alongside the replayed ones.

Ground truth, a survey, another system's output. The replay never reads it --
it is drawn, not measured against -- so these are about getting it *onto the
picture* correctly, which for a trajectory logged in its own frame and its own
clock is the whole difficulty.
"""

import dataclasses

import numpy as np
import pytest

from replay_scale.core.model import Frame
from replay_scale.io.tum import write_tum
from replay_scale.pipeline import load_reference_trajectory
from replay_scale.settings import ReferenceTrajectorySettings, ReplayToolSettings


def _frames(t0=1000.0, n=5, dt=0.1):
    """A replay whose chain starts at the origin, facing +x."""
    out = []
    for k in range(n):
        f = Frame()
        f.time = t0 + k * dt
        f.pose_prev = np.eye(4)
        out.append(f)
    return out


def _reference_file(tmp_path, *, start_stamp, offset=(100.0, 50.0, 0.0), n=5, dt=0.1):
    """A trajectory in its own frame and its own clock, as a real one would be."""
    poses = []
    for k in range(n):
        T = np.eye(4)
        T[:3, 3] = np.asarray(offset, dtype=float) + [k * dt, 0.0, 0.0]
        poses.append((start_stamp + k * dt, T))
    path = tmp_path / "ground_truth.tum"
    write_tum(str(path), poses)
    return str(path)


def _settings(**kw):
    return ReplayToolSettings(reference_trajectory=ReferenceTrajectorySettings(**kw))


def test_no_path_draws_nothing():
    assert load_reference_trajectory(_settings(), _frames()) == (None, None)


def test_it_is_moved_onto_the_replays_first_pose(tmp_path):
    """100 m away in its own frame; the shapes cannot be compared until it is not."""
    path = _reference_file(tmp_path, start_stamp=1000.0)
    (_, trajectory), status = load_reference_trajectory(
        _settings(path=path, align="first_pose"), _frames())

    np.testing.assert_allclose(trajectory[0][1][:3, 3], [0.0, 0.0, 0.0], atol=1e-9)
    assert "first stamp" in status


def test_alignment_is_rigid_so_the_shape_it_shows_is_its_own(tmp_path):
    """A least-squares fit would absorb the drift the plot exists to show."""
    path = _reference_file(tmp_path, start_stamp=1000.0)
    (_, trajectory), _ = load_reference_trajectory(
        _settings(path=path, align="first_pose"), _frames())

    def _steps(traj):
        return [np.linalg.norm(b[1][:3, 3] - a[1][:3, 3]) for a, b in zip(traj, traj[1:])]

    raw = _reference_file(tmp_path, start_stamp=1000.0)
    from replay_scale.io.tum import load_tum
    np.testing.assert_allclose(_steps(trajectory), _steps(load_tum(raw)), atol=1e-9)


def test_a_clock_that_does_not_overlap_the_run_uses_its_own_first_pose(tmp_path):
    """The normal case for a ground truth logged separately.

    "Nearest stamp" would pick whichever end of the reference happens to be
    closer -- its *last* pose, for a reference whose stamps start lower than the
    run's -- and anchor the comparison to the wrong end of the trajectory.
    """
    path = _reference_file(tmp_path, start_stamp=0.0)          # run starts at 1000
    (_, trajectory), status = load_reference_trajectory(
        _settings(path=path, align="first_pose"), _frames())

    np.testing.assert_allclose(trajectory[0][1][:3, 3], [0.0, 0.0, 0.0], atol=1e-9)
    assert "do not overlap" in status


def test_align_none_leaves_it_where_it_was(tmp_path):
    path = _reference_file(tmp_path, start_stamp=1000.0)
    (_, trajectory), status = load_reference_trajectory(
        _settings(path=path, align="none"), _frames())

    np.testing.assert_allclose(trajectory[0][1][:3, 3], [100.0, 50.0, 0.0], atol=1e-9)
    assert "not moved" in status


def test_the_label_comes_from_the_filename_unless_given(tmp_path):
    path = _reference_file(tmp_path, start_stamp=1000.0)
    (label, _), _ = load_reference_trajectory(_settings(path=path), _frames())
    assert label == "ground truth"

    (label, _), _ = load_reference_trajectory(
        _settings(path=path, label="RTK"), _frames())
    assert label == "RTK"


def test_a_missing_file_is_an_error_rather_than_a_missing_curve(tmp_path):
    """Silently drawing nothing would read as "the reference agrees"."""
    with pytest.raises(FileNotFoundError):
        load_reference_trajectory(_settings(path=str(tmp_path / "nope.tum")), _frames())


def test_an_empty_file_is_an_error_too(tmp_path):
    path = tmp_path / "empty.tum"
    path.write_text("# nothing here\n")
    with pytest.raises(ValueError):
        load_reference_trajectory(_settings(path=str(path)), _frames())


def test_an_unknown_alignment_is_refused():
    with pytest.raises(ValueError):
        _settings(path="/tmp/x.tum", align="umeyama").validated()


def test_it_reaches_the_plot_as_its_own_kind(tmp_path):
    """Not a replay and not the recorded original -- what those are judged against."""
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from replay_scale.plotting import EXTERNAL_COLOR, build_trajectory_figure
    from replay_scale.pipeline import ReplayResult

    path = _reference_file(tmp_path, start_stamp=1000.0)
    reference, _ = load_reference_trajectory(
        _settings(path=path, align="first_pose"), _frames())
    result = ReplayResult(csv_path="", settings=None, params=None, frames=[],
                          out_dir="", traj_dir="", log_dir="",
                          recorded_effective=[(0.0, np.eye(4))], reference=reference)

    curves = result.curves()
    assert curves[0] == ("ground truth", reference[1], "external")

    fig = build_trajectory_figure(curves)
    drawn = [line for line in fig.axes[0].get_lines()
             if line.get_label().startswith("reference (")]
    assert len(drawn) == 1
    assert drawn[0].get_color() == EXTERNAL_COLOR


# ---------------------------------------------------------------------------
# Fitting a rotation to the replayed trajectory
# ---------------------------------------------------------------------------
#
# A total station tracks a prism: it measures positions and nothing else, so
# the quaternions in its file are placeholders and "anchor the first pose" has
# nothing to anchor. What is unknown between its frame and the run's is where
# each was set up (a position) and which way north it called (a heading), and
# those are what these fit -- from the positions alone.

def _replay(n=60, dt=0.1, t0=1000.0, turn=0.02):
    """A replayed trajectory that curves, so a heading is actually observable."""
    out, heading, p = [], 0.0, np.zeros(3)
    for k in range(n):
        heading += turn
        p = p + [np.cos(heading), np.sin(heading), 0.0]
        T = np.eye(4)
        T[:3, 3] = p
        out.append((t0 + k * dt, T))
    return out


def _surveyed(replay, *, yaw_deg=0.0, pitch_deg=0.0, offset=(500.0, -200.0, 30.0),
              every=3, stamp_shift=0.0):
    """The same path seen from a total station's own frame and clock."""
    cy, sy = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    cp, sp = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    R = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]) @ \
        np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    poses = []
    for k, (stamp, T) in enumerate(replay):
        if k % every:
            continue
        P = np.eye(4)                       # orientation left meaningless, as it is
        P[:3, 3] = R @ T[:3, 3] + np.asarray(offset, dtype=float)
        poses.append((stamp + stamp_shift, P))
    return poses


def _write(tmp_path, poses, name="total_station.tum"):
    path = tmp_path / name
    write_tum(str(path), poses)
    return str(path)


def _positions(trajectory):
    return np.array([T[:3, 3] for _, T in trajectory], dtype=float)


def test_the_fitted_yaw_recovers_the_survey_frames_heading(tmp_path):
    replay = _replay()
    path = _write(tmp_path, _surveyed(replay, yaw_deg=37.0))

    (_, aligned), status = load_reference_trajectory(
        _settings(path=path), _frames(t0=replay[0][0]), replay)

    assert "-37.00 deg" in status                       # undoing +37
    assert "rms 0.000 m" in status
    # Every surveyed point lands back on the path it was surveyed from.
    for stamp, T in aligned:
        match = next(R for t, R in replay if abs(t - stamp) < 1e-9)
        np.testing.assert_allclose(T[:3, 3], match[:3, 3], atol=1e-9)


def test_the_reference_orientation_is_never_read(tmp_path):
    """It is a placeholder in a position-only file; trusting it would tilt the fit."""
    replay = _replay()
    surveyed = _surveyed(replay, yaw_deg=37.0)
    scrambled = [(stamp, T.copy()) for stamp, T in surveyed]
    for _, T in scrambled:
        T[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

    a, _ = load_reference_trajectory(
        _settings(path=_write(tmp_path, surveyed)), _frames(t0=replay[0][0]), replay)
    b, _ = load_reference_trajectory(
        _settings(path=_write(tmp_path, scrambled, "b.tum")),
        _frames(t0=replay[0][0]), replay)

    np.testing.assert_allclose(_positions(a[1]), _positions(b[1]), atol=1e-12)


def test_yaw_only_leaves_a_tilt_visible(tmp_path):
    """Fitting all three would absorb it, and a tilt error is a real finding."""
    replay = _replay()
    path = _write(tmp_path, _surveyed(replay, yaw_deg=10.0, pitch_deg=6.0))
    frames = _frames(t0=replay[0][0])

    _, yaw_status = load_reference_trajectory(_settings(path=path), frames, replay)
    _, full_status = load_reference_trajectory(
        _settings(path=path, align="first_position_rotation"), frames, replay)

    assert "rms 0.000 m" not in yaw_status        # the pitch survives the fit
    assert "rms 0.000 m" in full_status           # and is absorbed by this one


def test_no_scale_is_ever_fitted(tmp_path):
    """The reference keeps its own metric -- a scale is the quantity under test."""
    replay = _replay()
    surveyed = _surveyed(replay, yaw_deg=25.0)
    stretched = [(stamp, T.copy()) for stamp, T in surveyed]
    for _, T in stretched:
        T[:3, 3] *= 1.2

    (_, aligned), _ = load_reference_trajectory(
        _settings(path=_write(tmp_path, stretched)), _frames(t0=replay[0][0]), replay)

    def _span(points):
        return float(np.linalg.norm(points[-1] - points[0]))

    np.testing.assert_allclose(_span(_positions(aligned)),
                               _span(_positions(stretched)), rtol=1e-9)


def test_the_anchor_is_the_first_matched_position(tmp_path):
    replay = _replay()
    path = _write(tmp_path, _surveyed(replay, yaw_deg=37.0))
    (_, aligned), _ = load_reference_trajectory(
        _settings(path=path), _frames(t0=replay[0][0]), replay)

    np.testing.assert_allclose(aligned[0][1][:3, 3], replay[0][1][:3, 3], atol=1e-9)


def test_a_clock_with_no_overlap_says_what_offset_would_fix_it(tmp_path):
    """The normal state of a separately logged reference, and it is recoverable."""
    replay = _replay()
    shift = -replay[0][0]                            # its own clock starts at 0
    path = _write(tmp_path, _surveyed(replay, yaw_deg=37.0, stamp_shift=shift))
    frames = _frames(t0=replay[0][0])

    (_, unfitted), status = load_reference_trajectory(_settings(path=path), frames, replay)
    assert "first position only" in status
    assert "time_offset_s" in status
    # Anchored anyway, so it is still on the picture rather than 500 m away.
    np.testing.assert_allclose(unfitted[0][1][:3, 3], frames[0].pose_prev[:3, 3], atol=1e-9)

    offset = float(status.split("time_offset_s to ")[1].split()[0])
    (_, fitted), status = load_reference_trajectory(
        _settings(path=path, time_offset_s=offset), frames, replay)
    assert "-37.00 deg" in status
    np.testing.assert_allclose(_positions(fitted)[0], replay[0][1][:3, 3], atol=1e-6)


def test_without_a_replay_to_fit_against_it_still_gets_anchored(tmp_path):
    """The plot command can be pointed at a folder with no replay curve in it."""
    replay = _replay()
    path = _write(tmp_path, _surveyed(replay, yaw_deg=37.0))
    frames = _frames(t0=replay[0][0])

    (_, aligned), status = load_reference_trajectory(_settings(path=path), frames, None)
    assert "no replayed trajectory" in status
    np.testing.assert_allclose(aligned[0][1][:3, 3], frames[0].pose_prev[:3, 3], atol=1e-9)


# ---------------------------------------------------------------------------
# Changing it in the viewer, without replaying anything
# ---------------------------------------------------------------------------
#
# The replay never reads the reference, so choosing one must not cost a run of
# the estimator over the whole recording: the viewer refits it against the
# trajectory already loaded and redraws. These pin what that swap does to the
# curve list -- the part a re-run would otherwise have rebuilt from scratch.

def _loaded(curves=None, frames=None):
    """A FrameData with only what reload_reference reads filled in."""
    from replay_scale.gui.sources import FrameData

    frames = frames if frames is not None else _frames()
    return FrameData(csv_path="", views=[], geometries=[], axis_extent=1.0,
                     provenance="", frames=frames,
                     fit_target=[(f.time, f.pose_prev) for f in frames],
                     curves=list(curves if curves is not None
                                 else [("estimated", [], "replay")]))


def test_only_the_reference_changing_is_what_makes_a_redraw_enough():
    """The test the viewer applies before it decides not to replay."""
    from replay_scale.core.model import ReplayParams
    from replay_scale.settings import same_apart_from_run_scoped

    params = ReplayParams()
    loaded = ReplayToolSettings(), params

    assert same_apart_from_run_scoped((_settings(path="/data/gt.tum"), params), loaded)
    assert not same_apart_from_run_scoped(
        (ReplayToolSettings(correction_mode="translation"), params), loaded)
    assert not same_apart_from_run_scoped(
        (ReplayToolSettings(), dataclasses.replace(params, scale_line_history=17)), loaded)


def test_the_reference_is_drawn_first_and_the_replays_are_kept(tmp_path):
    from replay_scale.gui.sources import reload_reference

    data = _loaded()
    path = _reference_file(tmp_path, start_stamp=1000.0)
    curves, status = reload_reference(data, _settings(path=path, align="first_pose"))

    assert [kind for _, _, kind in curves] == ["external", "replay"]
    assert curves[1] == ("estimated", [], "replay")
    assert "ground truth" in curves[0][0] and status


def test_choosing_another_one_replaces_it_rather_than_piling_up(tmp_path):
    from replay_scale.gui.sources import reload_reference

    data = _loaded()
    first = _reference_file(tmp_path, start_stamp=1000.0)
    data.curves, _ = reload_reference(data, _settings(path=first, align="first_pose"))

    other = tmp_path / "other.tum"
    write_tum(str(other), [(1000.0, np.eye(4))])
    curves, _ = reload_reference(data, _settings(path=str(other), align="first_pose"))

    assert [kind for _, _, kind in curves] == ["external", "replay"]
    assert curves[0][0] == "other"


def test_clearing_the_path_takes_it_off_the_plot(tmp_path):
    from replay_scale.gui.sources import reload_reference

    data = _loaded()
    path = _reference_file(tmp_path, start_stamp=1000.0)
    data.curves, _ = reload_reference(data, _settings(path=path, align="first_pose"))

    curves, status = reload_reference(data, _settings())
    assert curves == [("estimated", [], "replay")]
    assert status is None


def test_a_file_that_is_not_there_leaves_the_caller_its_curves(tmp_path):
    """It raises rather than returning empty: the viewer keeps what is drawn."""
    from replay_scale.gui.sources import reload_reference

    data = _loaded()
    with pytest.raises(FileNotFoundError):
        reload_reference(data, _settings(path=str(tmp_path / "gone.tum")))
    assert data.curves == [("estimated", [], "replay")]


def test_the_alignment_is_the_one_a_re_run_would_have_produced(tmp_path):
    """Same inputs as run_replay hands the loader, so the same picture."""
    from replay_scale.gui.sources import reload_reference

    frames = _frames()
    data = _loaded(frames=frames)
    path = _reference_file(tmp_path, start_stamp=1000.0)
    settings = _settings(path=path, align="first_position_yaw")

    curves, _ = reload_reference(data, settings)
    (label, expected), _ = load_reference_trajectory(settings, frames, data.fit_target)

    assert curves[0][0] == label
    np.testing.assert_allclose([T[:3, 3] for _, T in curves[0][1]],
                               [T[:3, 3] for _, T in expected])
