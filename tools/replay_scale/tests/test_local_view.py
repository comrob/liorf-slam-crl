"""Geometry of the anchor-frame view, on synthetic data. No Qt, no files."""

import numpy as np
import pytest

from replay_scale.core.local_view import (
    EXTENT_LADDER,
    GATE_OBSERVABLE,
    NO_WINDOW,
    WINDOW,
    FrameGeometry,
    build_local_frame_views,
    compute_axis_extent,
    geometry_from_replay,
    snap_extent,
)
from replay_scale.core.model import Frame, ScaleVectorFrame
from replay_scale.core.se3 import orthonormal_translation_basis


def _rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _pose(R=None, p=(0.0, 0.0, 0.0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = p
    return T


def _geometry(**overrides):
    base = dict(
        frame_idx=1, time=1.0, degeneracy_detected=True, gate_observable=False,
        has_window=True, anchor_frame_idx=0,
        anchor_R=np.eye(3), anchor_p=np.zeros(3),
        latest_R=np.eye(3), latest_p=np.array([2.0, 0.0, 0.0]),
        t_comp_map=np.array([1.5, 0.5, 0.0]),
        degenerate_axes_map=[np.array([1.0, 0.0, 0.0])],
    )
    base.update(overrides)
    return FrameGeometry(**base)


def _view(geometry, frame="anchor"):
    return build_local_frame_views([geometry], frame=frame)[0]


# ---------------------------------------------------------------------------
# The anchor frame
# ---------------------------------------------------------------------------

def test_anchor_sits_at_the_origin_and_latest_is_relative_to_it():
    g = _geometry(anchor_p=np.array([10.0, -3.0, 1.0]),
                  latest_p=np.array([12.0, -3.0, 1.0]))
    view = _view(g)
    np.testing.assert_allclose(view.latest_pos, [2.0, 0.0], atol=1e-12)


def test_comp_vector_is_t_comp_map_rotated_into_the_anchor_frame():
    R = _rot_z(np.deg2rad(30.0))
    t_comp = np.array([1.5, 0.5, 0.0])
    view = _view(_geometry(anchor_R=R, t_comp_map=R @ t_comp))
    np.testing.assert_allclose(view.comp_vec, t_comp[:2], atol=1e-12)


def test_view_is_invariant_to_rotating_the_whole_world():
    """Re-expressing every pose in a rotated map frame must not move the view."""
    plain = _view(_geometry())

    R_world = _rot_z(np.deg2rad(57.0))
    g = _geometry()
    rotated = _view(_geometry(
        anchor_R=R_world @ g.anchor_R, anchor_p=R_world @ g.anchor_p,
        latest_R=R_world @ g.latest_R, latest_p=R_world @ g.latest_p,
        t_comp_map=R_world @ g.t_comp_map,
        degenerate_axes_map=[R_world @ a for a in g.degenerate_axes_map]))

    np.testing.assert_allclose(rotated.latest_pos, plain.latest_pos, atol=1e-12)
    np.testing.assert_allclose(rotated.comp_vec, plain.comp_vec, atol=1e-12)
    for a, b in zip(rotated.degenerate_dirs, plain.degenerate_dirs):
        np.testing.assert_allclose(a, b, atol=1e-12)


# ---------------------------------------------------------------------------
# The degenerate line
# ---------------------------------------------------------------------------

def test_degenerate_direction_is_a_unit_vector_in_plane():
    view = _view(_geometry(latest_p=np.array([2.0, 1.0, 0.0]),
                           degenerate_axes_map=[np.array([0.0, 1.0, 0.0])]))
    d, = view.degenerate_dirs
    assert float(np.linalg.norm(d)) == pytest.approx(1.0)
    np.testing.assert_allclose(d, [0.0, 1.0], atol=1e-12)


def test_degenerate_direction_follows_the_axis():
    axis = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
    view = _view(_geometry(degenerate_axes_map=[axis]))
    d, = view.degenerate_dirs
    np.testing.assert_allclose(np.abs(d), np.abs(axis[:2]), atol=1e-12)


def test_near_vertical_axis_is_reported_rather_than_drawn():
    view = _view(_geometry(degenerate_axes_map=[np.array([0.0, 0.0, 1.0])]))
    assert view.degenerate_dirs == []
    assert view.n_out_of_plane_axes == 1
    assert "no in-plane axis" in view.status()


def test_degenerate_line_is_drawn_without_a_complementary_window():
    """The case an alternative odometry source makes the norm, not the exception."""
    view = _view(_geometry(has_window=False, t_comp_map=np.full(3, np.nan)))
    assert view.comp_vec is None
    assert len(view.degenerate_dirs) == 1
    assert view.window_state == NO_WINDOW
    assert "no complementary window" in view.status()


# ---------------------------------------------------------------------------
# Coverage classes and extent
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("has_window,gate,expected", [
    (False, False, NO_WINDOW),
    (True, False, WINDOW),
    (True, True, GATE_OBSERVABLE),
])
def test_window_state_classes(has_window, gate, expected):
    g = _geometry(has_window=has_window, gate_observable=gate)
    assert g.window_state == expected
    assert _view(g).window_state == expected


def test_extent_covers_every_frame_and_respects_the_floor():
    far = _geometry(latest_p=np.array([7.0, 0.0, 0.0]), t_comp_map=np.array([3.0, 0.0, 0.0]))
    assert compute_axis_extent([far], frame="anchor", margin=1.0) == pytest.approx(7.0)
    tiny = _geometry(latest_p=np.zeros(3), t_comp_map=np.zeros(3))
    assert compute_axis_extent([tiny]) == pytest.approx(1.0)


def test_extent_ignores_the_comp_vector_when_there_is_no_window():
    g = _geometry(has_window=False, t_comp_map=np.full(3, np.nan),
                  latest_p=np.array([2.0, 0.0, 0.0]))
    assert np.isfinite(compute_axis_extent([g]))


def test_reach_is_the_farthest_thing_drawn():
    g = _geometry(latest_p=np.array([3.0, 0.0, 0.0]), t_comp_map=np.array([0.5, 0.0, 0.0]))
    assert _view(g).reach == pytest.approx(3.0)
    g = _geometry(latest_p=np.array([0.5, 0.0, 0.0]), t_comp_map=np.array([4.0, 0.0, 0.0]))
    assert _view(g).reach == pytest.approx(4.0)


def test_snap_extent_picks_the_smallest_step_that_fits():
    assert snap_extent(0.1, margin=1.0) == 0.1
    assert snap_extent(0.11, margin=1.0) == 0.25
    assert snap_extent(1.0, margin=1.25) == 2.0     # margin pushes it up a step
    assert snap_extent(0.0) == EXTENT_LADDER[0]


def test_snap_extent_saturates_rather_than_returning_nothing():
    assert snap_extent(1e6) == EXTENT_LADDER[-1]


def test_snap_extent_keeps_the_bimodal_case_readable():
    """A slow frame and a fast frame must not land on the same zoom step."""
    assert snap_extent(0.11) < snap_extent(1.05)


def test_time_is_reported_relative_to_the_first_frame():
    geometries = [_geometry(frame_idx=0, time=1785876000.0),
                  _geometry(frame_idx=1, time=1785876002.5)]
    views = build_local_frame_views(geometries)
    assert [v.time_rel for v in views] == pytest.approx([0.0, 2.5])


# ---------------------------------------------------------------------------
# Reference frame
# ---------------------------------------------------------------------------

def test_map_frame_translates_but_does_not_rotate():
    """The default view: anchor at the origin, axes still map-aligned."""
    R = _rot_z(np.deg2rad(40.0))
    g = _geometry(anchor_R=R, anchor_p=np.array([5.0, 5.0, 0.0]),
                  latest_p=np.array([7.0, 5.0, 0.0]),
                  t_comp_map=np.array([1.5, 0.5, 0.0]),
                  degenerate_axes_map=[np.array([0.0, 1.0, 0.0])])
    view = _view(g, frame="map")
    assert view.frame == "map"
    np.testing.assert_allclose(view.latest_pos, [2.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(view.comp_vec, [1.5, 0.5], atol=1e-12)
    np.testing.assert_allclose(view.degenerate_dirs[0], [0.0, 1.0], atol=1e-12)


def test_map_is_the_default_frame():
    assert build_local_frame_views([_geometry()])[0].frame == "map"


def test_anchor_frame_rotates_relative_to_map_frame():
    R = _rot_z(np.deg2rad(90.0))
    g = _geometry(anchor_R=R, t_comp_map=np.array([0.0, 1.0, 0.0]))
    np.testing.assert_allclose(_view(g, frame="map").comp_vec, [0.0, 1.0], atol=1e-12)
    # +y in map is the anchor's forward once the anchor is yawed 90 degrees.
    np.testing.assert_allclose(_view(g, frame="anchor").comp_vec, [1.0, 0.0], atol=1e-12)


def test_both_frames_agree_when_the_anchor_is_unrotated():
    g = _geometry(anchor_R=np.eye(3))
    np.testing.assert_allclose(_view(g, frame="map").comp_vec,
                               _view(g, frame="anchor").comp_vec, atol=1e-12)


def test_unknown_frame_is_rejected():
    with pytest.raises(ValueError, match="Unknown frame"):
        build_local_frame_views([_geometry()], frame="world")
    with pytest.raises(ValueError, match="Unknown frame"):
        compute_axis_extent([_geometry()], frame="world")


# ---------------------------------------------------------------------------
# Complementary-aligned frame
# ---------------------------------------------------------------------------

def test_comp_frame_turns_the_complementary_vector_onto_plus_x():
    g = _geometry(anchor_p=np.zeros(3),
                  t_comp_map=np.array([0.0, 2.0, 0.0]),
                  latest_p=np.array([0.0, 3.0, 0.0]))
    view = _view(g, frame="comp")
    np.testing.assert_allclose(view.comp_vec, [2.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(view.latest_pos, [3.0, 0.0], atol=1e-12)
    assert view.comp_aligned


def test_comp_frame_preserves_relative_angles():
    """Turning the picture back must not distort it."""
    g = _geometry(anchor_p=np.zeros(3),
                  t_comp_map=np.array([1.0, 1.0, 0.0]),
                  latest_p=np.array([0.0, 2.0, 0.0]))
    plain, aligned = _view(g, frame="map"), _view(g, frame="comp")

    def angle_between(a, b):
        cos = float(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
        return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))

    assert angle_between(plain.comp_vec, plain.latest_pos) == pytest.approx(
        angle_between(aligned.comp_vec, aligned.latest_pos))
    assert float(np.linalg.norm(aligned.latest_pos)) == pytest.approx(
        float(np.linalg.norm(plain.latest_pos)))


def test_comp_frame_is_independent_of_the_anchor_rotation():
    """Aligning to comp wipes out whatever base the rotation started from."""
    g = _geometry(anchor_p=np.zeros(3), t_comp_map=np.array([1.0, 1.0, 0.0]))
    from_map = _view(g, frame="comp")
    g.anchor_R = _rot_z(np.deg2rad(37.0))
    from_anchor = _view(g, frame="comp")
    np.testing.assert_allclose(from_map.comp_vec, from_anchor.comp_vec, atol=1e-12)


def test_comp_frame_falls_back_when_there_is_no_complementary_vector():
    view = _view(_geometry(has_window=False, t_comp_map=np.full(3, np.nan)),
                 frame="comp")
    assert view.comp_aligned is False
    np.testing.assert_allclose(view.latest_pos, [2.0, 0.0], atol=1e-12)


def test_comp_aligned_is_false_in_the_other_frames():
    assert _view(_geometry(), frame="map").comp_aligned is False
    assert _view(_geometry(), frame="anchor").comp_aligned is False


def test_history_lines_are_each_aligned_by_their_own_comp():
    geoms = _history_geometries(n=3)
    geoms[0].t_comp_map = np.array([0.0, 1.0, 0.0])     # frame 0 points +y
    geoms[0].latest_p = np.array([0.0, 2.0, 0.0])
    geoms[0].anchor_p = np.zeros(3)
    v = build_local_frame_views(geoms, frame="comp", history=1, history_step=1)[2]
    (line,) = v.history_lines
    # Frame 0's own displacement was +y and its comp +y, so aligned it is +x.
    np.testing.assert_allclose(line.own_origin, [2.0, 0.0], atol=1e-12)


# ---------------------------------------------------------------------------
# History stride
# ---------------------------------------------------------------------------

def test_history_step_strides_through_earlier_frames():
    views = build_local_frame_views(_history_geometries(n=10), history=3,
                                    history_step=2)
    assert sorted(h.age for h in views[9].history_lines) == [1, 3, 5]


def test_history_step_one_takes_consecutive_frames():
    views = build_local_frame_views(_history_geometries(n=10), history=3,
                                    history_step=1)
    assert sorted(h.age for h in views[9].history_lines) == [1, 2, 3]


def test_history_step_default_is_four():
    from replay_scale.core.local_view import DEFAULT_HISTORY_STEP
    assert DEFAULT_HISTORY_STEP == 4
    views = build_local_frame_views(_history_geometries(n=20), history=2)
    assert sorted(h.age for h in views[19].history_lines) == [1, 5]


def test_history_step_below_one_is_rejected():
    with pytest.raises(ValueError, match="history_step"):
        build_local_frame_views([_geometry()], history=2, history_step=0)


# ---------------------------------------------------------------------------
# Normalization by |comp|
# ---------------------------------------------------------------------------

def test_comp_norm_is_the_complementary_length():
    view = _view(_geometry(t_comp_map=np.array([3.0, 4.0, 0.0])), frame="map")
    assert view.comp_norm == pytest.approx(5.0)


def test_comp_norm_is_none_without_a_window():
    view = _view(_geometry(has_window=False, t_comp_map=np.full(3, np.nan)))
    assert view.comp_norm is None


def test_comp_norm_is_none_when_the_vector_is_degenerate_length():
    view = _view(_geometry(t_comp_map=np.zeros(3)))
    assert view.comp_norm is None


def test_normalizing_puts_lidar_at_the_scale_ratio():
    """The point of the toggle: latest position reads off as |lidar| / |comp|."""
    g = _geometry(latest_p=np.array([3.0, 0.0, 0.0]), anchor_p=np.zeros(3),
                  t_comp_map=np.array([2.0, 0.0, 0.0]))
    view = _view(g, frame="map")
    ratio = float(np.linalg.norm(view.latest_pos)) / view.comp_norm
    assert ratio == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# History overlay
# ---------------------------------------------------------------------------

def _history_geometries(n=6, spacing=1.0):
    """n frames marching along +x, each degenerate along +y, anchor two back."""
    out = []
    for k in range(n):
        out.append(_geometry(
            frame_idx=k, time=float(k), anchor_frame_idx=max(0, k - 2),
            anchor_p=np.array([spacing * max(0, k - 2), 0.0, 0.0]),
            latest_p=np.array([spacing * k, 0.0, 0.0]),
            t_comp_map=np.array([spacing * min(k, 2), 0.0, 0.0]),
            degenerate_axes_map=[np.array([0.0, 1.0, 0.0])]))
    return out


def test_history_overlays_the_requested_number_of_earlier_frames():
    views = build_local_frame_views(_history_geometries(), history=3, history_step=1)
    assert [len(v.history_lines) for v in views] == [0, 1, 2, 3, 3, 3]


def test_history_zero_disables_the_overlay():
    views = build_local_frame_views(_history_geometries(), history=0, history_step=1)
    assert all(v.history_lines == [] for v in views)


def test_history_default_is_fifty():
    from replay_scale.core.local_view import DEFAULT_HISTORY
    assert DEFAULT_HISTORY == 50
    views = build_local_frame_views(_history_geometries(n=100), history=DEFAULT_HISTORY,
                                    history_step=1)
    assert len(views[-1].history_lines) == 50


def test_history_line_is_re_referenced_to_the_current_anchor():
    """An earlier line must be offset by the distance between the two anchors."""
    geoms = _history_geometries(spacing=1.0)
    views = build_local_frame_views(geoms, history=1, history_step=1)
    v = views[4]                      # anchor at x=2, latest at x=4
    (line,) = v.history_lines
    # Frame 3's latest sits at x=3 in map, i.e. x=1 relative to frame 4's anchor.
    np.testing.assert_allclose(line.origin, [1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(v.latest_pos, [2.0, 0.0], atol=1e-12)
    assert line.age == 1


def test_history_ages_count_frames_back():
    views = build_local_frame_views(_history_geometries(), history=3, history_step=1)
    assert sorted(h.age for h in views[5].history_lines) == [1, 2, 3]


def test_history_skips_frames_without_a_degenerate_basis():
    geoms = _history_geometries(n=6)
    for k in (1, 2, 3):
        geoms[k].degenerate_axes_map = []
    views = build_local_frame_views(geoms, history=2, history_step=1)
    # Frame 5 must reach past the gap to frames 4 and 0.
    assert sorted(h.age for h in views[5].history_lines) == [1, 5]


def test_history_carries_its_own_anchor_referenced_origin():
    """own_origin is relative to that frame's own anchor, not the current one."""
    geoms = _history_geometries(spacing=1.0)
    v = build_local_frame_views(geoms, history=1, history_step=1)[4]
    (line,) = v.history_lines
    # Frame 3: anchor at x=1, latest at x=3 -> 2 in its own frame, 1 in frame 4's.
    np.testing.assert_allclose(line.own_origin, [2.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(line.origin, [1.0, 0.0], atol=1e-12)


def test_history_records_its_own_comp_length():
    geoms = _history_geometries(spacing=1.0)
    geoms[3].t_comp_map = np.array([0.0, 3.0, 0.0])
    v = build_local_frame_views(geoms, history=1, history_step=1)[4]
    (line,) = v.history_lines
    assert line.comp_norm == pytest.approx(3.0)


def test_history_comp_norm_is_none_without_a_window():
    geoms = _history_geometries(spacing=1.0)
    geoms[3].has_window = False
    geoms[3].t_comp_map = np.full(3, np.nan)
    v = build_local_frame_views(geoms, history=1, history_step=1)[4]
    assert v.history_lines[0].comp_norm is None


def test_history_can_be_restricted_to_observable_frames():
    geoms = _history_geometries(n=6)
    for k in (3, 4):                       # windowed, but failed the speed gate
        geoms[k].gate_observable = False
    views = build_local_frame_views(geoms, history=2, history_step=1, observable_only=True)
    # Frame 5 must skip 4 and 3 and reach back to 2 and 1.
    assert sorted(h.age for h in views[5].history_lines) == [3, 4]


def test_history_ignores_observability_by_default():
    geoms = _history_geometries(n=6)
    for k in (3, 4):
        geoms[k].gate_observable = False
    views = build_local_frame_views(geoms, history=2, history_step=1)
    assert sorted(h.age for h in views[5].history_lines) == [1, 2]


def test_observable_filter_also_excludes_frames_without_a_window():
    geoms = _history_geometries(n=6)
    geoms[4].has_window = False
    geoms[4].t_comp_map = np.full(3, np.nan)
    views = build_local_frame_views(geoms, history=1, history_step=1, observable_only=True)
    assert [h.age for h in views[5].history_lines] == [2]


def test_current_frame_line_is_drawn_even_when_it_is_not_observable():
    """The filter restricts the overlay, never the frame you are looking at."""
    geoms = _history_geometries(n=3)
    for g in geoms:
        g.gate_observable = False
    views = build_local_frame_views(geoms, history=2, history_step=1, observable_only=True)
    assert len(views[2].degenerate_dirs) == 1
    assert views[2].history_lines == []


def test_history_rotates_with_the_anchor_frame():
    geoms = _history_geometries()
    R = _rot_z(np.deg2rad(90.0))
    for g in geoms:
        g.anchor_R = R
    v = build_local_frame_views(geoms, frame="anchor", history=1, history_step=1)[4]
    (line,) = v.history_lines
    # +x in map becomes -y once the anchor is yawed 90 degrees.
    np.testing.assert_allclose(line.origin, [0.0, -1.0], atol=1e-12)


def test_negative_history_is_rejected():
    with pytest.raises(ValueError, match="history"):
        build_local_frame_views([_geometry()], history=-1)


# ---------------------------------------------------------------------------
# The replay adapter
# ---------------------------------------------------------------------------

def _replay_inputs(n=4, lag=2):
    frames, trajectory, trace = [], [], []
    for k in range(n):
        f = Frame()
        f.time = float(k)
        f.has_basis = True
        f.basis = [np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
        frames.append(f)
        trajectory.append((float(k), _pose(p=(float(k), 0.0, 0.0))))

        vf = ScaleVectorFrame()
        vf.frame_idx = k
        vf.time = float(k)
        vf.degeneracy_detected = True
        vf.gate_observable = k >= lag
        vf.anchor_frame_idx = max(0, k - lag)
        vf.t_comp_map = (np.array([0.9, 0.0, 0.0]) if k >= lag else np.full(3, np.nan))
        vf.scale_instant_raw = vf.scale_smooth = vf.scale_applied = 1.0
        trace.append(vf)
    return frames, trajectory, trace


def test_adapter_uses_the_recorded_anchor_index():
    frames, trajectory, trace = _replay_inputs(n=4, lag=2)
    geometries = geometry_from_replay(frames, trajectory, trace)
    assert [g.anchor_frame_idx for g in geometries] == [0, 0, 0, 1]
    # anchor_p must be the pose at that index, not at the current frame.
    np.testing.assert_allclose(geometries[3].anchor_p, [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(geometries[3].t_lidar_map, [2.0, 0.0, 0.0], atol=1e-12)


def test_adapter_marks_frames_without_a_window():
    frames, trajectory, trace = _replay_inputs(n=4, lag=2)
    geometries = geometry_from_replay(frames, trajectory, trace)
    assert [g.has_window for g in geometries] == [False, False, True, True]


def test_adapter_lifts_the_basis_through_the_latest_rotation():
    frames, trajectory, trace = _replay_inputs(n=2, lag=1)
    R = _rot_z(np.deg2rad(90.0))
    trajectory[1] = (1.0, _pose(R=R, p=(1.0, 0.0, 0.0)))
    geometries = geometry_from_replay(frames, trajectory, trace)
    axis, = geometries[1].degenerate_axes_map
    np.testing.assert_allclose(axis, R @ [1.0, 0.0, 0.0], atol=1e-12)


def test_adapter_rejects_misaligned_inputs():
    frames, trajectory, trace = _replay_inputs(n=4)
    with pytest.raises(ValueError, match="misaligned"):
        geometry_from_replay(frames, trajectory[:-1], trace)


def test_orthonormal_basis_drops_dependent_directions():
    basis = [np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
             np.array([1.0, 0.0, 0.0, 0.0, 0.0, 1.0]),   # linearly dependent linear part
             np.array([0.0, 3.0, 0.0, 0.0, 0.0, 0.0])]
    axes = orthonormal_translation_basis(basis)
    assert len(axes) == 2
    np.testing.assert_allclose(axes[0], [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(axes[1], [0.0, 1.0, 0.0], atol=1e-12)
