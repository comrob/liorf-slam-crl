"""What the drawer puts on the axes: orientation, normalization, history.

Uses the Agg backend and inspects artists rather than pixels -- and never their
label text. Legends and titles are wording, changed whenever the picture is
explained better, and a test that pins them turns rewording into a failure.
Artists are found by what they are: colour, style, geometry.
"""

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg")

from replay_scale.core.local_view import (  # noqa: E402
    NORMALIZED_EXTENT,
    FrameGeometry,
    build_local_frame_views,
)
from replay_scale.plotting import build_local_frame_figure  # noqa: E402


def _geom(**overrides):
    base = dict(
        frame_idx=3, time=3.0, degeneracy_detected=True, gate_observable=True,
        has_window=True, anchor_frame_idx=1,
        anchor_R=np.eye(3), anchor_p=np.zeros(3),
        latest_R=np.eye(3), latest_p=np.array([3.0, 0.0, 0.0]),
        t_comp_map=np.array([2.0, 0.0, 0.0]),
        degenerate_axes_map=[np.array([0.0, 1.0, 0.0])],
    )
    base.update(overrides)
    return FrameGeometry(**base)


def _view(frame="map", **overrides):
    return build_local_frame_views([_geom(**overrides)], frame=frame,
                                   history=0, history_step=1)[0]


#: matplotlib reports a patch's style as it was given, so both spellings count.
_SOLID = {"solid", "-"}
_DASHED = {"dashed", "--"}


def _arrow(ax, styles):
    """Where the arrow with one of these line styles points, in plot coordinates.

    Two green arrows can be on the axes -- the measured complementary vector
    (solid) and the inferred meeting point (dashed) -- so the style is what
    tells them apart.
    """
    for text in ax.texts:
        patch = getattr(text, "arrow_patch", None)
        if patch is not None and patch.get_linestyle() in styles:
            return np.asarray(text.xy, dtype=float)
    return None


def _comp_arrow(ax):
    """Where the complementary arrow points, in plot coordinates."""
    xy = _arrow(ax, _SOLID)
    if xy is None:
        raise AssertionError("complementary arrow not drawn")
    return xy


def _meeting_arrow(ax):
    """Where the lines-meet arrow points, or None when it was not drawn."""
    return _arrow(ax, _DASHED)


def _screen_xy(view_xy):
    """View-frame (x, y) as the plot's (horizontal, vertical), y to the left."""
    return [float(view_xy[1]), float(view_xy[0])]


def _straight_lines(ax):
    """(horizontal, vertical) point pairs of the dashed/dotted straight lines."""
    out = []
    for line in ax.lines:
        xd, yd = np.asarray(line.get_xdata()), np.asarray(line.get_ydata())
        if len(xd) == 2 and line.get_linestyle() in (":", "--"):
            out.append((xd, yd))
    return out


def _history_lines(ax):
    """How many earlier frames' lines were overlaid: the dotted straight ones."""
    return sum(1 for line in ax.lines
               if line.get_linestyle() == ":" and len(line.get_xdata()) == 2)


def _horizontal_line_heights(ax):
    """Vertical positions of screen-horizontal degenerate lines."""
    return sorted({round(float(yd[0]), 6) for xd, yd in _straight_lines(ax)
                   if abs(yd[0] - yd[1]) < 1e-9})


# ---------------------------------------------------------------------------
# Orientation: x up the page, y to the left
# ---------------------------------------------------------------------------

def test_horizontal_axis_is_reversed_so_y_points_left():
    ax = build_local_frame_figure(_view(), extent=4.0).axes[0]
    lo, hi = ax.get_xlim()
    assert lo > hi, "horizontal axis must be inverted for +y on the left"
    assert ax.get_ylim() == pytest.approx((-4.0, 4.0))


def test_forward_x_is_drawn_upwards():
    ax = build_local_frame_figure(_view(), extent=4.0).axes[0]
    # comp is +x in the view; on screen that is straight up.
    np.testing.assert_allclose(_comp_arrow(ax), [0.0, 2.0], atol=1e-9)


def test_plus_y_is_drawn_towards_the_left():
    view = _view(t_comp_map=np.array([0.0, 2.0, 0.0]))
    ax = build_local_frame_figure(view, extent=4.0).axes[0]
    arrow = _comp_arrow(ax)
    np.testing.assert_allclose(arrow, [2.0, 0.0], atol=1e-9)
    # Positive horizontal data on a reversed axis renders left of the origin.
    assert ax.get_xlim()[0] > arrow[0] > ax.get_xlim()[1]


def _frame_axis_tips(ax):
    """Where the drawn coordinate arrows point, in plot coordinates."""
    return [np.asarray(patch._posA_posB[1], dtype=float) for patch in ax.patches
            if hasattr(patch, "_posA_posB")]


def test_the_frame_axes_are_drawn_up_the_page_and_to_the_left():
    """The convention, drawn: one arrow up for x, one to the left for y."""
    tips = _frame_axis_tips(build_local_frame_figure(_view(), extent=4.0).axes[0])
    assert len(tips) == 2
    up = [t for t in tips if t[1] > 0 and t[0] == 0.0]
    # Positive horizontal on the reversed axis renders to the left.
    left = [t for t in tips if t[0] > 0 and t[1] == 0.0]
    assert len(up) == 1 and len(left) == 1


# ---------------------------------------------------------------------------
# Normalization
# ---------------------------------------------------------------------------

def test_normalized_puts_comp_at_unit_length():
    ax = build_local_frame_figure(_view(), extent=NORMALIZED_EXTENT,
                                  normalize=True).axes[0]
    np.testing.assert_allclose(_comp_arrow(ax), [0.0, 1.0], atol=1e-9)


def test_normalized_degenerate_line_sits_at_the_scale_ratio():
    # |lidar| = 3, |comp| = 2, and the line runs along +y through the latest
    # position, so it lands at 1.5 up the page.
    ax = build_local_frame_figure(_view(), extent=NORMALIZED_EXTENT,
                                  normalize=True).axes[0]
    assert 1.5 in _horizontal_line_heights(ax)


def test_normalize_falls_back_to_metres_without_a_window():
    view = _view(has_window=False, t_comp_map=np.full(3, np.nan))
    ax = build_local_frame_figure(view, extent=4.0, normalize=True).axes[0]
    # In metres, where normalizing would have put the line at 3/|comp|.
    assert 3.0 in _horizontal_line_heights(ax)


# ---------------------------------------------------------------------------
# History placement
# ---------------------------------------------------------------------------

def _two_frames_with_different_comp_lengths():
    """Frame 0: |lidar| 1, |comp| 4.  Frame 1: |lidar| 3, |comp| 2."""
    def geom(k, latest_x, comp_x):
        return FrameGeometry(
            frame_idx=k, time=float(k), degeneracy_detected=True, gate_observable=True,
            has_window=True, anchor_frame_idx=0,
            anchor_R=np.eye(3), anchor_p=np.zeros(3),
            latest_R=np.eye(3), latest_p=np.array([latest_x, 0.0, 0.0]),
            t_comp_map=np.array([comp_x, 0.0, 0.0]),
            degenerate_axes_map=[np.array([0.0, 1.0, 0.0])])
    return [geom(0, 1.0, 4.0), geom(1, 3.0, 2.0)]


def test_history_line_is_normalized_by_its_own_comp_not_the_current_one():
    view = build_local_frame_views(_two_frames_with_different_comp_lengths(),
                                   frame="map", history=1, history_step=1)[1]
    (h,) = view.history_lines
    assert h.comp_norm == pytest.approx(4.0)
    np.testing.assert_allclose(h.own_origin, [1.0, 0.0], atol=1e-12)

    ax = build_local_frame_figure(view, extent=NORMALIZED_EXTENT, normalize=True).axes[0]
    heights = _horizontal_line_heights(ax)
    # Frame 0 normalized by its own |comp| = 4 -> 0.25. The current frame's
    # |comp| = 2 would have put it at 0.5, as would plain re-referencing.
    assert 0.25 in heights
    assert 0.5 not in heights
    assert 1.5 in heights          # the current frame: 3 / 2


def test_history_line_is_re_referenced_when_not_normalized():
    view = build_local_frame_views(_two_frames_with_different_comp_lengths(),
                                   frame="map", history=1, history_step=1)[1]
    heights = _horizontal_line_heights(
        build_local_frame_figure(view, extent=4.0).axes[0])
    assert 1.0 in heights          # frame 0's latest, in metres
    assert 3.0 in heights          # the current frame's latest


def test_unnormalizable_history_lines_are_dropped_when_normalizing():
    geoms = _two_frames_with_different_comp_lengths()
    geoms[0].has_window = False
    geoms[0].t_comp_map = np.full(3, np.nan)
    view = build_local_frame_views(geoms, frame="map", history=1, history_step=1)[1]
    assert view.history_lines[0].comp_norm is None

    ax = build_local_frame_figure(view, extent=NORMALIZED_EXTENT, normalize=True).axes[0]
    assert _history_lines(ax) == 0


# ---------------------------------------------------------------------------
# Where the lines meet
# ---------------------------------------------------------------------------

def _crossing_frames():
    """Two frames whose degenerate lines cross at a known point.

    Frame 0 is degenerate along +y through (0, 0); frame 1 along +x through
    (2, 3). Sharing an anchor at the origin, those lines cross at (0, 3).
    """
    def geom(idx, latest, axis):
        return _geom(frame_idx=idx, time=float(idx), anchor_frame_idx=0,
                     anchor_p=np.zeros(3), latest_p=np.array(latest, dtype=float),
                     t_comp_map=np.array([1.0, 0.0, 0.0]),
                     degenerate_axes_map=[np.array(axis, dtype=float)])
    return [geom(0, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
            geom(1, [2.0, 3.0, 0.0], [1.0, 0.0, 0.0])]


def _view_with_history(geoms, **kwargs):
    return build_local_frame_views(geoms, frame="map", history=1, history_step=1,
                                   **kwargs)[1]


def test_the_arrow_points_where_the_lines_meet():
    ax = build_local_frame_figure(_view_with_history(_crossing_frames()),
                                  extent=6.0).axes[0]
    # Plot coordinates are (y, x) with y inverted for the robotics convention.
    np.testing.assert_allclose(_meeting_arrow(ax), _screen_xy([0.0, 3.0]), atol=1e-9)


def test_the_meeting_arrow_starts_at_the_anchor():
    """It is a displacement from the anchor, like the complementary one."""
    ax = build_local_frame_figure(_view_with_history(_crossing_frames()),
                                  extent=6.0).axes[0]
    text = next(t for t in ax.texts
                if getattr(t, "arrow_patch", None) is not None
                and t.arrow_patch.get_linestyle() in _DASHED)
    np.testing.assert_allclose(np.asarray(text.xyann, dtype=float), [0.0, 0.0], atol=1e-12)


def test_parallel_lines_draw_no_arrow():
    """A straight tunnel: the lines meet nowhere in particular, so say nothing."""
    geoms = _crossing_frames()
    geoms[1].degenerate_axes_map = [np.array([0.0, 1.0, 0.0])]   # same as frame 0
    ax = build_local_frame_figure(_view_with_history(geoms), extent=6.0).axes[0]
    assert _meeting_arrow(ax) is None


def test_a_single_line_draws_no_arrow():
    ax = build_local_frame_figure(_view(frame="map"), extent=5.0).axes[0]
    assert _meeting_arrow(ax) is None


def test_a_meeting_point_far_outside_the_view_is_not_drawn():
    """An arrow to a point 100 extents away says nothing and jitters wildly."""
    ax = build_local_frame_figure(_view_with_history(_crossing_frames()),
                                  extent=0.01).axes[0]
    assert _meeting_arrow(ax) is None


# ---------------------------------------------------------------------------
# The trail of earlier meeting points
# ---------------------------------------------------------------------------

def _meet_trail(ax):
    """The scattered previous meeting points, or None.

    The only scatter on these axes: the ellipse is a filled polygon and every
    other mark is a line.
    """
    return ax.collections[0] if ax.collections else None


def _normalized_view_with_meets(points=((0.9, 0.1), (0.8, -0.2))):
    """Two earlier frames carrying a meeting point, seen from a third."""
    geoms = []
    for k, point in enumerate(points):
        geoms.append(_geom(frame_idx=k, time=float(k), anchor_frame_idx=k,
                           anchor_p=np.array([float(k), 0.0, 0.0]),
                           latest_p=np.array([float(k) + 1.0, 0.0, 0.0]),
                           meet_point=np.array(point, dtype=float)))
    geoms.append(_geom(frame_idx=len(points), time=float(len(points)),
                       anchor_frame_idx=len(points),
                       anchor_p=np.array([float(len(points)), 0.0, 0.0]),
                       latest_p=np.array([float(len(points)) + 1.0, 0.0, 0.0])))
    return geoms


def test_the_trail_scatters_one_point_per_earlier_frame():
    geoms = _normalized_view_with_meets()
    view = build_local_frame_views(geoms, frame="comp", history=2, history_step=1)[2]
    ax = build_local_frame_figure(view, extent=2.0, normalize=True,
                                  meet_history=True).axes[0]
    trail = _meet_trail(ax)
    assert trail is not None
    # Plot coordinates are (y, x), y inverted -- the same convention as the arrow.
    np.testing.assert_allclose(sorted(trail.get_offsets().tolist()),
                               sorted([_screen_xy([0.9, 0.1]), _screen_xy([0.8, -0.2])]),
                               atol=1e-9)


def test_the_trail_is_off_unless_asked_for():
    geoms = _normalized_view_with_meets()
    view = build_local_frame_views(geoms, frame="comp", history=2, history_step=1)[2]
    ax = build_local_frame_figure(view, extent=2.0, normalize=True).axes[0]
    assert _meet_trail(ax) is None


def test_the_trail_is_dropped_where_its_units_would_be_meaningless():
    """Each point is in units of its own frame's |comp|, so only |comp| = 1."""
    geoms = _normalized_view_with_meets()
    view = build_local_frame_views(geoms, frame="comp", history=2, history_step=1)[2]
    metric = build_local_frame_figure(view, extent=5.0, normalize=False,
                                      meet_history=True).axes[0]
    assert _meet_trail(metric) is None

    map_view = build_local_frame_views(geoms, frame="map", history=2,
                                       history_step=1)[2]
    in_map = build_local_frame_figure(map_view, extent=2.0, normalize=True,
                                      meet_history=True).axes[0]
    assert _meet_trail(in_map) is None


def test_frames_that_never_met_contribute_nothing_to_the_trail():
    geoms = _normalized_view_with_meets()
    geoms[0].meet_point = None
    view = build_local_frame_views(geoms, frame="comp", history=2, history_step=1)[2]
    ax = build_local_frame_figure(view, extent=2.0, normalize=True,
                                  meet_history=True).axes[0]
    assert len(_meet_trail(ax).get_offsets()) == 1
