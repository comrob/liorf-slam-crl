"""What the drawer puts on the axes: orientation, normalization, history.

Uses the Agg backend and inspects artists rather than pixels.
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


def _comp_arrow(ax):
    """Where the complementary arrow points, in plot coordinates."""
    for text in ax.texts:
        if getattr(text, "xy", None) is not None:
            return np.asarray(text.xy, dtype=float)
    raise AssertionError("complementary arrow not drawn")


def _straight_lines(ax):
    """(horizontal, vertical) point pairs of the dashed/dotted straight lines."""
    out = []
    for line in ax.lines:
        xd, yd = np.asarray(line.get_xdata()), np.asarray(line.get_ydata())
        if len(xd) == 2 and line.get_linestyle() in (":", "--"):
            out.append((xd, yd))
    return out


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


def test_axis_labels_name_the_right_axes():
    ax = build_local_frame_figure(_view(), extent=4.0).axes[0]
    assert ax.get_ylabel().startswith("x [m]")
    assert ax.get_xlabel().startswith("y [m]")


def test_latest_lidar_marker_is_not_drawn():
    ax = build_local_frame_figure(_view(), extent=4.0).axes[0]
    assert not any("latest" in str(l.get_label()).lower() for l in ax.lines)


# ---------------------------------------------------------------------------
# Normalization
# ---------------------------------------------------------------------------

def test_normalized_puts_comp_at_unit_length():
    ax = build_local_frame_figure(_view(), extent=NORMALIZED_EXTENT,
                                  normalize=True).axes[0]
    np.testing.assert_allclose(_comp_arrow(ax), [0.0, 1.0], atol=1e-9)
    assert "|comp|" in ax.get_ylabel()
    assert "normalized" in ax.get_title()


def test_normalized_degenerate_line_sits_at_the_scale_ratio():
    # |lidar| = 3, |comp| = 2, and the line runs along +y through the latest
    # position, so it lands at 1.5 up the page.
    ax = build_local_frame_figure(_view(), extent=NORMALIZED_EXTENT,
                                  normalize=True).axes[0]
    assert 1.5 in _horizontal_line_heights(ax)


def test_normalize_falls_back_to_metres_without_a_window():
    view = _view(has_window=False, t_comp_map=np.full(3, np.nan))
    ax = build_local_frame_figure(view, extent=4.0, normalize=True).axes[0]
    assert "cannot normalize" in ax.get_title()
    assert "[m]" in ax.get_ylabel()
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
    assert not any(str(l.get_label()).startswith("previous degenerate")
                   for l in ax.lines)
