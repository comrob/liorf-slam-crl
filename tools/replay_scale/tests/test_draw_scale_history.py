"""What the scale tab puts on the axes.

Uses the Agg backend and inspects artists rather than pixels -- and never their
label text, which is wording and changes whenever the plot is explained better.
Series are found by the colour constant they are drawn with. The y-range is the
part worth pinning: on a real run the raw ratio reaches 60x, and a naive fit to
the data flattens the curves that are actually being read into one line.
"""

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg")

from replay_scale.core.local_view import FrameGeometry  # noqa: E402
from replay_scale.plotting import (  # noqa: E402
    APPLIED_LATERAL_COLOR,
    APPLIED_SCALE_COLOR,
    BOUND_COLOR,
    RAW_LATERAL_COLOR,
    RAW_SCALE_COLOR,
    SMOOTH_SCALE_COLOR,
    build_scale_history_figure,
    draw_scale_history,
    scale_axis_limits,
)


def _geom(k, *, raw=1.0, smooth=1.0, applied=1.0, observable=True):
    return FrameGeometry(
        frame_idx=k, time=float(k) * 0.1, degeneracy_detected=True,
        gate_observable=observable, has_window=True, anchor_frame_idx=max(0, k - 1),
        anchor_R=np.eye(3), anchor_p=np.zeros(3),
        latest_R=np.eye(3), latest_p=np.array([0.1 * k, 0.0, 0.0]),
        t_comp_map=np.array([0.1, 0.0, 0.0]),
        scale_instant_raw=raw, scale_smooth=smooth, scale_applied=applied)


def _geometries(n=50, **kwargs):
    return [_geom(k, **kwargs) for k in range(n)]


def _axes(geometries, **kwargs):
    fig = build_scale_history_figure(geometries, **kwargs)
    return fig.axes[0]


def _series(ax, color):
    """The drawn curve in this colour, or None."""
    return next((l for l in ax.lines if l.get_color() == color), None)


def _legend_entries(ax, color):
    """How many legend rows are drawn in this colour."""
    return sum(1 for handle in ax.get_legend().legend_handles
               if getattr(handle, "get_color", lambda: None)() == color)


# ---------------------------------------------------------------------------
# What is drawn
# ---------------------------------------------------------------------------

def test_all_three_series_are_drawn():
    """Raw, smoothed and applied: reading them against each other is the plot."""
    ax = _axes(_geometries())
    for color in (RAW_SCALE_COLOR, SMOOTH_SCALE_COLOR, APPLIED_SCALE_COLOR):
        assert _series(ax, color) is not None


def test_the_x_axis_is_time_from_the_start_of_the_run():
    """Frame indices would not line up with anything else the run is read by."""
    ax = _axes(_geometries(n=50))          # 0.1 s apart, starting at t = 0
    xs = _series(ax, APPLIED_SCALE_COLOR).get_xdata()
    assert xs[0] == pytest.approx(0.0)
    assert xs[-1] == pytest.approx(4.9)


def test_time_is_relative_even_when_stamps_are_absolute():
    geoms = _geometries(n=10)
    for g in geoms:
        g.time += 1_700_000_000.0
    line = _series(_axes(geoms), APPLIED_SCALE_COLOR)
    assert line.get_xdata()[0] == pytest.approx(0.0)


def test_unobservable_frames_are_shaded_as_one_artist():
    """One fill per run, not one per frame: runs have thousands of frames."""
    geoms = _geometries(n=100)
    for g in geoms[40:60]:
        g.gate_observable = False
    ax = _axes(geoms)
    assert len(ax.collections) == 1


def _bounds(ax):
    return [l for l in ax.lines if l.get_color() == BOUND_COLOR]


def test_configured_bounds_are_drawn_once_each():
    ax = _axes(_geometries(), scale_min=0.5, scale_max=2.0)
    bounds = _bounds(ax)
    assert len(bounds) == 2
    # One legend row for the pair, not two identical ones.
    assert _legend_entries(ax, BOUND_COLOR) == 1


def test_absent_bounds_draw_nothing():
    ax = _axes(_geometries(), scale_min=0.0, scale_max=float("inf"))
    assert _bounds(ax) == []


# ---------------------------------------------------------------------------
# The y-range
# ---------------------------------------------------------------------------

def test_the_axis_is_linear_by_default():
    """A deviation should read as the number it is; log is opt-in."""
    assert _axes(_geometries()).get_yscale() == "linear"
    assert _axes(_geometries(), log_y=True).get_yscale() == "log"


def test_a_single_wild_raw_sample_does_not_set_the_limits():
    geoms = _geometries(n=100)
    geoms[50].scale_instant_raw = 60.0
    _, hi = scale_axis_limits(
        np.array([g.scale_instant_raw for g in geoms]),
        np.array([g.scale_smooth for g in geoms]),
        np.array([g.scale_applied for g in geoms]))
    assert hi < 10.0


def test_the_bulk_of_the_estimate_is_inside_the_limits():
    """Clipping the raw ratio is fine; clipping the applied curve's bulk is not."""
    geoms = _geometries(n=100)
    for g in geoms[60:]:
        g.scale_smooth = g.scale_applied = 4.0
        g.scale_instant_raw = 90.0
    lo, hi = scale_axis_limits(
        np.array([g.scale_instant_raw for g in geoms]),
        np.array([g.scale_smooth for g in geoms]),
        np.array([g.scale_applied for g in geoms]))
    assert lo <= 1.0 and hi >= 4.0


def test_one_is_always_in_range():
    """Everything is read against 1, so it has to be on screen."""
    lo, hi = scale_axis_limits(*(np.full(10, 5.0) for _ in range(3)))
    assert lo <= 1.0 <= hi


def test_limits_stay_positive_for_a_log_axis():
    lo, _ = scale_axis_limits(np.full(10, 0.001), np.full(10, 0.002),
                              np.full(10, 0.002), log=True)
    assert lo > 0.0


def test_an_all_nan_estimate_still_produces_a_usable_range():
    """A run the odometry never matched: nothing to draw, but no crash either."""
    nan = np.full(10, np.nan)
    lo, hi = scale_axis_limits(nan, nan, nan)
    assert lo < 1.0 < hi


def test_drawing_an_empty_run_is_harmless():
    fig = build_scale_history_figure([])
    assert fig.axes[0].get_yscale() == "linear"


def test_draw_scale_history_clears_the_axes_it_is_given():
    """Reloading a run must not leave the previous one's curves behind."""
    fig = build_scale_history_figure(_geometries(n=10))
    ax = fig.axes[0]
    before = len(ax.lines)
    draw_scale_history(ax, _geometries(n=10))
    assert len(ax.lines) == before


# ---------------------------------------------------------------------------
# The cross-track coordinate, on its own plot
# ---------------------------------------------------------------------------

def _lateral_geometries(n=50, raw=0.2, applied=0.15):
    geoms = _geometries(n)
    for g in geoms:
        g.lateral_instant_raw = raw
        g.lateral_applied = applied
    return geoms


def _lateral_series(ax):
    return [_series(ax, color)
            for color in (RAW_LATERAL_COLOR, APPLIED_LATERAL_COLOR)]


def test_a_run_without_a_lateral_correction_gets_one_plot():
    fig = build_scale_history_figure(_geometries())
    assert len(fig.axes) == 1
    assert _lateral_series(fig.axes[0]) == [None, None]


def test_a_lateral_correction_gets_a_second_plot_under_the_first():
    fig = build_scale_history_figure(_lateral_geometries())
    assert len(fig.axes) == 2
    scale_ax, lateral_ax = fig.axes
    assert _lateral_series(scale_ax) == [None, None]
    assert None not in _lateral_series(lateral_ax)


def test_the_two_plots_share_one_timeline():
    fig = build_scale_history_figure(_lateral_geometries())
    scale_ax, lateral_ax = fig.axes
    assert scale_ax.get_xlim() == pytest.approx(lateral_ax.get_xlim())
    # Only the lower one carries the time label, which is what sharing a
    # timeline means on screen. What it says is wording, not behaviour.
    assert scale_ax.get_xlabel() == ""
    assert lateral_ax.get_xlabel() != ""


def test_the_lateral_range_is_symmetric_about_zero():
    """It is a direction: left and right are the same size of error."""
    geoms = _lateral_geometries(raw=0.3, applied=0.2)
    lateral_ax = build_scale_history_figure(geoms).axes[1]
    lo, hi = lateral_ax.get_ylim()
    assert lo == pytest.approx(-hi)
    assert hi > 0.3


def test_the_lateral_bound_is_drawn_on_both_sides():
    geoms = _lateral_geometries()
    lateral_ax = build_scale_history_figure(geoms, lateral_max=0.25).axes[1]
    heights = sorted(round(float(l.get_ydata()[0]), 6) for l in lateral_ax.lines
                     if len(l.get_ydata()) == 2 and len(set(l.get_ydata())) == 1)
    assert -0.25 in heights and 0.25 in heights


def test_the_lateral_plot_stays_linear_when_the_scale_goes_logarithmic():
    """A signed quantity has no place on a log axis, and it has its own now."""
    fig = build_scale_history_figure(_lateral_geometries(), log_y=True)
    scale_ax, lateral_ax = fig.axes
    assert scale_ax.get_yscale() == "log"
    assert lateral_ax.get_yscale() == "linear"
    assert None not in _lateral_series(lateral_ax)


def test_overlaying_keeps_one_plot_and_lifts_the_floor_below_zero():
    """split=False is for a figure too short for two panels."""
    geoms = _lateral_geometries(raw=-0.3, applied=-0.2)
    ax = build_scale_history_figure(geoms, split=False).axes[0]
    assert None not in _lateral_series(ax)
    assert ax.get_ylim()[0] < 0.0
