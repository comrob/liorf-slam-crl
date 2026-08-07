"""What the scale tab puts on the axes.

Uses the Agg backend and inspects artists rather than pixels. The y-range is
the part worth pinning: on a real run the raw ratio reaches 60x, and a naive
fit to the data flattens the curves that are actually being read into one line.
"""

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg")

from replay_scale.core.local_view import FrameGeometry  # noqa: E402
from replay_scale.plotting import (  # noqa: E402
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


def _labels(ax):
    return {t.get_text() for t in ax.get_legend().get_texts()}


# ---------------------------------------------------------------------------
# What is drawn
# ---------------------------------------------------------------------------

def test_all_three_series_are_drawn():
    ax = _axes(_geometries())
    assert {"instant raw", "smoothed estimate", "applied"} <= _labels(ax)


def test_the_x_axis_is_time_from_the_start_of_the_run():
    """Frame indices would not line up with anything else the run is read by."""
    ax = _axes(_geometries(n=50))          # 0.1 s apart, starting at t = 0
    line = next(l for l in ax.lines if l.get_label() == "applied")
    xs = line.get_xdata()
    assert xs[0] == pytest.approx(0.0)
    assert xs[-1] == pytest.approx(4.9)


def test_time_is_relative_even_when_stamps_are_absolute():
    geoms = _geometries(n=10)
    for g in geoms:
        g.time += 1_700_000_000.0
    line = next(l for l in _axes(geoms).lines if l.get_label() == "applied")
    assert line.get_xdata()[0] == pytest.approx(0.0)


def test_unobservable_frames_are_shaded_as_one_artist():
    """One fill per run, not one per frame: runs have thousands of frames."""
    geoms = _geometries(n=100)
    for g in geoms[40:60]:
        g.gate_observable = False
    ax = _axes(geoms)
    assert len(ax.collections) == 1
    assert "not observable" in _labels(ax)


def test_configured_bounds_are_drawn_once_each():
    ax = _axes(_geometries(), scale_min=0.5, scale_max=2.0)
    bounds = [l for l in ax.lines if l.get_linestyle() == "--" and l.get_label() == "sample bound"]
    assert len(bounds) == 2
    # One legend entry, not two identical ones.
    assert sum(1 for t in ax.get_legend().get_texts() if t.get_text() == "sample bound") == 1


def test_absent_bounds_draw_nothing():
    ax = _axes(_geometries(), scale_min=0.0, scale_max=float("inf"))
    assert "sample bound" not in _labels(ax)


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
