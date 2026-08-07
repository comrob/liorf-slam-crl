"""Figure construction for replay trajectories.

Builds and returns matplotlib ``Figure`` objects; it never calls ``show()`` or
``savefig()``. The CLI saves what it gets back, a GUI embeds it in a canvas.

matplotlib is imported lazily so that importing :mod:`replay_scale` -- and
therefore running the replay itself -- does not require it.
"""

import glob
import os

import numpy as np

from .core.estimator import reconstruct_complementary_only
from .io.tum import load_tum

#: Colors cycled through for replay curves, in order.
REPLAY_COLORS = ("tab:orange", "tab:red", "tab:purple", "tab:brown",
                 "tab:pink", "tab:gray", "tab:olive")
REFERENCE_COLOR = "tab:blue"
AUXILIARY_COLOR = "tab:green"

#: Curve kinds accepted by build_trajectory_figure.
KINDS = ("reference", "replay", "auxiliary")


#: Anchor-frame view colors.
ANCHOR_COLOR = "tab:blue"
COMP_COLOR = "tab:green"
DEGENERATE_COLOR = "tab:orange"


def _screen(v):
    """View-frame (x, y) -> plot (horizontal, vertical).

    Robotics convention: x points up the page, y to the left. The horizontal
    axis therefore carries y, and is inverted so +y lands on the left.
    """
    return float(v[1]), float(v[0])


def _xy(trajectory):
    xs = [T[0, 3] for _, T in trajectory]
    ys = [T[1, 3] for _, T in trajectory]
    return xs, ys


def label_from_tum_path(path):
    """Return (label, is_reference) derived from a trajectory_*.tum filename."""
    name = os.path.splitext(os.path.basename(path))[0]
    if name.startswith("trajectory_"):
        name = name[len("trajectory_"):]
    is_reference = name == "recorded_effective"
    if name.startswith("replay_"):
        name = name[len("replay_"):]
    return name.replace("_", " "), is_reference


def curves_from_tum_dir(traj_dir):
    """Load every ``*.tum`` in a trajectories/ folder as plottable curves.

    Returns a list of ``(label, trajectory, kind)``. Raises FileNotFoundError
    when the folder holds no trajectory files.
    """
    tum_paths = sorted(glob.glob(os.path.join(traj_dir, "*.tum")))
    if not tum_paths:
        raise FileNotFoundError(
            f"No .tum files found in {traj_dir}. "
            "Run replay-scale-trajectory first to populate it.")
    curves = []
    for tum_path in tum_paths:
        label, is_reference = label_from_tum_path(tum_path)
        curves.append((label, load_tum(tum_path), "reference" if is_reference else "replay"))
    return curves


def complementary_only_curve(frames, params):
    """The raw complementary odometry integrated on its own, as an aux curve.

    Not written to a file by the replay, so it is recomputed from the frames
    whenever it is plotted.
    """
    traj = reconstruct_complementary_only(
        frames, translation_scale_multiplier=params.translation_scale)
    return ("additional odometry (complementary, raw)", traj, "auxiliary")


def draw_trajectories(ax, curves, *, title=""):
    """Draw the given curves into an existing axes; returns the axes.

    Clears and redraws ``ax``. ``curves`` is a sequence of
    ``(label, trajectory, kind)`` with kind in :data:`KINDS`. Trajectories are
    lists of ``(stamp, 4x4 pose)``.
    """
    ax.clear()

    color_idx = 0
    for label, trajectory, kind in curves:
        xs, ys = _xy(trajectory)
        if kind == "reference":
            ax.plot(xs, ys, label=f"original ({label})", color=REFERENCE_COLOR, linewidth=1.5)
        elif kind == "replay":
            color = REPLAY_COLORS[color_idx % len(REPLAY_COLORS)]
            color_idx += 1
            ax.plot(xs, ys, label=f"replay ({label})", color=color, linewidth=1.5, linestyle="--")
        elif kind == "auxiliary":
            ax.plot(xs, ys, label=label, color=AUXILIARY_COLOR, linewidth=1.0, alpha=0.8)
        else:
            raise ValueError(f"Unknown curve kind {kind!r}; expected one of {KINDS}")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", linewidth=0.5)
    ax.legend()
    return ax


def build_trajectory_figure(curves, *, title="", figsize=(9, 9)):
    """Top-down XY plot of the given curves; returns the Figure.

    Standalone figure around :func:`draw_trajectories`, for the CLI and for
    headless use; a GUI draws into its own canvas's axes instead.
    """
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=figsize)
    draw_trajectories(ax, curves, title=title)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Anchor-frame vector view
# ---------------------------------------------------------------------------

def draw_local_frame(ax, view, *, extent=None, n_frames=None, normalize=False):
    """Draw one :class:`LocalFrameView` into an existing axes.

    Clears and redraws ``ax``. ``extent`` is the half-width of the square view;
    omit it to snap to a zoom step sized for this frame. Limits are always set
    explicitly -- letting matplotlib autoscale makes scrubbing unreadable.

    With ``normalize``, everything is divided by ``|comp_vec|`` so the
    complementary arrow has unit length and the LiDAR displacement can be read
    straight off the axes as a multiple of it. Axes are then dimensionless.
    Frames with no complementary window cannot be normalized and are drawn in
    metres with a note. Applied at draw time, so the view itself is untouched
    and toggling costs a redraw rather than a rebuild.
    """
    from .core.local_view import snap_extent

    divisor = view.comp_norm if normalize else None
    k = 1.0 / divisor if divisor else 1.0
    normalized = divisor is not None

    if extent is None:
        extent = snap_extent(view.reach * k)

    ax.clear()

    ax.axhline(0.0, color="0.85", linewidth=0.8, zorder=0)
    ax.axvline(0.0, color="0.85", linewidth=0.8, zorder=0)

    latest_pos = view.latest_pos * k

    # Extended past the corners so a line always spans the view wherever the
    # latest position sits inside it.
    half_len = 3.0 * extent

    # Older lines first and fainter, so the current one stays legible on top.
    # Normalized, each earlier line is drawn in *its own* normalized geometry so
    # every overlaid frame has its own complementary vector at unit length;
    # frames that had no window cannot be normalized and are dropped.
    if view.history_lines:
        oldest = max(h.age for h in view.history_lines)
        drawn = 0
        for h in sorted(view.history_lines, key=lambda h: -h.age):
            if normalized:
                if h.comp_norm is None:
                    continue
                origin, direction = h.own_origin / h.comp_norm, h.own_direction
            else:
                origin, direction = h.origin, h.direction
            fade = 1.0 - (h.age / (oldest + 1))
            sx, sy = _screen(origin - half_len * direction), _screen(origin + half_len * direction)
            ax.plot([sx[0], sy[0]], [sx[1], sy[1]],
                    color=DEGENERATE_COLOR, linewidth=0.9, linestyle=":",
                    alpha=0.15 + 0.35 * fade, zorder=1)
            drawn += 1
        if drawn:
            ax.plot([], [], color=DEGENERATE_COLOR, linewidth=0.9, linestyle=":",
                    alpha=0.4, label=f"previous degenerate ({drawn})")

    for i, d in enumerate(view.degenerate_dirs):
        sx, sy = _screen(latest_pos - half_len * d), _screen(latest_pos + half_len * d)
        ax.plot([sx[0], sy[0]], [sx[1], sy[1]],
                color=DEGENERATE_COLOR, linewidth=1.4, linestyle="--", zorder=2,
                label="degenerate direction" if i == 0 else None)

    if view.has_comp_vec:
        ax.annotate("", xy=_screen(view.comp_vec * k), xytext=(0.0, 0.0),
                    arrowprops=dict(arrowstyle="->", color=COMP_COLOR, linewidth=1.8),
                    zorder=3)
        # Proxy artist: annotate() arrows do not appear in the legend.
        ax.plot([], [], color=COMP_COLOR, linewidth=1.8,
                label="complementary (unit)" if normalized else "complementary")

    ax.plot([0.0], [0.0], marker="o", markersize=8, color=ANCHOR_COLOR,
            linestyle="none", zorder=4,
            label=f"anchor (frame {view.anchor_frame_idx})")

    if normalized:
        # The unit circle the complementary arrow now lands on, as a ruler.
        theta = np.linspace(0.0, 2.0 * np.pi, 181)
        ax.plot(np.cos(theta), np.sin(theta), color=COMP_COLOR, linewidth=0.8,
                linestyle=":", alpha=0.5, zorder=0)

    # Horizontal axis reversed so +y is on the left, with x up the page.
    ax.set_xlim(extent, -extent)
    ax.set_ylim(-extent, extent)
    ax.set_aspect("equal", adjustable="box")
    unit = "|comp|" if normalized else "m"
    if view.frame == "anchor":
        forward, frame_note = "anchor forward", "anchor body frame"
    elif view.frame == "comp":
        forward = "complementary forward"
        frame_note = ("complementary-aligned frame" if view.comp_aligned
                      else "complementary-aligned (no vector — map orientation)")
    else:
        forward, frame_note = "map", "map frame, origin at anchor"
    ax.set_ylabel(f"x [{unit}] ({forward})")
    ax.set_xlabel(f"y [{unit}] (left)")
    ax.grid(True, linestyle=":", linewidth=0.5)

    if normalized:
        scale_note = f"  ·  normalized: |comp| = 1 ({divisor:.3f} m)"
    elif normalize:
        scale_note = "  ·  cannot normalize: no complementary window"
    else:
        scale_note = ""

    total = f"/{n_frames - 1}" if n_frames else ""
    flag = " · degenerate" if view.degeneracy_detected else ""
    ax.set_title(f"frame {view.frame_idx}{total}  ·  t +{view.time_rel:.2f} s{flag}\n"
                 f"{frame_note}  ·  view ±{extent:g} {unit}{scale_note}", fontsize="medium")
    ax.legend(loc="upper right", fontsize="small")
    return ax


def build_local_frame_figure(view, *, extent=None, n_frames=None, normalize=False,
                             figsize=(7, 7)):
    """Standalone figure for one frame view; for tests and headless use."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=figsize)
    draw_local_frame(ax, view, extent=extent, n_frames=n_frames, normalize=normalize)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Scale over the run
# ---------------------------------------------------------------------------

#: Scale-history colors. The raw sample is the noisy one and stays in the
#: background; what the trajectory was actually built with is the loudest.
RAW_SCALE_COLOR = "0.6"
SMOOTH_SCALE_COLOR = "tab:green"
APPLIED_SCALE_COLOR = "tab:orange"
UNOBSERVABLE_COLOR = "#d9534f"
BOUND_COLOR = "tab:red"

#: Percentile windows the y-axis is fitted to. The raw ratio reaches 60x on real
#: runs, so it never sets the limits; the estimate is what has to stay readable.
#: A linear axis has no decades to absorb the scatter, so it shows less of it.
_RAW_PERCENTILES_LOG = (5.0, 95.0)
_RAW_PERCENTILES_LINEAR = (5.0, 75.0)
_ESTIMATE_PERCENTILES = (1.0, 99.0)


def draw_scale_history(ax, geometries, *, title="", scale_min=None, scale_max=None,
                       log_y=False):
    """Draw the scale estimate over the run into an existing axes.

    Three curves per frame, all as the estimator produced them: the raw
    instantaneous ratio, the smoothed mean of the observable samples, and what
    the trajectory was actually built with. Frames whose sample failed the
    observability gate are shaded, since that is why the smoothed curve holds
    flat instead of following the raw one.

    The axis is linear, so a deviation reads as the number it is. ``log_y``
    switches to a logarithmic one, which is the axis a *ratio* deserves -- 2 and
    0.5 are the same error in opposite directions -- and which keeps a run whose
    estimate reaches 20x readable near 1. Either way the range is fitted to the
    bulk of the estimate rather than to the raw ratio's excursions.

    ``scale_min``/``scale_max`` draw the configured sample bounds when finite.
    Clears and redraws ``ax``; returns it.
    """
    ax.clear()

    t = np.array([g.time for g in geometries], dtype=float)
    t = t - t[0] if t.size else t
    raw = np.array([g.scale_instant_raw for g in geometries], dtype=float)
    smooth = np.array([g.scale_smooth for g in geometries], dtype=float)
    applied = np.array([g.scale_applied for g in geometries], dtype=float)
    observable = np.array([bool(g.gate_observable) for g in geometries])

    if t.size:
        # One artist rather than a span per frame: runs have thousands of them.
        from matplotlib.transforms import blended_transform_factory

        ax.fill_between(t, 0.0, 1.0, where=~observable, step="mid",
                        transform=blended_transform_factory(ax.transData, ax.transAxes),
                        color=UNOBSERVABLE_COLOR, alpha=0.10, linewidth=0,
                        label="not observable")

    # A scale of 1 is the complementary odometry believed as measured; the whole
    # estimate is a statement about distance from this line.
    ax.axhline(1.0, color="0.45", linewidth=0.9, zorder=0)
    for bound in (scale_min, scale_max):
        if bound is not None and np.isfinite(bound) and bound > 0.0:
            ax.axhline(bound, color=BOUND_COLOR, linewidth=0.9, linestyle="--",
                       alpha=0.7, zorder=0, label="sample bound")

    ax.plot(t, raw, color=RAW_SCALE_COLOR, linewidth=0.7, zorder=1,
            label="instant raw")
    # Applied is the previous frames' mean, so it tracks smoothed almost exactly;
    # dashed on top of solid keeps both readable where they coincide.
    ax.plot(t, smooth, color=SMOOTH_SCALE_COLOR, linewidth=1.6, zorder=2,
            label="smoothed estimate")
    ax.plot(t, applied, color=APPLIED_SCALE_COLOR, linewidth=1.3, linestyle="--",
            zorder=3, label="applied")

    if log_y:
        ax.set_yscale("log")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("scale" + ("  (log)" if log_y else ""))
    ax.set_title(title)
    ax.grid(True, which="both", linestyle=":", linewidth=0.5)
    if t.size:
        ax.set_xlim(float(t[0]), float(t[-1]) if t[-1] > t[0] else float(t[0]) + 1.0)
    ax.set_ylim(*scale_axis_limits(raw, smooth, applied,
                                   bounds=(scale_min, scale_max), log=log_y))
    # Duplicate labels: both bounds share one, and every curve is one artist.
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="upper right", fontsize="small", ncol=2)
    return ax


def scale_axis_limits(raw, smooth, applied, *, bounds=(None, None), log=False):
    """A y-range that keeps the estimate readable, letting the raw ratio clip.

    Fitted to the bulk of each series rather than to its extremes -- one 60x
    sample would otherwise squash the rest flat -- and always including 1.0,
    which is what the curves are read against.
    """
    def percentiles(series, window):
        finite = series[np.isfinite(series)]
        if log:
            finite = finite[finite > 0.0]
        return [float(v) for v in np.percentile(finite, window)] if finite.size else []

    values = [1.0]
    for series in (smooth, applied):
        values += percentiles(series, _ESTIMATE_PERCENTILES)
    values += percentiles(raw, _RAW_PERCENTILES_LOG if log else _RAW_PERCENTILES_LINEAR)
    for bound in bounds:
        if bound is not None and np.isfinite(bound) and bound > 0.0:
            values.append(float(bound))

    lo, hi = min(values), max(values)
    if log:
        # Multiplicative margin, because the spacing itself is multiplicative.
        return max(1e-3, lo / 1.3), hi * 1.3
    if hi - lo < 1e-9:
        lo, hi = lo - 0.5, hi + 0.5
    margin = 0.1 * (hi - lo)
    return max(0.0, lo - margin), hi + margin


def build_scale_history_figure(geometries, *, title="", scale_min=None, scale_max=None,
                               log_y=False, figsize=(9, 5)):
    """Standalone figure of the scale history; for tests and headless use."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=figsize)
    draw_scale_history(ax, geometries, title=title, scale_min=scale_min,
                       scale_max=scale_max, log_y=log_y)
    fig.tight_layout()
    return fig
