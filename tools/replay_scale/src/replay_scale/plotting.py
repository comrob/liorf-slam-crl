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
#: An external trajectory -- ground truth, a survey, another system's output.
#: Black rather than another hue: it is not one of the things being compared,
#: it is what they are being compared against.
EXTERNAL_COLOR = "black"

#: Curve kinds accepted by build_trajectory_figure.
KINDS = ("reference", "replay", "auxiliary", "external")


#: Anchor-frame view colors.
ANCHOR_COLOR = "tab:blue"
COMP_COLOR = "tab:green"
DEGENERATE_COLOR = "tab:orange"
#: The point the degeneracy spaces agree on. Its own color rather than the
#: complementary arrow's: it is the same kind of quantity -- a displacement from
#: the anchor -- but not a measured one, and telling the inferred point apart
#: from what the odometry claimed is the whole reading. Dashed for the same
#: reason.
MEETING_COLOR = "tab:purple"

#: The view frame's own axes, drawn faint so they read as the ground the rest
#: sits on rather than as another quantity.
AXIS_COLOR = "0.55"

#: How far outside the view the lines may meet before the arrow is dropped, in
#: view half-widths. Near-parallel lines do meet, but hundreds of metres away
#: and wherever noise put them; an arrow to that says nothing and moves wildly.
MEETING_MAX_EXTENTS = 3.0

#: Confidence ellipse drawn around the meeting point, in standard deviations.
ELLIPSE_SIGMA = 1.0

#: The least a cropped view may keep, as a fraction of its full span. Cropping
#: is for cutting away the empty half of a picture, not for emptying it.
MIN_VIEW_SPAN = 0.1

#: Type sizes in the anchor view. Sized to be read in a figure rather than only
#: on the screen it was scrubbed on, so they are larger than matplotlib's
#: defaults and set in one place.
LEGEND_FONTSIZE = "large"
ANNOTATION_FONTSIZE = "large"
AXIS_NAME_FONTSIZE = "medium"


def _screen(v):
    """View-frame (x, y) -> plot (horizontal, vertical).

    Robotics convention: x points up the page, y to the left. The horizontal
    axis therefore carries y, and is inverted so +y lands on the left.
    """
    return float(v[1]), float(v[0])


def _lower_limit(value, extent):
    """The low edge of one axis: ``-extent``, or where the caller cropped it.

    Clamped so a crop can never collapse the view: asking to start above the
    top of it is a slip, and an empty axes is a worse answer than a small one.
    """
    if value is None:
        return -extent
    return min(float(value), extent - MIN_VIEW_SPAN * extent)


def _upper_limit(value, extent):
    """The high edge of one axis: ``extent``, or where the caller cropped it."""
    if value is None:
        return extent
    return max(float(value), -extent + MIN_VIEW_SPAN * extent)


def _draw_frame_axes(ax, *, x_lo, x_hi, y_lo, y_hi):
    """Draw the view frame's axes through the origin: x up the page, y left.

    The convention is in the axis labels too, but a reader following the
    picture should not have to leave it: the arrows say which way the robot
    faces and which way is sideways. Each label sits beside its own arrow --
    the x one clear of the legend in the top right, the y one above the axis
    line rather than down beside the vertical axis's own label, where the two
    would read as a contradiction.

    Drawn only where the origin is actually in view: a cropped view may not
    contain it, and an axis cross pinned to an edge would be a lie about where
    the anchor is.
    """
    from matplotlib.patches import FancyArrowPatch

    if not (x_lo <= 0.0 <= x_hi and y_lo <= 0.0 <= y_hi):
        return

    ax.plot([y_hi, y_lo], [0.0, 0.0], color="0.85", linewidth=0.8, zorder=0)
    ax.plot([0.0, 0.0], [x_lo, x_hi], color="0.85", linewidth=0.8, zorder=0)

    # Each arrow reaches the visible end of its own positive half, which a crop
    # may have moved.
    x_tip, y_tip = 0.97 * x_hi, 0.97 * y_hi
    off = 0.03 * min(x_hi, y_hi)
    # Each label rides beside its own arrow rather than at the tip: the legend
    # sits in the top corner and the vertical axis's own label runs down the
    # left edge, and a name landing on either of those reads as a contradiction.
    # (label, arrow tip, label anchor, alignment) in plot coordinates -- the
    # horizontal axis is reversed, so +y (screen-left) is a positive tip and
    # ha="right" puts a label further to the screen-left of its anchor.
    axes = (("$x$", (0.0, x_tip), (off, 0.55 * x_hi), "right", "center"),
            ("$y$", (y_tip, 0.0), (0.55 * y_hi, off), "center", "bottom"))
    for label, xy, text_xy, ha, va in axes:
        ax.add_patch(FancyArrowPatch((0.0, 0.0), xy, arrowstyle="-|>",
                                     mutation_scale=9, color=AXIS_COLOR,
                                     linewidth=0.8, shrinkA=0.0, shrinkB=0.0,
                                     zorder=0))
        ax.text(*text_xy, label, color=AXIS_COLOR, fontsize=AXIS_NAME_FONTSIZE,
                ha=ha, va=va, zorder=3)


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


def complementary_only_curve(frames, params, body_frame=None):
    """The raw complementary odometry integrated on its own, as an aux curve.

    Not written to a file by the replay, so it is recomputed from the frames
    whenever it is plotted. ``body_frame`` is the replay's estimation frame, so
    that translationScale stretches here what it stretches there.
    """
    traj = reconstruct_complementary_only(
        frames, translation_scale_multiplier=params.translation_scale,
        body_frame=body_frame)
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
        elif kind == "external":
            ax.plot(xs, ys, label=f"reference ({label})", color=EXTERNAL_COLOR,
                    linewidth=1.2, linestyle=(0, (6, 2, 1, 2)), alpha=0.9)
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

def draw_local_frame(ax, view, *, extent=None, n_frames=None, normalize=False,
                     line_fit_norm="l2", meet_history=False, x_min=None, y_max=None):
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

    ``line_fit_norm`` selects how the degenerate lines' meeting point is fitted,
    and should be whatever the estimator is configured with, so the point drawn
    is the one a "lines_meet_*" correction would be read from.

    ``x_min`` and ``y_max`` crop the square view to the two edges worth moving:
    ``x_min`` is the bottom of the page, ``y_max`` the left-hand side, since +y
    is drawn to the left. Both are in view-frame coordinates, in whatever unit
    the axes are currently in, and the other two edges stay at ``extent``. Omit
    them for the symmetric view centred on the anchor.

    ``meet_history`` additionally scatters the *estimator's own* meeting point
    for each frame the overlay looks back at, which is the trail of what the
    correction has been reading. Those points are each in units of their own
    frame's |comp|, so they are only comparable in the normalized
    complementary-aligned view and are silently skipped in any other.
    """
    from .core.lines import covariance_ellipse, fit_lines
    from .core.local_view import snap_extent

    divisor = view.comp_norm if normalize else None
    k = 1.0 / divisor if divisor else 1.0
    normalized = divisor is not None

    if extent is None:
        extent = snap_extent(view.reach * k)

    ax.clear()

    x_lo, y_hi = _lower_limit(x_min, extent), _upper_limit(y_max, extent)
    _draw_frame_axes(ax, x_lo=x_lo, x_hi=extent, y_lo=-extent, y_hi=y_hi)

    latest_pos = view.latest_pos * k

    # Extended past the corners so a line always spans the view wherever the
    # latest position sits inside it.
    half_len = 3.0 * extent

    # Older lines first and fainter, so the current one stays legible on top.
    # Normalized, each earlier line is drawn in *its own* normalized geometry so
    # every overlaid frame has its own complementary vector at unit length;
    # frames that had no window cannot be normalized and are dropped.
    # Collected as they are drawn, so the meeting point below is the one for
    # the lines actually on screen -- normalized or not, history or not.
    line_origins, line_directions = [], []

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
            line_origins.append(origin)
            line_directions.append(direction)
            drawn += 1
        if drawn:
            ax.plot([], [], color=DEGENERATE_COLOR, linewidth=0.9, linestyle=":",
                    alpha=0.4, label=f"previous degeneracy spaces ({drawn})")

    for i, d in enumerate(view.degenerate_dirs):
        sx, sy = _screen(latest_pos - half_len * d), _screen(latest_pos + half_len * d)
        ax.plot([sx[0], sy[0]], [sx[1], sy[1]],
                color=DEGENERATE_COLOR, linewidth=1.4, linestyle="--", zorder=2,
                label="degeneracy space" if i == 0 else None)
        line_origins.append(latest_pos)
        line_directions.append(d)

    # The trail of what the estimator has been reading: one point per earlier
    # frame in the overlay, each fitted from *that* frame's own line window
    # rather than from what is on screen. Scattered rather than joined -- they
    # are independent readings of the same quantity, not a path -- and drawn
    # under the current frame's point, which is the one being applied.
    if meet_history and normalized and view.frame == "comp" and view.comp_aligned:
        points = [(age, p) for age, p in view.meet_history if p is not None]
        if points:
            from matplotlib.colors import to_rgba

            oldest = max(age for age, _ in points)
            xy = np.array([_screen(p) for _, p in points])
            r, g, b, _ = to_rgba(MEETING_COLOR)
            # Per-point RGBA rather than an alpha array: same fading rule as the
            # lines above, without depending on how old the matplotlib is.
            colors = [(r, g, b, 0.15 + 0.45 * (1.0 - age / (oldest + 1)))
                      for age, _ in points]
            ax.scatter(xy[:, 0], xy[:, 1], s=9, marker="o", color=colors,
                       linewidths=0.0, zorder=2,
                       label=f"previous meets ({len(points)})")

    # Each line says the truth lies somewhere along it; where they agree is a
    # position the LiDAR alone could not give, and the arrow to it is directly
    # comparable with the complementary one it is drawn like. Absent when the
    # lines are too parallel to meet anywhere in particular -- a straight
    # stretch of tunnel, where there is genuinely nothing to say.
    fit = fit_lines(line_origins, line_directions, norm=line_fit_norm)
    if fit is not None and np.linalg.norm(fit.point) <= MEETING_MAX_EXTENTS * extent:
        ax.annotate("", xy=_screen(fit.point), xytext=(0.0, 0.0),
                    arrowprops=dict(arrowstyle="->", color=MEETING_COLOR,
                                    linewidth=1.6, linestyle="--"),
                    zorder=3)
        ax.plot(*_screen(fit.point), marker="x", markersize=8, markeredgewidth=1.6,
                color=MEETING_COLOR, linestyle="none", zorder=4)

        # What the point *means*, next to it: in units of the complementary
        # displacement it is (1 + alpha, beta) -- how much longer than the
        # odometry claimed, and how far sideways of its own direction. Those
        # are the picture's own coordinates in the normalized
        # complementary-forward view, and the same two numbers scaled by
        # |comp| in any other.
        ax.annotate(r"$(1+\alpha,\ \beta)$", xy=_screen(fit.point),
                    xytext=(7, 7), textcoords="offset points",
                    color=MEETING_COLOR, fontsize=ANNOTATION_FONTSIZE, zorder=4)

        # How much the lines disagree, drawn where they disagree: a long thin
        # ellipse means one direction is pinned and the other is a guess, which
        # a single point would hide entirely.
        ellipse = covariance_ellipse(fit.covariance, n_sigma=ELLIPSE_SIGMA)
        ellipse = None
        if ellipse is not None:
            screen = np.array([_screen(fit.point + offset) for offset in ellipse])
            # Filled rather than another dotted outline: normalized frames already
            # carry a dotted green unit circle, and two green rings would read as
            # the same kind of thing.
            ax.fill(screen[:, 0], screen[:, 1], color=MEETING_COLOR, alpha=0.18,
                    linewidth=0.0, zorder=2,
                    label=f"{ELLIPSE_SIGMA:g}$\\sigma$ uncertainty")
            ax.plot(screen[:, 0], screen[:, 1], color=MEETING_COLOR, linewidth=0.8,
                    alpha=0.8, zorder=3)

        # annotate() arrows never reach the legend, so a proxy carries the label.
        ax.plot([], [], color=MEETING_COLOR, linewidth=1.6, linestyle="--",
                label=f"estimated actual displacement")

    if view.has_comp_vec:
        ax.annotate("", xy=_screen(view.comp_vec * k), xytext=(0.0, 0.0),
                    arrowprops=dict(arrowstyle="->", color=COMP_COLOR, linewidth=1.8),
                    zorder=3)
        # Proxy artist: annotate() arrows do not appear in the legend.
        ax.plot([], [], color=COMP_COLOR, linewidth=1.8,
                label="complementary odometry displacement (normalized)" if normalized else "complementary odometry displacement")

    ax.plot([0.0], [0.0], marker="o", markersize=8, color=ANCHOR_COLOR,
            linestyle="none", zorder=4,
            label="anchor (previous position)")

    if normalized:
        # The unit circle the complementary arrow now lands on, as a ruler.
        theta = np.linspace(0.0, 2.0 * np.pi, 181)
        ax.plot(np.cos(theta), np.sin(theta), color=COMP_COLOR, linewidth=0.8,
                linestyle=":", alpha=0.5, zorder=0)

    # Horizontal axis reversed so +y is on the left, with x up the page. A crop
    # moves the bottom edge and the left one; the other two stay at the extent.
    ax.set_xlim(y_hi, -extent)
    ax.set_ylim(x_lo, extent)
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
    # The arrows repeat these two lines inside the picture, so the arrow
    # direction is spelled out here as well: nothing should have to be inferred
    # from which of the two names ended up on which side of the axes.
    ax.set_ylabel(f"x [{unit}] ↑ up the page ({forward})")
    ax.set_xlabel(f"y [{unit}] ← to the left")
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
    ax.legend(loc="upper right", fontsize=LEGEND_FONTSIZE)
    return ax


def build_local_frame_figure(view, *, extent=None, n_frames=None, normalize=False,
                             line_fit_norm="l2", meet_history=False, x_min=None,
                             y_max=None, figsize=(7, 7)):
    """Standalone figure for one frame view; for tests and headless use."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=figsize)
    draw_local_frame(ax, view, extent=extent, n_frames=n_frames, normalize=normalize,
                     line_fit_norm=line_fit_norm, meet_history=meet_history,
                     x_min=x_min, y_max=y_max)
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

#: The cross-track half of the same sample, in its own colour family so the two
#: coordinates of one point never read as two unrelated scales.
RAW_LATERAL_COLOR = "#c5b0d5"
APPLIED_LATERAL_COLOR = "tab:purple"

#: Percentile windows the y-axis is fitted to. The raw ratio reaches 60x on real
#: runs, so it never sets the limits; the estimate is what has to stay readable.
#: A linear axis has no decades to absorb the scatter, so it shows less of it.
_RAW_PERCENTILES_LOG = (5.0, 95.0)
_RAW_PERCENTILES_LINEAR = (5.0, 75.0)
_ESTIMATE_PERCENTILES = (1.0, 99.0)


def has_lateral(geometries):
    """True when any frame carries a cross-track sample worth its own axes."""
    return any(np.isfinite(getattr(g, "lateral_instant_raw", np.nan))
               or np.isfinite(getattr(g, "lateral_applied", np.nan))
               for g in geometries)


def draw_scale_history(ax, geometries, *, title="", scale_min=None, scale_max=None,
                       lateral_max=None, log_y=False, lateral_ax=None):
    """Draw the scale estimate over the run into an existing axes.

    Three curves per frame, all as the estimator produced them: the raw
    instantaneous ratio, the smoothed mean of the observable samples, and what
    the trajectory was actually built with. Frames whose sample failed the
    observability gate are shaded, since that is why the smoothed curve holds
    flat instead of following the raw one.

    Under ``complementaryCorrection: lines_meet_xy`` the sample is a point
    rather than a number, and its cross-track coordinate is drawn alongside in
    purple -- raw and applied, against a zero line, since 0 is what "no sideways
    correction" means there as 1 is for the scale. It is skipped on a log axis,
    where a signed quantity cannot be drawn.

    The axis is linear, so a deviation reads as the number it is. ``log_y``
    switches to a logarithmic one, which is the axis a *ratio* deserves -- 2 and
    0.5 are the same error in opposite directions -- and which keeps a run whose
    estimate reaches 20x readable near 1. Either way the range is fitted to the
    bulk of the estimate rather than to the raw ratio's excursions.

    ``scale_min``/``scale_max`` draw the configured sample bounds when finite.
    Clears and redraws ``ax``; returns it.

    ``lateral_ax`` moves the cross-track curves onto their own axes instead of
    sharing these -- the two coordinates are different quantities, one centred
    on 1 and one on 0, and stacking them under a shared time axis reads better
    than overlaying them. Pass None to overlay, which is what a single-axes
    figure does.
    """
    ax.clear()

    t = np.array([g.time for g in geometries], dtype=float)
    t = t - t[0] if t.size else t
    raw = np.array([g.scale_instant_raw for g in geometries], dtype=float)
    smooth = np.array([g.scale_smooth for g in geometries], dtype=float)
    applied = np.array([g.scale_applied for g in geometries], dtype=float)
    observable = np.array([bool(g.gate_observable) for g in geometries])

    def _series(attr):
        return np.array([getattr(g, attr, np.nan) for g in geometries], dtype=float)

    lateral_raw = _series("lateral_instant_raw")
    lateral_applied = _series("lateral_applied")
    # Only worth drawing when something produced one; the other corrections
    # leave these all-NaN. On its own axes it survives a log scale change,
    # which a signed quantity sharing a log axis could not.
    have_lateral = bool(np.any(np.isfinite(lateral_applied))
                        or np.any(np.isfinite(lateral_raw)))
    show_lateral = have_lateral and (lateral_ax is not None or not log_y)
    overlaid = show_lateral and lateral_ax is None

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

    if overlaid:
        # 0 is this coordinate's 1: the odometry pointing exactly where the
        # lines say the robot went.
        ax.axhline(0.0, color="0.45", linewidth=0.9, zorder=0)
        ax.plot(t, lateral_raw, color=RAW_LATERAL_COLOR, linewidth=0.7, zorder=1,
                label="lateral raw")
        ax.plot(t, lateral_applied, color=APPLIED_LATERAL_COLOR, linewidth=1.3,
                linestyle="--", zorder=3, label="lateral applied")

    if log_y:
        ax.set_yscale("log")
    # The shared time axis belongs to whichever plot is at the bottom.
    ax.set_xlabel("" if show_lateral and not overlaid else "t [s]")
    ax.set_ylabel(("scale / lateral [|comp|]" if overlaid else "scale")
                  + ("  (log)" if log_y else ""))
    ax.set_title(title)
    ax.grid(True, which="both", linestyle=":", linewidth=0.5)
    if t.size:
        ax.set_xlim(float(t[0]), float(t[-1]) if t[-1] > t[0] else float(t[0]) + 1.0)
    ax.set_ylim(*scale_axis_limits(raw, smooth, applied,
                                   lateral=((lateral_raw, lateral_applied)
                                            if overlaid else ()),
                                   bounds=(scale_min, scale_max), log=log_y))
    _legend(ax)

    if show_lateral and not overlaid:
        draw_lateral_history(lateral_ax, geometries, lateral_max=lateral_max)
    return ax


def draw_lateral_history(ax, geometries, *, lateral_max=None):
    """Draw the cross-track half of the correction into its own axes.

    The companion of :func:`draw_scale_history` under a shared time axis: same
    frames, same observability shading, the other coordinate of the same point.
    Read against 0 rather than 1 -- zero is the odometry already pointing where
    the lines say the robot went -- and always linear, since it is signed.

    Clears and redraws ``ax``; returns it.
    """
    ax.clear()

    t = np.array([g.time for g in geometries], dtype=float)
    t = t - t[0] if t.size else t
    raw = np.array([getattr(g, "lateral_instant_raw", np.nan) for g in geometries],
                   dtype=float)
    applied = np.array([getattr(g, "lateral_applied", np.nan) for g in geometries],
                       dtype=float)
    observable = np.array([bool(g.gate_observable) for g in geometries])

    if t.size:
        from matplotlib.transforms import blended_transform_factory

        ax.fill_between(t, 0.0, 1.0, where=~observable, step="mid",
                        transform=blended_transform_factory(ax.transData, ax.transAxes),
                        color=UNOBSERVABLE_COLOR, alpha=0.10, linewidth=0,
                        label="not observable")

    ax.axhline(0.0, color="0.45", linewidth=0.9, zorder=0)
    for bound in (lateral_max, -lateral_max if lateral_max is not None else None):
        if bound is not None and np.isfinite(bound):
            ax.axhline(bound, color=BOUND_COLOR, linewidth=0.9, linestyle="--",
                       alpha=0.7, zorder=0, label="sample bound")

    ax.plot(t, raw, color=RAW_LATERAL_COLOR, linewidth=0.7, zorder=1,
            label="lateral raw")
    ax.plot(t, applied, color=APPLIED_LATERAL_COLOR, linewidth=1.3, linestyle="--",
            zorder=3, label="lateral applied")

    ax.set_xlabel("t [s]")
    ax.set_ylabel("lateral [|comp|]")
    ax.grid(True, which="both", linestyle=":", linewidth=0.5)
    if t.size:
        ax.set_xlim(float(t[0]), float(t[-1]) if t[-1] > t[0] else float(t[0]) + 1.0)
    ax.set_ylim(*lateral_axis_limits(raw, applied, bound=lateral_max))
    _legend(ax)
    return ax


def _legend(ax):
    """Legend without the duplicates two bounds and one shading produce."""
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="upper right", fontsize="small", ncol=2)


def scale_axis_limits(raw, smooth, applied, *, lateral=(), bounds=(None, None), log=False):
    """A y-range that keeps the estimate readable, letting the raw ratio clip.

    Fitted to the bulk of each series rather than to its extremes -- one 60x
    sample would otherwise squash the rest flat -- and always including 1.0,
    which is what the curves are read against.

    ``lateral`` are cross-track series, which are signed: including any of them
    lifts the floor at zero that a scale-only axis keeps.
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
    signed = False
    for series in lateral:
        found = percentiles(np.asarray(series, dtype=float), _ESTIMATE_PERCENTILES)
        values += found
        signed = signed or bool(found)
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
    floor = lo - margin if signed else max(0.0, lo - margin)
    return floor, hi + margin


def lateral_axis_limits(raw, applied, *, bound=None):
    """A symmetric y-range around 0 for the cross-track curves.

    Symmetric because the quantity is a direction: left and right are the same
    size of error, and an axis that says otherwise reads as a trend. Fitted to
    the bulk of the raw samples, which have the same heavy tail the scale does.
    """
    values = [0.05]
    for series, window in ((applied, _ESTIMATE_PERCENTILES), (raw, (5.0, 95.0))):
        finite = np.asarray(series, dtype=float)
        finite = finite[np.isfinite(finite)]
        if finite.size:
            values += [abs(float(v)) for v in np.percentile(finite, window)]
    if bound is not None and np.isfinite(bound):
        values.append(abs(float(bound)))
    reach = max(values) * 1.15
    return -reach, reach


def build_scale_history_figure(geometries, *, title="", scale_min=None, scale_max=None,
                               lateral_max=None, log_y=False, split=True,
                               figsize=(9, 5)):
    """Standalone figure of the scale history; for tests and headless use.

    Two stacked plots on a shared time axis when the correction produced a
    cross-track coordinate, one otherwise. ``split=False`` overlays them on one
    axes instead, which is what a figure too short for two panels wants.
    """
    import matplotlib.pyplot as plt

    if split and has_lateral(geometries):
        fig, (ax, lateral_ax) = plt.subplots(
            2, 1, figsize=figsize, sharex=True,
            gridspec_kw={"height_ratios": [2, 1]})
    else:
        fig, ax = plt.subplots(figsize=figsize)
        lateral_ax = None
    draw_scale_history(ax, geometries, title=title, scale_min=scale_min,
                       scale_max=scale_max, lateral_max=lateral_max, log_y=log_y,
                       lateral_ax=lateral_ax)
    fig.tight_layout()
    return fig
