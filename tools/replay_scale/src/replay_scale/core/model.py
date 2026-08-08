"""Data model shared by the estimator, the readers and the pipeline.

Pure data: no I/O, no algorithm. Kept separate from :mod:`estimator` so that
the readers in ``replay_scale.io`` can build frames without importing the
reconstruction code.
"""

from dataclasses import dataclass

# "twist6" mirrors the node; "translation" leaves observable orientation alone.
CORRECTION_MODES = ("twist6", "translation")

# Scale sources accepted by replay_scale_tool.scale_mode.
SCALE_MODES = ("fixed", "recorded", "estimated")

# How the complementary displacement is corrected before it is substituted into
# the degenerate directions. The first two are scalars -- the displacement keeps
# its direction and only its length changes -- and differ in where that scalar
# is measured; the third also moves it sideways.
#   ratio         - |lidar_nondeg projected on comp| / |comp_nondeg|, as the node
#                   does: one frame measured against its own odometry.
#   line_x_axis   - where this frame's own degenerate line crosses the
#                   complementary axis (y = 0) in that normalized frame. The
#                   same measurement as the ratio -- in the plane with one
#                   degenerate direction the two are equal up to sign -- but
#                   signed, and read off the geometry the view draws.
#   lines_meet_x  - the along-complementary coordinate of the point where the
#                   recent degenerate lines meet, in units of |comp|.
#   lines_meet_xy - that whole point. In the frame where the complementary
#                   displacement is (d, 0), the lines put the robot at
#                   (d*cx, d*cy), so the correction is that vector rather than
#                   the length d*cx alone. It is the only one that can express a
#                   *lateral* odometry error, which no scale can.
COMPLEMENTARY_CORRECTIONS = ("ratio", "line_x_axis", "lines_meet_x", "lines_meet_xy")

# The subset fitted from where the degenerate lines *meet*, i.e. the ones that
# scaleLineHistory / scaleLineHistoryStep / scaleLineFitNorm apply to.
# "line_x_axis" reads one frame's own line and needs no history.
LINES_MEET_CORRECTIONS = ("lines_meet_x", "lines_meet_xy")

# How the accepted samples in the smoothing window become the applied scale.
# "mean" mirrors the node; "median" is robust to the ratio's heavy tail;
# "trimmed" drops the tail by quartile fence and averages what is left.
SMOOTHING_MODES = ("mean", "median", "trimmed")


@dataclass
class ReplayParams:
    """The complementaryOdom subset of the node's parameters used by replay.

    Grouped as the config file is: what the correction is measured from, then
    the settings of each method, then what every method shares.
    """

    # -- all methods --------------------------------------------------------
    translation_scale: float = 1.0
    scale_estimation_apply: bool = True
    scale_min_nondegenerate_speed: float = 0.2
    scale_baseline_frame_lag: int = 1
    ignore_dz: bool = False
    # What the complementary displacement is corrected by; see
    # COMPLEMENTARY_CORRECTIONS. "ratio" is the node's.
    complementary_correction: str = "ratio"

    # -- lines_meet_x / lines_meet_xy ---------------------------------------
    # How many earlier frames' degenerate lines the meeting point is fitted
    # through, alongside the current one. 0 fits nothing and also turns off the
    # meeting point the viewer draws its trail from.
    scale_line_history: int = 50
    # Stride through those earlier lines: consecutive frames' lines are nearly
    # identical, so a stride buys span -- which is what makes them meet -- at
    # the same cost. The window reaches back history * step frames.
    scale_line_history_step: int = 1
    # What that fit minimises over the distances to those lines; see
    # core.lines.LINE_FIT_NORMS. Also decides how the view draws the point.
    scale_line_fit_norm: str = "l2"
    # Symmetric bound the cross-track coordinate is clamped into, in units of
    # |comp|, before it enters the smoothing filter. Only "lines_meet_xy" has
    # one to clamp.
    scale_lateral_max: float = float("inf")

    # -- smoothing, all methods ---------------------------------------------
    scale_smoothing_window_size: int = 20
    # Statistic taken over that window; see SMOOTHING_MODES. "mean" is the
    # node's, so it is the default here.
    scale_smoothing_mode: str = "mean"
    # Range the along-track sample is clamped into before it enters that
    # filter. Defaults are no bound at all: the node has no such limit, so
    # bounding is opt-in.
    scale_min: float = 0.0
    scale_max: float = float("inf")

    @property
    def uses_lines(self):
        """True when the correction is fitted from where the lines meet."""
        return self.complementary_correction in LINES_MEET_CORRECTIONS

    @property
    def corrects_laterally(self):
        """True when the cross-track coordinate is applied, not just measured."""
        return self.complementary_correction == "lines_meet_xy"


class Frame:
    """One row of scale_replay_frames.csv, decoded."""

    __slots__ = (
        "time", "lidar_prev_stamp", "dt_scan", "degeneracy_detected", "has_basis",
        "has_complementary", "scale_applied", "basis_size", "basis",
        "lidar_increment", "complementary_twist", "dt_complementary",
        "pose_prev", "pose_optimized", "pose_effective",
    )


@dataclass
class ScaleEstimateFrame:
    frame_idx: int
    time: float
    gate_observable: bool
    scale_instant_raw: float
    scale_filtered: float
    scale_smooth: float
    scale_applied: float
    # The cross-track half of the same sample, in units of |comp|. NaN unless
    # the correction is "lines_meet_xy", which is the only one that applies it.
    lateral_instant_raw: float = float("nan")
    lateral_filtered: float = float("nan")
    lateral_smooth: float = float("nan")
    lateral_applied: float = float("nan")


class ScaleVectorFrame:
    # anchor_frame_idx is the estimator's lagged-baseline anchor for this frame.
    # It is always set -- the anchor pose exists whether or not the complementary
    # window closed -- so frontends never re-derive the buffer rule themselves.
    #
    # meet_point is (cx, cy) in the frame's own complementary-aligned, |comp| = 1
    # geometry -- where the recent degenerate lines put the robot. Recorded
    # whenever there is a line history to fit through, whatever the correction
    # is, since the viewer draws its trail as a diagnostic either way.
    __slots__ = (
        "frame_idx", "time", "degeneracy_detected", "gate_observable",
        "anchor_frame_idx",
        "anchor_pos", "latest_pos",
        "t_lidar_map", "t_comp_map",
        "t_lidar_nondeg_map", "t_comp_nondeg_map",
        "t_lidar_nondeg_proj_map", "nondeg_axis_map",
        "meet_point",
        "scale_instant_raw", "scale_smooth", "scale_applied",
        "lateral_instant_raw", "lateral_smooth", "lateral_applied",
    )
