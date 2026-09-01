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

# Which body frame the correction is measured and applied in.
#   lidar         - the node's own: everything happens in the LiDAR frame, so a
#                   body rotation about any other point translates the LiDAR by
#                   the lever arm between them, and that translation is measured
#                   as odometry error and then multiplied by the scale.
#   complementary - the frame of the odometry being corrected, located by
#                   T_complementary_to_lidar: base_link for a legged state
#                   estimator, the camera for visual odometry. An in-place
#                   rotation about that origin carries no translation there, so
#                   no sample is taken from one and no scale is applied to one.
ESTIMATION_FRAMES = ("lidar", "complementary")

# How an external reference trajectory is placed against the replay. All of
# them are rigid: the reference's own shape is never touched, and no scale is
# ever fitted -- a scale is the quantity under test.
#   first_position_yaw - anchor its first matched position on the replay's, then
#                turn it about the vertical by the angle that best fits the
#                replayed trajectory. The default, and the one a total station
#                wants: it uses only positions, which is all such a reference
#                has, and it fits only the heading, which is all that is
#                genuinely unknown between two gravity-levelled frames. Roll and
#                pitch stay unfitted, so a real tilt error still shows.
#   first_position_rotation - the same, with the full 3D rotation fitted. Use
#                where the reference's frame is not levelled; it will absorb a
#                genuine tilt error along with the unknown mounting.
#   first_pose - anchor its *pose* -- position and orientation -- at the
#                replay's first. No fit at all, but it trusts the orientation in
#                the file, which a position-only reference does not have.
#   none       - draw it in its own coordinates, untouched. For a reference
#                already in the run's map frame.
REFERENCE_ALIGNMENTS = ("first_position_yaw", "first_position_rotation",
                        "first_pose", "none")

#: Alignments that fit a rotation against the replayed trajectory.
FITTED_ALIGNMENTS = ("first_position_yaw", "first_position_rotation")

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
    # Which body frame that correction is measured and applied in; see
    # ESTIMATION_FRAMES. "lidar" is the node's, so it is the default.
    estimation_frame: str = "lidar"
    # Whether the complementary frame adopts the extrinsic's rotation as well as
    # its origin. Off keeps the LiDAR's axes and moves only the origin, which is
    # what removes the lever arm; the axes decide something else entirely -- the
    # plane the lines are fitted in, what ignore_dz drops, and which way a
    # simulated drift points. Turn it on for an odometry frame that shares the
    # robot's convention (a base_link), leave it off for one that does not (a
    # camera optical frame is z-forward, and adopting it would fit the lines in
    # the vertical plane).
    estimation_frame_use_extrinsic_rot: bool = False

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
    # Largest uncertainty along the scale axis, in units of |comp|, that still
    # yields a sample: "only answer when the lines pin the scale to better than
    # this". inf takes whatever the fit returns, which is what the node does.
    scale_line_max_scale_sigma: float = 0.05
    # How many of the fitted lines must come from the *other* group of
    # directions -- the opposite leg of a zig-zag -- and how far apart the two
    # groups must be. Geometry the sigma above cannot see: near-parallel lines
    # that agree closely with each other report a small spread while crossing at
    # a glancing angle. 0 lines turns the test off.
    scale_line_min_leg_lines: int = 3
    scale_line_min_leg_separation_deg: float = 10.0

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
    def estimates_in_complementary_frame(self):
        """True when the correction is measured where the odometry actually is."""
        return self.estimation_frame == "complementary"

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
        # The observability speed gate on its own. Under a lines-meet
        # correction gate_observable is additionally narrowed by whether the
        # fit was accepted, so it stops answering "did this frame contribute a
        # line"; this one still does, and it is what the estimator admits into
        # its line history.
        "speed_gate_observable",
        "anchor_frame_idx",
        "anchor_pos", "latest_pos",
        "t_lidar_map", "t_comp_map",
        "t_lidar_nondeg_map", "t_comp_nondeg_map",
        "t_lidar_nondeg_proj_map", "nondeg_axis_map",
        "meet_point",
        # How far the lag window turned, in radians. Frame-independent, and the
        # axis to plot scale_instant_raw against: under "lidar" the two are
        # correlated through the lever arm, which is the artefact the
        # "complementary" frame removes.
        "window_rotation_rad",
        "scale_instant_raw", "scale_smooth", "scale_applied",
        "lateral_instant_raw", "lateral_smooth", "lateral_applied",
    )
