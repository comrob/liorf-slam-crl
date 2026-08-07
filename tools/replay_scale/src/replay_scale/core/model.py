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

# How the accepted samples in the smoothing window become the applied scale.
# "mean" mirrors the node; "median" is robust to the ratio's heavy tail;
# "trimmed" drops the tail by quartile fence and averages what is left.
SMOOTHING_MODES = ("mean", "median", "trimmed")


@dataclass
class ReplayParams:
    """The complementaryOdom subset of the node's parameters used by replay."""

    translation_scale: float = 1.0
    scale_estimation_apply: bool = True
    scale_min_nondegenerate_speed: float = 0.2
    scale_baseline_frame_lag: int = 1
    scale_smoothing_window_size: int = 20
    # Statistic taken over that window; see SMOOTHING_MODES. "mean" is the
    # node's, so it is the default here.
    scale_smoothing_mode: str = "mean"
    ignore_dz: bool = False
    # Range a scale sample is clamped into before it enters the smoothing
    # filter. Defaults are no bound at all: the node has no such limit, so
    # bounding is opt-in.
    scale_min: float = 0.0
    scale_max: float = float("inf")


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


class ScaleVectorFrame:
    # anchor_frame_idx is the estimator's lagged-baseline anchor for this frame.
    # It is always set -- the anchor pose exists whether or not the complementary
    # window closed -- so frontends never re-derive the buffer rule themselves.
    __slots__ = (
        "frame_idx", "time", "degeneracy_detected", "gate_observable",
        "anchor_frame_idx",
        "anchor_pos", "latest_pos",
        "t_lidar_map", "t_comp_map",
        "t_lidar_nondeg_map", "t_comp_nondeg_map",
        "t_lidar_nondeg_proj_map", "nondeg_axis_map",
        "scale_instant_raw", "scale_smooth", "scale_applied",
    )
