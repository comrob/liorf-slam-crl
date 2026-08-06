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


@dataclass
class ReplayParams:
    """The complementaryOdom subset of the node's parameters used by replay."""

    translation_scale: float = 1.0
    scale_estimation_apply: bool = True
    scale_min_nondegenerate_speed: float = 0.2
    scale_baseline_frame_lag: int = 1
    scale_smoothing_window_size: int = 20
    ignore_dz: bool = False


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
    __slots__ = (
        "frame_idx", "time", "degeneracy_detected", "gate_observable",
        "anchor_pos", "latest_pos",
        "t_lidar_map", "t_comp_map",
        "t_lidar_nondeg_map", "t_comp_nondeg_map",
        "t_lidar_nondeg_proj_map", "nondeg_axis_map",
        "scale_instant_raw", "scale_smooth", "scale_applied",
    )
