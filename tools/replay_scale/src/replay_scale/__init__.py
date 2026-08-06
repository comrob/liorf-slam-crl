"""Offline replay of complementary-odometry scaling and pose correction.

Layers, innermost first. Each may import the ones above it, never the ones
below:

    core/       pure math, data model, estimator, odom sync -- no I/O
    io/         readers and writers (CSV, TUM, run-directory paths)
    settings    the settings objects and the YAML that populates them
    pipeline    run_replay(): CSV in, trajectories out
    plotting    build_trajectory_figure(): curves in, Figure out
    cli         argparse and printing
    gui/        (future) -- same peer level as cli

Both frontends go through :func:`replay_scale.pipeline.run_replay`, so neither
can do anything the other cannot.
"""

from .core.model import (
    CORRECTION_MODES,
    SCALE_MODES,
    Frame,
    ReplayParams,
    ScaleEstimateFrame,
    ScaleVectorFrame,
)
from .pipeline import ReplayResult, ValidationReport, run_replay
from .settings import (
    DEFAULT_CONFIG_PATH,
    ComplementarySourceSettings,
    ReplayToolSettings,
    load_config,
)

__all__ = [
    "CORRECTION_MODES",
    "SCALE_MODES",
    "DEFAULT_CONFIG_PATH",
    "ComplementarySourceSettings",
    "Frame",
    "ReplayParams",
    "ReplayResult",
    "ReplayToolSettings",
    "ScaleEstimateFrame",
    "ScaleVectorFrame",
    "ValidationReport",
    "load_config",
    "run_replay",
]
