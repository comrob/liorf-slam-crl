"""The replay itself: CSV in, trajectories out.

This is the seam every frontend goes through. :func:`run_replay` performs a
complete replay and returns everything it computed in memory; it never prints
and, with ``write=False``, never touches the filesystem either. The CLI is a
thin argparse wrapper around it, and a GUI is expected to be the same -- see
``README.md`` for why frontends call this rather than shelling out to the CLI.
"""

import os
from dataclasses import dataclass, field

import numpy as np

from .core.estimator import reconstruct_fixed, reconstruct_with_estimator
from .core.odom_source import apply_odom_source, sync_odom_to_frames
from .io.paths import (
    complementary_source_tag,
    expand_path,
    resolve_output_dirs,
)
from .io.frames_csv import load_frames
from .io.traces_csv import write_scale_trace_csv, write_scale_vector_csv
from .io.tum import load_tum, write_tum
from .settings import META_NAME, load_complementary_odom_meta


@dataclass
class ValidationReport:
    """Replay-with-recorded-scale vs. the pose the online run actually used."""

    trajectory: list
    drift: np.ndarray

    @property
    def mean(self):
        return float(self.drift.mean()) if self.drift.size else float("nan")

    @property
    def max(self):
        return float(self.drift.max()) if self.drift.size else float("nan")

    @property
    def rms(self):
        return float((self.drift ** 2).mean()) ** 0.5 if self.drift.size else float("nan")


@dataclass
class ReplayResult:
    """Everything one replay produced, in memory.

    ``written`` lists the files actually created; it stays empty when
    :func:`run_replay` is called with ``write=False``.
    """

    csv_path: str
    settings: object
    params: object
    frames: list
    out_dir: str
    traj_dir: str
    log_dir: str
    source_status: str = None
    recorded_effective: list = field(default_factory=list)
    replays: list = field(default_factory=list)       # [(tag, trajectory)]
    lidar_only: list = None
    validation: ValidationReport = None
    scale_trace: list = field(default_factory=list)
    vector_trace: list = field(default_factory=list)
    written: list = field(default_factory=list)

    @property
    def n_frames(self):
        return len(self.frames)

    @property
    def n_degeneracy_override(self):
        return sum(1 for f in self.frames if f.degeneracy_detected and f.has_basis)

    def curves(self):
        """(label, trajectory, kind) triples for :mod:`replay_scale.plotting`."""
        out = [("recorded effective", self.recorded_effective, "reference")]
        out += [(tag.replace("_", " "), traj, "replay") for tag, traj in self.replays]
        if self.validation is not None:
            out.append(("validate", self.validation.trajectory, "replay"))
        if self.lidar_only is not None:
            out.append(("lidar only", self.lidar_only, "replay"))
        return out


# ---------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------

def recorded_effective_trajectory(frames):
    return [(f.time, f.pose_effective) for f in frames]


def position_drift(traj_a, traj_b):
    """Per-pose Euclidean position difference (assumes aligned frame order)."""
    n = min(len(traj_a), len(traj_b))
    diffs = np.array([np.linalg.norm(traj_a[i][1][:3, 3] - traj_b[i][1][:3, 3])
                      for i in range(n)], dtype=float)
    return diffs


def apply_complementary_source(frames, settings, csv_path):
    """Re-synchronize an alternative complementary odometry source onto frames.

    No-op returning None when no source path is configured, in which case the
    twist recorded by the online run is kept. Otherwise the frames are mutated
    in place and a short human-readable status is returned.
    """
    source = settings.complementary_source
    if not source.path:
        return None

    stream_path = expand_path(source.path)
    if not os.path.isfile(stream_path):
        raise FileNotFoundError(f"Complementary odometry source not found: {stream_path}")

    T_ext = source.extrinsic
    extrinsic_origin = "config override"
    if T_ext is None:
        T_ext = load_complementary_odom_meta(csv_path)
        extrinsic_origin = f"run {META_NAME}"
    if T_ext is None:
        T_ext = np.eye(4, dtype=float)
        extrinsic_origin = "identity (no config override and no run meta file)"

    odom_stream = load_tum(stream_path)
    synced = sync_odom_to_frames(odom_stream, frames, T_ext,
                                 max_match_dt_s=source.max_match_dt_s)
    matched = apply_odom_source(frames, synced)
    return (f"complementary source: {stream_path}\n"
            f"    extrinsic: {extrinsic_origin}\n"
            f"    stream samples: {len(odom_stream)}\n"
            f"    matched frames: {matched}/{len(frames)}")


def reconstruct_replay_trajectories(frames, settings, params, collect_traces=False):
    """Build the trajectory/trajectories selected by settings.scale_mode.

    Returns (results, scale_trace, vector_trace) where results is a list of
    (tag, trajectory) pairs; scale_trace/vector_trace are only populated for
    scale_mode "estimated" (and only when collect_traces is True).
    """
    mode = settings.correction_mode
    if settings.scale_mode == "fixed":
        results = []
        for scale in settings.scales:
            traj = reconstruct_fixed(frames, lambda f, s=scale: s, apply_correction=True,
                                     correction_mode=mode)
            results.append((f"scale_{scale:g}", traj))
        return results, [], []

    if settings.scale_mode == "recorded":
        traj = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True,
                                 correction_mode=mode)
        return [("recorded_scale", traj)], [], []

    if settings.scale_mode == "estimated":
        traj, scale_trace, vector_trace = reconstruct_with_estimator(
            frames, params, collect_vectors=collect_traces, correction_mode=mode)
        return [("estimated_scale", traj)], scale_trace, vector_trace

    raise ValueError(f"Unknown replay_scale_tool.scale_mode: {settings.scale_mode!r}")


# ---------------------------------------------------------------------------
# The pipeline
# ---------------------------------------------------------------------------

def _format_params(params):
    return ("  estimator params:\n"
            f"    translationScale: {params.translation_scale}\n"
            f"    scaleEstimationApply: {params.scale_estimation_apply}\n"
            f"    scaleMinNonDegenerateSpeed: {params.scale_min_nondegenerate_speed}\n"
            f"    scaleBaselineFrameLag: {params.scale_baseline_frame_lag}\n"
            f"    scaleSmoothingWindowSize: {params.scale_smoothing_window_size}\n"
            f"    ignore_dz: {params.ignore_dz}")


def run_replay(csv_path, settings, params=None, *, write=True, on_progress=None):
    """Replay one run and return a :class:`ReplayResult`.

    ``on_progress`` receives human-readable status lines as the replay
    advances; the CLI passes ``print``, a GUI can append to a log pane. Pass
    ``write=False`` to compute without emitting any file -- what a GUI wants
    while the user is dragging a parameter around.

    Raises ValueError if the CSV holds no frames.
    """
    emit = on_progress if on_progress is not None else (lambda _msg: None)

    frames = load_frames(csv_path)
    if not frames:
        raise ValueError(f"No frames found in {csv_path}")

    out_dir, traj_dir, log_dir = resolve_output_dirs(csv_path, settings)
    if write:
        os.makedirs(traj_dir, exist_ok=True)
        os.makedirs(log_dir, exist_ok=True)

    result = ReplayResult(
        csv_path=csv_path, settings=settings, params=params, frames=frames,
        out_dir=out_dir, traj_dir=traj_dir, log_dir=log_dir,
    )

    def _write_tum(path, trajectory):
        if not write:
            return
        write_tum(path, trajectory)
        result.written.append(path)
        emit(f"  wrote {path}")

    emit(f"Loaded {len(frames)} frames from {csv_path}")
    emit(f"  frames with degeneracy override: {result.n_degeneracy_override}")
    emit(f"  output directory: {out_dir}")
    emit(f"  scale mode: {settings.scale_mode}")

    result.source_status = apply_complementary_source(frames, settings, csv_path)
    if result.source_status:
        emit(f"  {result.source_status}")

    result.recorded_effective = recorded_effective_trajectory(frames)
    _write_tum(os.path.join(traj_dir, "trajectory_recorded_effective.tum"),
               result.recorded_effective)

    if settings.validate:
        # Deliberately twist6 regardless of settings.correction_mode: this
        # reproduces what the online run did, so it must use the node's rule.
        traj_val = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True)
        result.validation = ValidationReport(
            trajectory=traj_val, drift=position_drift(traj_val, result.recorded_effective))
        _write_tum(os.path.join(traj_dir, "trajectory_replay_validate.tum"), traj_val)
        if result.validation.drift.size:
            emit("  validation drift vs recorded effective: "
                 f"mean={result.validation.mean:.6f} m  max={result.validation.max:.6f} m  "
                 f"rms={result.validation.rms:.6f} m")

    if settings.scale_mode == "estimated":
        if params is None:
            raise ValueError("scale_mode 'estimated' requires params")
        emit(_format_params(params))

    source_tag = complementary_source_tag(settings)
    result.replays, result.scale_trace, result.vector_trace = reconstruct_replay_trajectories(
        frames, settings, params, collect_traces=True)
    for tag, traj in result.replays:
        _write_tum(os.path.join(traj_dir, f"trajectory_replay_{tag}{source_tag}.tum"), traj)

    if settings.scale_mode == "estimated" and write:
        trace_path = os.path.join(log_dir, f"scale_replay_estimator_trace{source_tag}.csv")
        vector_path = os.path.join(log_dir, f"scale_replay_vectors{source_tag}.csv")
        write_scale_trace_csv(trace_path, result.scale_trace)
        write_scale_vector_csv(vector_path, result.vector_trace)
        result.written += [trace_path, vector_path]
        emit(f"  wrote {trace_path}")
        emit(f"  wrote {vector_path}")

    if settings.no_correction:
        result.lidar_only = reconstruct_fixed(frames, lambda f: 1.0, apply_correction=False)
        _write_tum(os.path.join(traj_dir, "trajectory_replay_lidar_only.tum"), result.lidar_only)

    return result
