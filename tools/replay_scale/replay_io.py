"""CSV/YAML input and TUM/CSV output helpers."""

import csv
import glob
import os
from dataclasses import dataclass, field

import numpy as np

try:
    import yaml
except ImportError:  # Optional dependency until used.
    yaml = None

from .scale_estimator import (
    Frame,
    ReplayParams,
    ScaleEstimateFrame,
    ScaleVectorFrame,
    reconstruct_fixed,
    reconstruct_with_estimator,
)
from .se3_math import matrix_to_quat, quat_to_matrix

_SCALE_MODES = ("fixed", "recorded", "estimated")

CSV_NAME = "scale_replay_frames.csv"
DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config", "default.yaml")


@dataclass
class ReplayToolSettings:
    scale_mode: str = "estimated"
    scales: list = field(default_factory=lambda: [1.0])
    output_dir: str = ""
    output_subdir: str = "replay"
    no_correction: bool = False
    validate: bool = False


# ---------------------------------------------------------------------------
# CLI / path helpers shared by replay_scale_trajectory.py and plot_trajectories.py
# ---------------------------------------------------------------------------

def expand_path(path):
    return os.path.expanduser(path)


def latest_run_dir(base_dir):
    expanded = expand_path(base_dir)
    latest = os.path.join(expanded, "latest")
    if os.path.isdir(latest):
        return latest
    candidates = [d for d in glob.glob(os.path.join(expanded, "run_*")) if os.path.isdir(d)]
    if not candidates:
        raise FileNotFoundError(f"No run directories found under: {base_dir}")
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def resolve_csv_path(input_path, latest, base_dir):
    if latest or not input_path:
        run_dir = latest_run_dir(base_dir)
        csv_path = os.path.join(run_dir, CSV_NAME)
    else:
        expanded = expand_path(input_path)
        csv_path = os.path.join(expanded, CSV_NAME) if os.path.isdir(expanded) else expanded
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"{CSV_NAME} not found: {csv_path}")
    return csv_path


def resolve_output_dirs(csv_path, settings):
    """Return (out_dir, trajectories_dir, log_dir) for the given settings."""
    base_out_dir = expand_path(settings.output_dir) if settings.output_dir else os.path.dirname(csv_path)
    out_dir = os.path.join(base_out_dir, settings.output_subdir)
    traj_dir = os.path.join(out_dir, "trajectories")
    log_dir = os.path.join(out_dir, "log")
    return out_dir, traj_dir, log_dir


def reconstruct_replay_trajectories(frames, settings, params, collect_traces=False):
    """Build the trajectory/trajectories selected by settings.scale_mode.

    Returns (results, scale_trace, vector_trace) where results is a list of
    (tag, trajectory) pairs; scale_trace/vector_trace are only populated for
    scale_mode "estimated" (and only when collect_traces is True).
    """
    if settings.scale_mode == "fixed":
        results = []
        for scale in settings.scales:
            traj = reconstruct_fixed(frames, lambda f, s=scale: s, apply_correction=True)
            results.append((f"scale_{scale:g}", traj))
        return results, [], []

    if settings.scale_mode == "recorded":
        traj = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True)
        return [("recorded_scale", traj)], [], []

    if settings.scale_mode == "estimated":
        traj, scale_trace, vector_trace = reconstruct_with_estimator(
            frames, params, collect_vectors=collect_traces)
        return [("estimated_scale", traj)], scale_trace, vector_trace

    raise ValueError(f"Unknown replay_scale_tool.scale_mode: {settings.scale_mode!r}")


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def _col(row, key):
    return float(row[key])


def _twist(row, prefix):
    return np.array([_col(row, f"{prefix}/{a}") for a in ("vx", "vy", "vz", "wx", "wy", "wz")], dtype=float)


def _pose(row, prefix):
    vals = [_col(row, f"{prefix}/{a}") for a in ("tx", "ty", "tz", "qx", "qy", "qz", "qw")]
    return quat_to_matrix(*vals)


def load_frames(csv_path):
    frames = []
    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            f = Frame()
            f.time = _col(row, "time")
            f.dt_scan = _col(row, "scale_replay/dt/scan_s")
            f.degeneracy_detected = _col(row, "scale_replay/flags/degeneracy_detected") >= 0.5
            f.has_basis = _col(row, "scale_replay/flags/has_degeneracy_basis") >= 0.5
            f.has_complementary = _col(row, "scale_replay/flags/has_complementary_twist") >= 0.5
            f.scale_applied = _col(row, "scale_replay/scale/applied")
            f.basis_size = int(round(_col(row, "scale_replay/basis/size")))
            f.basis = []
            for i in range(min(f.basis_size, 3)):
                b = np.array([_col(row, f"scale_replay/basis/{i}/{a}")
                              for a in ("vx", "vy", "vz", "wx", "wy", "wz")], dtype=float)
                if np.all(np.isfinite(b)):
                    f.basis.append(b)
            f.lidar_increment = _twist(row, "scale_replay/lidar_increment")
            f.complementary_twist = _twist(row, "scale_replay/complementary_twist")
            f.dt_complementary = _col(row, "scale_replay/complementary/dt_s")
            f.pose_prev = _pose(row, "scale_replay/pose_prev")
            f.pose_optimized = _pose(row, "scale_replay/pose_optimized")
            f.pose_effective = _pose(row, "scale_replay/pose_effective")
            frames.append(f)
    return frames


# ---------------------------------------------------------------------------
# YAML parameter loading
# ---------------------------------------------------------------------------

def _extract_ros_parameters_root(raw_yaml):
    if not isinstance(raw_yaml, dict):
        return {}

    if "/**" in raw_yaml and isinstance(raw_yaml["/**"], dict):
        candidate = raw_yaml["/**"].get("ros__parameters")
        if isinstance(candidate, dict):
            return candidate

    for _, value in raw_yaml.items():
        if isinstance(value, dict):
            candidate = value.get("ros__parameters")
            if isinstance(candidate, dict):
                return candidate

    # Fallback: assume user passed just the parameters subtree.
    return raw_yaml


def load_replay_params_from_ros_yaml(yaml_path):
    if yaml is None:
        raise RuntimeError("PyYAML is required for --ros-params-yaml. Install pyyaml.")

    with open(yaml_path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)

    root = _extract_ros_parameters_root(raw)
    params = ReplayParams()

    comp = root.get("complementaryOdom", {}) if isinstance(root, dict) else {}
    if isinstance(comp, dict):
        params.translation_scale = float(comp.get("translationScale", params.translation_scale))
        params.scale_estimation_apply = bool(comp.get("scaleEstimationApply", params.scale_estimation_apply))
        params.scale_min_nondegenerate_speed = float(
            comp.get("scaleMinNonDegenerateSpeed", params.scale_min_nondegenerate_speed))
        params.scale_baseline_frame_lag = int(comp.get("scaleBaselineFrameLag", params.scale_baseline_frame_lag))
        params.scale_smoothing_window_size = int(comp.get("scaleSmoothingWindowSize", params.scale_smoothing_window_size))
        params.ignore_dz = bool(comp.get("ignore_dz", params.ignore_dz))

    params.scale_baseline_frame_lag = max(1, params.scale_baseline_frame_lag)
    params.scale_smoothing_window_size = max(1, params.scale_smoothing_window_size)
    return params


def load_replay_tool_settings(yaml_path):
    """Load the replay_scale_tool section (sibling of the ROS ros__parameters tree)."""
    if yaml is None:
        raise RuntimeError("PyYAML is required to load replay tool settings. Install pyyaml.")

    with open(yaml_path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)

    section = raw.get("replay_scale_tool", {}) if isinstance(raw, dict) else {}
    settings = ReplayToolSettings()
    if isinstance(section, dict):
        settings.scale_mode = str(section.get("scale_mode", settings.scale_mode))
        settings.scales = [float(s) for s in section.get("scales", settings.scales)]
        settings.output_dir = str(section.get("output_dir", settings.output_dir))
        settings.output_subdir = str(section.get("output_subdir", settings.output_subdir))
        settings.no_correction = bool(section.get("no_correction", settings.no_correction))
        settings.validate = bool(section.get("validate", settings.validate))

    if settings.scale_mode not in _SCALE_MODES:
        raise ValueError(
            f"replay_scale_tool.scale_mode must be one of {_SCALE_MODES}, got {settings.scale_mode!r}")
    if not settings.scales:
        raise ValueError("replay_scale_tool.scales must contain at least one value")
    return settings


# ---------------------------------------------------------------------------
# Output writers
# ---------------------------------------------------------------------------

def write_tum(path, trajectory):
    with open(path, "w", encoding="utf-8") as fh:
        for stamp, T in trajectory:
            tx, ty, tz, qx, qy, qz, qw = matrix_to_quat(T)
            fh.write(f"{stamp:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                     f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")


def load_tum(path):
    trajectory = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            stamp, tx, ty, tz, qx, qy, qz, qw = (float(v) for v in line.split()[:8])
            trajectory.append((stamp, quat_to_matrix(tx, ty, tz, qx, qy, qz, qw)))
    return trajectory


def write_scale_trace_csv(path, scale_trace):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "frame_idx",
            "time",
            "gate_observable",
            "scale_instant_raw",
            "scale_filtered",
            "scale_smooth",
            "scale_applied",
        ])
        for s in scale_trace:
            writer.writerow([
                s.frame_idx,
                f"{s.time:.9f}",
                1 if s.gate_observable else 0,
                f"{s.scale_instant_raw:.9f}" if np.isfinite(s.scale_instant_raw) else "nan",
                f"{s.scale_filtered:.9f}" if np.isfinite(s.scale_filtered) else "nan",
                f"{s.scale_smooth:.9f}" if np.isfinite(s.scale_smooth) else "nan",
                f"{s.scale_applied:.9f}" if np.isfinite(s.scale_applied) else "nan",
            ])


def recorded_effective_trajectory(frames):
    return [(f.time, f.pose_effective) for f in frames]


def position_drift(traj_a, traj_b):
    """Per-pose Euclidean position difference (assumes aligned frame order)."""
    n = min(len(traj_a), len(traj_b))
    diffs = np.array([np.linalg.norm(traj_a[i][1][:3, 3] - traj_b[i][1][:3, 3])
                      for i in range(n)], dtype=float)
    return diffs


def write_scale_vector_csv(path, vector_trace):
    if not vector_trace:
        return

    _vec_fields = (
        ("anchor_pos",              "anchor"),
        ("latest_pos",              "latest"),
        ("t_lidar_map",             "t_lidar_map"),
        ("t_comp_map",              "t_comp_map"),
        ("t_lidar_nondeg_map",      "t_lidar_nondeg_map"),
        ("t_comp_nondeg_map",       "t_comp_nondeg_map"),
        ("t_lidar_nondeg_proj_map", "t_lidar_nondeg_proj_map"),
        ("nondeg_axis_map",         "nondeg_axis_map"),
    )
    header = ["frame_idx", "time", "degeneracy_detected", "gate_observable"]
    for _, col in _vec_fields:
        for ax in ("x", "y", "z"):
            header.append(f"{col}/{ax}")
    header += ["scale_instant_raw", "scale_smooth", "scale_applied"]

    def _fs(x):
        return f"{float(x):.9f}" if np.isfinite(x) else "nan"

    def _fv(v):
        return [_fs(c) for c in v]

    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for vf in vector_trace:
            row = [
                vf.frame_idx,
                f"{vf.time:.9f}",
                1 if vf.degeneracy_detected else 0,
                1 if vf.gate_observable else 0,
            ]
            for attr, _ in _vec_fields:
                row.extend(_fv(getattr(vf, attr)))
            row += [_fs(vf.scale_instant_raw), _fs(vf.scale_smooth), _fs(vf.scale_applied)]
            writer.writerow(row)
