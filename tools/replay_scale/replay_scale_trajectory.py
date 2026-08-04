#!/usr/bin/env python3
"""Offline replay of complementary-odometry scaling and pose correction.

Reads ``scale_replay_frames.csv`` (produced by LiorfDiagnostics when
``log.diagnostics.enable_scale_replay`` is true) and reconstructs map-frame
trajectories without re-running SLAM.

Modes:
- fixed: apply one or more user-provided constant scales.
- recorded: replay with the per-frame scale recorded by the online run.
- estimated: run the C++-style online scale estimator logic over replayed poses,
  using parameters loaded from a ROS YAML config.

The SE(3) math and projection logic are ports of C++ helpers in:
- include/degeneracyDetection/TwistManipulation.hpp
- src/mapOptimization/mapOptimization_degeneracy.cpp
"""

import argparse
import csv
import glob
import os
import sys
from collections import deque
from dataclasses import dataclass

import numpy as np

try:
    import yaml
except ImportError:  # Optional dependency until used.
    yaml = None

CSV_NAME = "scale_replay_frames.csv"


# ----------------------------------------------------------------------------
# SE(3) helpers (ports of include/degeneracyDetection/TwistManipulation.hpp)
# ----------------------------------------------------------------------------
def _skew(w):
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]], dtype=float)


def exp_map(twist, dt=1.0):
    """SE(3) exponential of a velocity twist integrated over ``dt``."""
    v = twist[:3]
    omega = twist[3:]
    oh = _skew(omega)
    theta = float(np.linalg.norm(omega))
    T = np.eye(4, dtype=float)
    if theta < 1e-3:
        R = np.eye(3, dtype=float)
        J = np.eye(3, dtype=float) * dt
    else:
        R = (np.eye(3, dtype=float)
             + (np.sin(theta * dt) / theta) * oh
             + ((1.0 - np.cos(theta * dt)) / (theta * theta)) * (oh @ oh))
        J = (np.eye(3, dtype=float) * dt
             + ((1.0 - np.cos(theta * dt)) / (theta * theta)) * oh
             + ((theta * dt - np.sin(theta * dt)) / (theta ** 3)) * (oh @ oh))
    T[:3, :3] = R
    T[:3, 3] = J @ v
    return T


def matrix_to_twist(T, time=1.0):
    """SE(3) logarithm returning a velocity twist over ``time``."""
    R = T[:3, :3]
    t = T[:3, 3]

    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cos_angle))

    twist = np.zeros(6, dtype=float)
    if angle < 1e-5:
        twist[3:] = 0.0
        twist[:3] = t / time
        return twist

    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]], dtype=float)
    axis = axis / (2.0 * np.sin(angle))

    angular_velocity = (angle / time) * axis
    ow = _skew(angular_velocity)
    J = (np.eye(3, dtype=float)
         + (1.0 - np.cos(angle)) / (angle * angle) * ow
         + (angle - np.sin(angle)) / (angle ** 3) * (ow @ ow))
    twist[:3] = np.linalg.solve(J, t) / time
    twist[3:] = angular_velocity
    return twist


def project_onto_basis(twist, basis):
    """Project a 6D twist onto the span of the given twist basis."""
    proj = np.zeros(6, dtype=float)
    for b in basis:
        denom = float(b @ b)
        if denom < 1e-12:
            continue
        proj += (float(twist @ b) / denom) * b
    return proj


def project_onto_basis_translation(t_vec, basis):
    """Project translation onto span of linear parts of the twist basis."""
    lin = []
    for b in basis:
        v = b[:3].copy()
        for u in lin:
            v -= float(v @ u) * u
        n = float(np.linalg.norm(v))
        if n > 1e-6:
            lin.append(v / n)

    proj = np.zeros(3, dtype=float)
    for u in lin:
        proj += float(t_vec @ u) * u
    return proj


def project_degenerate_correction(T_optimized, T_predicted, basis):
    """Replace degenerate-direction component of T_optimized with prediction."""
    T_diff = np.linalg.inv(T_optimized) @ T_predicted
    xi_diff = matrix_to_twist(T_diff, 1.0)
    xi_proj = project_onto_basis(xi_diff, basis)
    return T_optimized @ exp_map(xi_proj, 1.0)


# ----------------------------------------------------------------------------
# Quaternion helpers (order: qx, qy, qz, qw)
# ----------------------------------------------------------------------------
def quat_to_matrix(tx, ty, tz, qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def matrix_to_quat(T):
    R = T[:3, :3]
    tr = float(np.trace(R))
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    t = T[:3, 3]
    return float(t[0]), float(t[1]), float(t[2]), float(qx), float(qy), float(qz), float(qw)


# ----------------------------------------------------------------------------
# Data model
# ----------------------------------------------------------------------------
@dataclass
class ReplayParams:
    translation_scale: float = 1.0
    scale_estimation_apply: bool = True
    scale_min_nondegenerate_speed: float = 0.2
    scale_baseline_frame_lag: int = 1
    scale_smoothing_window_size: int = 20
    ignore_dz: bool = False


class Frame:
    __slots__ = (
        "time", "dt_scan", "degeneracy_detected", "has_basis",
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


# ----------------------------------------------------------------------------
# CSV loading
# ----------------------------------------------------------------------------
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


# ----------------------------------------------------------------------------
# YAML parameter loading
# ----------------------------------------------------------------------------
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


# ----------------------------------------------------------------------------
# Trajectory reconstruction
# ----------------------------------------------------------------------------
def reconstruct_fixed(frames, scale_of_frame, apply_correction=True, translation_scale_multiplier=1.0):
    """Chain a trajectory from LiDAR increments using fixed/recorded scales."""
    if not frames:
        return []

    T_prev = frames[0].pose_prev.copy()
    out = []
    for f in frames:
        T_optimized = T_prev @ exp_map(f.lidar_increment, 1.0)

        do_correction = (apply_correction and f.degeneracy_detected
                         and f.has_basis and len(f.basis) > 0)
        if do_correction:
            if f.has_complementary:
                scale = scale_of_frame(f)
                dt = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
                xi_comp = f.complementary_twist.copy()
                xi_comp[:3] *= translation_scale_multiplier
                T_comp_rel = exp_map(xi_comp, dt)
                T_comp_rel[:3, 3] *= scale
                T_comp_abs = T_prev @ T_comp_rel
            else:
                T_comp_abs = T_prev.copy()
            T_corrected = project_degenerate_correction(T_optimized, T_comp_abs, f.basis)
        else:
            T_corrected = T_optimized

        out.append((f.time, T_corrected))
        T_prev = T_corrected
    return out


def build_additional_odom_scale_sample(
    T_anchor,
    T_latest,
    T_lidar_rel,
    T_comp_rel,
    basis,
    ignore_dz,
    dt_complementary_s,
    min_nondegenerate_speed,
):
    """Port of mapOptimization::buildAdditionalOdomCorrectionResult for scaling."""
    t_lidar_rel_anchor = T_lidar_rel[:3, 3].copy()
    t_comp_rel_uncorrected_anchor = T_comp_rel[:3, 3].copy()

    R_lidar_rel = T_lidar_rel[:3, :3]
    R_comp_rel = T_comp_rel[:3, :3]
    R_orientation_drift = R_comp_rel @ R_lidar_rel.T
    t_comp_rel_corrected_anchor = R_orientation_drift.T @ t_comp_rel_uncorrected_anchor

    if ignore_dz:
        t_lidar_rel_anchor[2] = 0.0
        t_comp_rel_uncorrected_anchor[2] = 0.0
        t_comp_rel_corrected_anchor[2] = 0.0

    R_anchor = T_anchor[:3, :3]
    R_latest = T_latest[:3, :3]

    t_comp_corrected_map = R_anchor @ t_comp_rel_corrected_anchor

    t_lidar_map = T_latest[:3, 3] - T_anchor[:3, 3]
    t_lidar_latest_local = R_latest.T @ t_lidar_map
    t_comp_latest_local = R_latest.T @ t_comp_corrected_map

    if ignore_dz:
        t_lidar_latest_local[2] = 0.0
        t_comp_latest_local[2] = 0.0

    t_lidar_nondeg = t_lidar_latest_local - project_onto_basis_translation(t_lidar_latest_local, basis)
    t_comp_nondeg = t_comp_latest_local - project_onto_basis_translation(t_comp_latest_local, basis)

    comp_nondeg_norm = float(np.linalg.norm(t_comp_nondeg))
    comp_unit = np.zeros(3, dtype=float)
    if comp_nondeg_norm > 1e-6:
        comp_unit = t_comp_nondeg / comp_nondeg_norm

    t_lidar_nondeg_proj = float(t_lidar_nondeg @ comp_unit) * comp_unit
    lidar_proj_norm = float(np.linalg.norm(t_lidar_nondeg_proj))

    scale_instant_raw = np.nan
    if comp_nondeg_norm > 1e-6:
        scale_instant_raw = lidar_proj_norm / comp_nondeg_norm

    projected_nondeg_speed = lidar_proj_norm / max(1e-5, dt_complementary_s)
    gate_observable = (
        np.isfinite(scale_instant_raw)
        and projected_nondeg_speed >= max(0.0, min_nondegenerate_speed)
    )

    return {
        "valid": True,
        "gate_observable": bool(gate_observable),
        "scale_instant_raw": float(scale_instant_raw),
        "scale_filtered": float(scale_instant_raw) if gate_observable else np.nan,
    }


def reconstruct_with_estimator(frames, params):
    """Replay with online-style lagged scale estimation and application.

        Notes about parity with C++:
        - Uses same observability gate and smoothing history update rule.
        - Uses same causal ordering: applied scale at frame k comes from history
            accumulated up to frame k-1.
        - In replay, estimation is always on; only application is gated by
            ``scaleEstimationApply``.
    """
    if not frames:
        return [], []

    T_prev = frames[0].pose_prev.copy()
    out = []
    scale_trace = []

    baseline_lag = max(1, int(params.scale_baseline_frame_lag))
    lidar_pose_buffer = deque()  # tuples: (frame_idx, stamp, T_effective)
    lagged_scale_filtered_history = deque()

    n = len(frames)
    comp_step_rel = [None] * n  # index k stores rel transform from k-1 -> k.
    comp_step_dt = [np.nan] * n

    for k, f in enumerate(frames):
        basis = f.basis if (f.has_basis and len(f.basis) > 0) else []
        estimator_mode_active = bool(f.degeneracy_detected)
        scale_apply_enabled = bool(params.scale_estimation_apply)

        smoothed_scale_for_apply = np.nan
        if scale_apply_enabled and len(lagged_scale_filtered_history) > 0:
            smoothed_scale_for_apply = float(np.mean(lagged_scale_filtered_history))

        has_applied_scale = scale_apply_enabled and np.isfinite(smoothed_scale_for_apply)
        scale_applied = float(smoothed_scale_for_apply) if has_applied_scale else 1.0

        T_optimized = T_prev @ exp_map(f.lidar_increment, 1.0)
        T_comp_unscaled_abs = T_prev.copy()
        T_comp_scaled_abs = T_prev.copy()

        has_comp = bool(f.has_complementary and np.all(np.isfinite(f.complementary_twist)))
        if has_comp:
            dt_pred = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
            dt_pred = max(1e-5, float(dt_pred))
            xi_comp = f.complementary_twist.copy()
            # Recorded twists already include the original run's translationScale.
            # This multiplier is interpreted as an additional replay-time factor.
            xi_comp[:3] *= float(params.translation_scale)

            T_comp_rel_unscaled = exp_map(xi_comp, dt_pred)
            T_comp_rel_scaled = T_comp_rel_unscaled.copy()
            T_comp_rel_scaled[:3, 3] *= scale_applied

            T_comp_unscaled_abs = T_prev @ T_comp_rel_unscaled
            T_comp_scaled_abs = T_prev @ T_comp_rel_scaled

            comp_step_rel[k] = T_comp_rel_unscaled
            comp_step_dt[k] = dt_pred

        if estimator_mode_active and len(basis) > 0:
            T_corrected = project_degenerate_correction(T_optimized, T_comp_scaled_abs, basis)
        else:
            T_corrected = T_optimized

        T_effective = T_corrected if (estimator_mode_active and len(basis) > 0) else T_optimized
        out.append((f.time, T_effective))

        lidar_pose_buffer.append((k, f.time, T_effective.copy()))
        while len(lidar_pose_buffer) > baseline_lag + 1:
            lidar_pose_buffer.popleft()

        gate_observable = False
        scale_instant_raw = np.nan
        scale_filtered = np.nan

        # Port of lagged scale update path:
        # if buffer has anchor/current pair, estimate lagged scale sample and
        # append to filtered history when observable.
        if len(lidar_pose_buffer) >= 2:
            anchor_idx, _, T_anchor = lidar_pose_buffer[0]
            latest_idx, _, T_latest = lidar_pose_buffer[-1]

            # Compose complementary relative delta over same lag window.
            T_comp_rel_window = np.eye(4, dtype=float)
            dt_comp_window = 0.0
            have_full_window = True
            for j in range(anchor_idx + 1, latest_idx + 1):
                if comp_step_rel[j] is None or not np.isfinite(comp_step_dt[j]):
                    have_full_window = False
                    break
                T_comp_rel_window = T_comp_rel_window @ comp_step_rel[j]
                dt_comp_window += float(comp_step_dt[j])

            if have_full_window and dt_comp_window > 1e-5:
                T_lidar_rel = np.linalg.inv(T_anchor) @ T_latest
                sample = build_additional_odom_scale_sample(
                    T_anchor,
                    T_latest,
                    T_lidar_rel,
                    T_comp_rel_window,
                    basis,
                    bool(params.ignore_dz),
                    dt_comp_window,
                    float(params.scale_min_nondegenerate_speed),
                )
                gate_observable = sample["gate_observable"]
                scale_instant_raw = sample["scale_instant_raw"]
                scale_filtered = sample["scale_filtered"]

                if np.isfinite(scale_filtered):
                    lagged_scale_filtered_history.append(float(scale_filtered))
                while len(lagged_scale_filtered_history) > max(1, int(params.scale_smoothing_window_size)):
                    lagged_scale_filtered_history.popleft()

        scale_smooth = float(np.mean(lagged_scale_filtered_history)) if lagged_scale_filtered_history else np.nan
        scale_trace.append(ScaleEstimateFrame(
            frame_idx=k,
            time=float(f.time),
            gate_observable=bool(gate_observable),
            scale_instant_raw=float(scale_instant_raw),
            scale_filtered=float(scale_filtered),
            scale_smooth=float(scale_smooth),
            scale_applied=float(scale_applied),
        ))

        T_prev = T_effective

    return out, scale_trace


def write_tum(path, trajectory):
    with open(path, "w", encoding="utf-8") as fh:
        for stamp, T in trajectory:
            tx, ty, tz, qx, qy, qz, qw = matrix_to_quat(T)
            fh.write(f"{stamp:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                     f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")


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


# ----------------------------------------------------------------------------
# CLI helpers
# ----------------------------------------------------------------------------
def _expand(path):
    return os.path.expanduser(path)


def _latest_run_dir(base_dir):
    expanded = _expand(base_dir)
    latest = os.path.join(expanded, "latest")
    if os.path.isdir(latest):
        return latest
    candidates = [d for d in glob.glob(os.path.join(expanded, "run_*")) if os.path.isdir(d)]
    if not candidates:
        raise FileNotFoundError(f"No run directories found under: {base_dir}")
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def _resolve_csv(input_path, latest, base_dir):
    if latest or not input_path:
        run_dir = _latest_run_dir(base_dir)
        csv_path = os.path.join(run_dir, CSV_NAME)
    else:
        expanded = _expand(input_path)
        csv_path = os.path.join(expanded, CSV_NAME) if os.path.isdir(expanded) else expanded
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"{CSV_NAME} not found: {csv_path}")
    return csv_path


def _parse_scales(text):
    values = [float(s) for s in text.split(",") if s.strip()]
    if not values:
        raise ValueError("--scales must contain at least one value")
    return values


def _print_replay_params(params):
    print("  estimator params:")
    print(f"    translationScale: {params.translation_scale}")
    print(f"    scaleEstimationApply: {params.scale_estimation_apply}")
    print(f"    scaleMinNonDegenerateSpeed: {params.scale_min_nondegenerate_speed}")
    print(f"    scaleBaselineFrameLag: {params.scale_baseline_frame_lag}")
    print(f"    scaleSmoothingWindowSize: {params.scale_smoothing_window_size}")
    print(f"    ignore_dz: {params.ignore_dz}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", default="",
                        help="Path to scale_replay_frames.csv or a run directory.")
    parser.add_argument("--latest", action="store_true",
                        help="Use the newest run under --base-dir.")
    parser.add_argument("--base-dir", default="~/.ros/liorf_logs",
                        help="Base directory holding run_* folders (default: %(default)s).")
    parser.add_argument("--scale-mode", choices=("fixed", "recorded", "estimated"), default="fixed",
                        help="Scale source: fixed values, recorded per-frame scales, or online-style estimated scales.")
    parser.add_argument("--scales", default="1.0",
                        help="Comma-separated translation scales for --scale-mode fixed (default: %(default)s).")
    parser.add_argument("--ros-params-yaml", default="",
                        help="ROS YAML with complementaryOdom parameters. Used by --scale-mode estimated.")
    parser.add_argument("--output-dir", default="",
                        help="Base directory for output files (default: alongside the CSV).")
    parser.add_argument("--output-subdir", default="replay_trajectories",
                        help="Subfolder under --output-dir for outputs (default: %(default)s).")
    parser.add_argument("--no-correction", action="store_true",
                        help="Also emit the LiDAR-only trajectory (no degeneracy override).")
    parser.add_argument("--validate", action="store_true",
                        help="Replay with the recorded per-frame scale and report drift vs recorded effective.")
    args = parser.parse_args(argv)

    csv_path = _resolve_csv(args.input, args.latest, args.base_dir)
    frames = load_frames(csv_path)
    if not frames:
        print(f"No frames found in {csv_path}", file=sys.stderr)
        return 1

    base_out_dir = _expand(args.output_dir) if args.output_dir else os.path.dirname(csv_path)
    out_dir = os.path.join(base_out_dir, args.output_subdir)
    os.makedirs(out_dir, exist_ok=True)

    n_deg = sum(1 for f in frames if f.degeneracy_detected and f.has_basis)
    print(f"Loaded {len(frames)} frames from {csv_path}")
    print(f"  frames with degeneracy override: {n_deg}")
    print(f"  output directory: {out_dir}")
    print(f"  scale mode: {args.scale_mode}")

    recorded = recorded_effective_trajectory(frames)
    rec_path = os.path.join(out_dir, "trajectory_recorded_effective.tum")
    write_tum(rec_path, recorded)
    print(f"  wrote {rec_path}")

    if args.validate:
        traj_val = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True)
        drift = position_drift(traj_val, recorded)
        val_path = os.path.join(out_dir, "trajectory_replay_validate.tum")
        write_tum(val_path, traj_val)
        print(f"  wrote {val_path}")
        if drift.size:
            print("  validation drift vs recorded effective: "
                  f"mean={drift.mean():.6f} m  max={drift.max():.6f} m  "
                  f"rms={np.sqrt((drift ** 2).mean()):.6f} m")

    if args.scale_mode == "fixed":
        for scale in _parse_scales(args.scales):
            traj = reconstruct_fixed(frames, lambda f, s=scale: s, apply_correction=True)
            path = os.path.join(out_dir, f"trajectory_replay_scale_{scale:g}.tum")
            write_tum(path, traj)
            print(f"  wrote {path}  (scale={scale:g})")

    elif args.scale_mode == "recorded":
        traj = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True)
        path = os.path.join(out_dir, "trajectory_replay_recorded_scale.tum")
        write_tum(path, traj)
        print(f"  wrote {path}")

    elif args.scale_mode == "estimated":
        params = ReplayParams()
        if args.ros_params_yaml:
            params = load_replay_params_from_ros_yaml(_expand(args.ros_params_yaml))
        _print_replay_params(params)

        traj, scale_trace = reconstruct_with_estimator(frames, params)
        traj_path = os.path.join(out_dir, "trajectory_replay_estimated_scale.tum")
        trace_path = os.path.join(out_dir, "scale_replay_estimator_trace.csv")
        write_tum(traj_path, traj)
        write_scale_trace_csv(trace_path, scale_trace)
        print(f"  wrote {traj_path}")
        print(f"  wrote {trace_path}")

    if args.no_correction:
        traj = reconstruct_fixed(frames, lambda f: 1.0, apply_correction=False)
        path = os.path.join(out_dir, "trajectory_replay_lidar_only.tum")
        write_tum(path, traj)
        print(f"  wrote {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
