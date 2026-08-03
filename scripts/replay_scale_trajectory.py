#!/usr/bin/env python3
"""Offline replay of the complementary-odometry scaling correction.

Reads ``scale_replay_frames.csv`` (produced by LiorfDiagnostics when
``log.diagnostics.enable_scale_replay`` is true) and reconstructs the map-frame
trajectory for one or more chosen translation scales, WITHOUT re-running SLAM.

Only the LiDAR increment is treated as trustworthy across counterfactual
scales: each pose is chained from the previous reconstructed pose using the
recorded per-frame LiDAR increment twist, and the degenerate directions are
overwritten by the (scaled) complementary-odometry prediction, exactly as the
online node does in ``applyDegeneracyStateOverride``.

The SE(3) exponential/logarithm and the subspace projection are ports of the
C++ helpers in ``include/degeneracyDetection/TwistManipulation.hpp`` so that the
replayed geometry matches the online pipeline frame-for-frame.
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np

CSV_NAME = "scale_replay_frames.csv"


# ----------------------------------------------------------------------------
# SE(3) helpers (ports of include/degeneracyDetection/TwistManipulation.hpp)
# ----------------------------------------------------------------------------
def _skew(w):
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def exp_map(twist, dt=1.0):
    """SE(3) exponential of a velocity twist integrated over ``dt``."""
    v = twist[:3]
    omega = twist[3:]
    oh = _skew(omega)
    theta = float(np.linalg.norm(omega))
    T = np.eye(4)
    if theta < 1e-3:
        R = np.eye(3)
        J = np.eye(3) * dt
    else:
        R = (np.eye(3)
             + (np.sin(theta * dt) / theta) * oh
             + ((1.0 - np.cos(theta * dt)) / (theta * theta)) * (oh @ oh))
        J = (np.eye(3) * dt
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

    twist = np.zeros(6)
    if angle < 1e-5:
        twist[3:] = 0.0
        twist[:3] = t / time
        return twist

    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]])
    axis = axis / (2.0 * np.sin(angle))

    angular_velocity = (angle / time) * axis
    ow = _skew(angular_velocity)
    J = (np.eye(3)
         + (1.0 - np.cos(angle)) / (angle * angle) * ow
         + (angle - np.sin(angle)) / (angle ** 3) * (ow @ ow))
    twist[:3] = np.linalg.solve(J, t) / time
    twist[3:] = angular_velocity
    return twist


def project_onto_basis(twist, basis):
    """Project a 6D twist onto the span of the given twist basis."""
    proj = np.zeros(6)
    for b in basis:
        denom = float(b @ b)
        if denom < 1e-12:
            continue
        proj += (float(twist @ b) / denom) * b
    return proj


def project_degenerate_correction(T_optimized, T_predicted, basis):
    """Replace the degenerate-direction component of ``T_optimized`` with the
    prediction, keeping the non-degenerate directions from LiDAR."""
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
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def matrix_to_quat(T):
    R = T[:3, :3]
    tr = np.trace(R)
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
    return t[0], t[1], t[2], qx, qy, qz, qw


# ----------------------------------------------------------------------------
# CSV loading
# ----------------------------------------------------------------------------
class Frame:
    __slots__ = (
        "time", "dt_scan", "degeneracy_detected", "has_basis",
        "has_complementary", "scale_applied", "basis_size", "basis",
        "lidar_increment", "complementary_twist", "dt_complementary",
        "pose_prev", "pose_optimized", "pose_effective",
    )


def _col(row, key):
    return float(row[key])


def _twist(row, prefix):
    return np.array([_col(row, f"{prefix}/{a}") for a in ("vx", "vy", "vz", "wx", "wy", "wz")])


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
                              for a in ("vx", "vy", "vz", "wx", "wy", "wz")])
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
# Trajectory reconstruction
# ----------------------------------------------------------------------------
def reconstruct(frames, scale_of_frame, apply_correction=True):
    """Chain a trajectory from LiDAR increments, overriding degenerate
    directions with the scaled complementary prediction.

    ``scale_of_frame(frame)`` returns the translation scale to apply for a
    frame. Returns a list of (time, T_pose) tuples.
    """
    if not frames:
        return []

    T_prev = frames[0].pose_prev.copy()
    out = []
    for f in frames:
        T_optimized = T_prev @ exp_map(f.lidar_increment, 1.0)

        do_correction = (apply_correction and f.degeneracy_detected
                         and f.has_basis and len(f.basis) > 0)
        if do_correction:
            # Mirror the online init: absent a complementary twist, the
            # prediction defaults to the previous pose (no relative motion).
            if f.has_complementary:
                scale = scale_of_frame(f)
                dt = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
                T_comp_rel = exp_map(f.complementary_twist, dt)
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


def write_tum(path, trajectory):
    with open(path, "w") as fh:
        for stamp, T in trajectory:
            tx, ty, tz, qx, qy, qz, qw = matrix_to_quat(T)
            fh.write(f"{stamp:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                     f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")


def recorded_effective_trajectory(frames):
    return [(f.time, f.pose_effective) for f in frames]


def position_drift(traj_a, traj_b):
    """Per-pose Euclidean position difference (assumes aligned frame order)."""
    n = min(len(traj_a), len(traj_b))
    diffs = np.array([np.linalg.norm(traj_a[i][1][:3, 3] - traj_b[i][1][:3, 3])
                      for i in range(n)])
    return diffs


# ----------------------------------------------------------------------------
# CLI
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
    return [float(s) for s in text.split(",") if s.strip()]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", default="",
                        help="Path to scale_replay_frames.csv or a run directory.")
    parser.add_argument("--latest", action="store_true",
                        help="Use the newest run under --base-dir.")
    parser.add_argument("--base-dir", default="~/.ros/liorf_logs",
                        help="Base directory holding run_* folders (default: %(default)s).")
    parser.add_argument("--scales", default="1.0",
                        help="Comma-separated translation scales to replay (default: %(default)s).")
    parser.add_argument("--output-dir", default="",
                        help="Directory for output .tum files (default: alongside the CSV).")
    parser.add_argument("--no-correction", action="store_true",
                        help="Also emit the LiDAR-only trajectory (no degeneracy override).")
    parser.add_argument("--validate", action="store_true",
                        help="Replay with the recorded per-frame applied scale and report "
                             "position drift versus the recorded effective trajectory.")
    args = parser.parse_args(argv)

    csv_path = _resolve_csv(args.input, args.latest, args.base_dir)
    frames = load_frames(csv_path)
    if not frames:
        print(f"No frames found in {csv_path}", file=sys.stderr)
        return 1

    out_dir = _expand(args.output_dir) if args.output_dir else os.path.dirname(csv_path)
    os.makedirs(out_dir, exist_ok=True)

    n_deg = sum(1 for f in frames if f.degeneracy_detected and f.has_basis)
    print(f"Loaded {len(frames)} frames from {csv_path}")
    print(f"  frames with degeneracy override: {n_deg}")

    recorded = recorded_effective_trajectory(frames)
    rec_path = os.path.join(out_dir, "trajectory_recorded_effective.tum")
    write_tum(rec_path, recorded)
    print(f"  wrote {rec_path}")

    if args.validate:
        traj = reconstruct(frames, lambda f: f.scale_applied, apply_correction=True)
        drift = position_drift(traj, recorded)
        val_path = os.path.join(out_dir, "trajectory_replay_validate.tum")
        write_tum(val_path, traj)
        print(f"  wrote {val_path}")
        if drift.size:
            print(f"  validation drift vs recorded effective: "
                  f"mean={drift.mean():.6f} m  max={drift.max():.6f} m  "
                  f"rms={np.sqrt((drift ** 2).mean()):.6f} m")

    for scale in _parse_scales(args.scales):
        traj = reconstruct(frames, lambda f, s=scale: s, apply_correction=True)
        path = os.path.join(out_dir, f"trajectory_replay_scale_{scale:g}.tum")
        write_tum(path, traj)
        print(f"  wrote {path}  (scale={scale:g})")

    if args.no_correction:
        traj = reconstruct(frames, lambda f: 1.0, apply_correction=False)
        path = os.path.join(out_dir, "trajectory_replay_lidar_only.tum")
        write_tum(path, traj)
        print(f"  wrote {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
