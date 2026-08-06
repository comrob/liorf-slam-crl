"""Reader for ``scale_replay_frames.csv`` as written by LiliDiagnostics."""

import csv

import numpy as np

from ..core.model import Frame
from ..core.se3 import quat_to_matrix

CSV_NAME = "scale_replay_frames.csv"


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
            f.lidar_prev_stamp = _col(row, "scale_replay/stamp/lidar_prev_s")
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
