"""TUM trajectory files ("stamp tx ty tz qx qy qz qw", one pose per line)."""

from ..core.se3 import matrix_to_quat, quat_to_matrix


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
