"""SE(3) and quaternion math helpers.

Ports of:
- include/degeneracyDetection/TwistManipulation.hpp
"""

import numpy as np


# ---------------------------------------------------------------------------
# SE(3) helpers
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Quaternion helpers (order: qx, qy, qz, qw)
# ---------------------------------------------------------------------------

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
