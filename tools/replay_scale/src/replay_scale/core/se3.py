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


def orthonormal_translation_basis(basis):
    """Gram-Schmidt the linear parts of a twist basis into unit 3-vectors.

    Returns at most three vectors; near-dependent directions are dropped. This
    is the degenerate translational subspace: the directions along which LiDAR
    constrains nothing and the complementary prediction is substituted.
    """
    lin = []
    for b in basis:
        v = b[:3].copy()
        for u in lin:
            v -= float(v @ u) * u
        n = float(np.linalg.norm(v))
        if n > 1e-6:
            lin.append(v / n)
    return lin


def project_onto_basis_translation(t_vec, basis):
    """Project translation onto span of linear parts of the twist basis."""
    proj = np.zeros(3, dtype=float)
    for u in orthonormal_translation_basis(basis):
        proj += float(t_vec @ u) * u
    return proj


def project_degenerate_correction(T_optimized, T_predicted, basis):
    """Replace degenerate-direction component of T_optimized with prediction.

    Port of mapOptimization::projectDegenerateCorrection. Note this projects the
    full 6D twist, so a basis vector carrying any angular part also rotates the
    pose; see ``project_degenerate_correction_translation``.
    """
    T_diff = np.linalg.inv(T_optimized) @ T_predicted
    xi_diff = matrix_to_twist(T_diff, 1.0)
    xi_proj = project_onto_basis(xi_diff, basis)
    return T_optimized @ exp_map(xi_proj, 1.0)


def project_degenerate_correction_translation(T_optimized, T_predicted, basis):
    """Correct only translation along the degenerate directions; keep orientation.

    The 6D projection mixes metres and radians in one inner product, so residual
    angular content in the basis converts an injected translation into a heading
    change. Where rotation is observable (tunnel walls constrain yaw) that
    rotation is spurious, so this variant applies a pure body-frame translation
    along the degenerate translational subspace and leaves orientation to LiDAR.
    """
    T_diff = np.linalg.inv(T_optimized) @ T_predicted
    t_proj = project_onto_basis_translation(T_diff[:3, 3], basis)
    correction = np.eye(4, dtype=float)
    correction[:3, 3] = t_proj
    return T_optimized @ correction


# ---------------------------------------------------------------------------
# Changing the body frame
# ---------------------------------------------------------------------------

def rotation_angle(T):
    """Magnitude of a transform's rotation, in radians.

    Invariant under a change of body frame -- conjugation cannot make a
    rotation larger or smaller -- so it says the same thing whichever frame the
    caller measured the motion in.
    """
    R = np.asarray(T, dtype=float)[:3, :3]
    return float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def adjoint(T):
    """The 6x6 Ad_T for this module's ``[v, omega]`` twist ordering.

    Twists are attached to a frame, not to a body: the same motion has a
    different ``v`` at every point of the rigid body, and Ad is what carries one
    to another. For ``T = (R, t)``:

        Ad = [[R, skew(t) @ R],
              [0, R          ]]
    """
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    Ad = np.zeros((6, 6), dtype=float)
    Ad[:3, :3] = R
    Ad[:3, 3:] = _skew(T[:3, 3]) @ R
    Ad[3:, 3:] = R
    return Ad


class BodyFrame:
    """A frame rigidly attached to the LiDAR, and the changes of frame it induces.

    ``T`` is that frame expressed in LiDAR coordinates -- the same convention
    the node's ``extrinsicTrans`` / ``extrinsicRot`` are written in -- so the
    identity is the LiDAR frame itself and every method below is then a no-op.
    The inverse is held alongside it because the estimator applies these once
    per frame to a constant.

    The point of the class is the lever arm. A body rotating about a point
    *other* than this frame's origin moves it: that is real motion of the LiDAR,
    but it is not motion of the odometry sensor and it must not be scaled as
    though it were. Re-expressing a step here makes an in-place rotation about
    this origin carry exactly zero translation, so a scale applied to it changes
    nothing -- which is the property the whole estimation frame exists for.
    """

    __slots__ = ("T", "T_inv", "is_identity")

    def __init__(self, T=None):
        self.T = np.eye(4, dtype=float) if T is None else np.asarray(T, dtype=float).copy()
        self.T_inv = np.linalg.inv(self.T)
        self.is_identity = bool(np.array_equal(self.T, np.eye(4)))

    @property
    def offset(self):
        """Where this frame's origin sits in the LiDAR frame -- the lever arm."""
        return self.T[:3, 3].copy()

    def pose(self, T_map_lidar):
        """Where this frame is, given where the LiDAR is."""
        return np.asarray(T_map_lidar, dtype=float) @ self.T

    def rebase(self, T_rel):
        """A relative motion of the LiDAR, as the same motion of this frame."""
        return self.T_inv @ np.asarray(T_rel, dtype=float) @ self.T

    def unbase(self, T_rel):
        """The inverse of :meth:`rebase`: back to the LiDAR frame."""
        return self.T @ np.asarray(T_rel, dtype=float) @ self.T_inv

    def twist(self, xi):
        """A body twist of the LiDAR, re-expressed in this frame."""
        return adjoint(self.T_inv) @ np.asarray(xi, dtype=float)

    def untwist(self, xi):
        """The inverse of :meth:`twist`."""
        return adjoint(self.T) @ np.asarray(xi, dtype=float)

    def __repr__(self):
        if self.is_identity:
            return "BodyFrame(lidar)"
        t = self.offset
        return f"BodyFrame(offset=[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}])"

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


def rpy_to_matrix(tx, ty, tz, roll, pitch, yaw):
    """4x4 from a translation and intrinsic Z-Y-X Euler angles, in radians.

    The rotation is ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` -- the convention ROS
    spells roll/pitch/yaw in, so an extrinsic typed as three angles means here
    what it means in a URDF or a static_transform_publisher.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def matrix_to_rpy(T):
    """``(tx, ty, tz, roll, pitch, yaw)`` in radians; inverse of rpy_to_matrix.

    At |pitch| = 90 degrees roll and yaw are the same rotation and cannot be
    told apart, so the split is reported as roll = 0 and the whole of it in yaw.
    That still rebuilds the matrix it came from, which is what a round trip
    through an editor needs; it is only the two numbers shown that are a choice.
    """
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
    if abs(R[2, 0]) > 1.0 - 1e-9:
        roll = 0.0
        yaw = float(np.arctan2(-R[0, 1], R[1, 1]))
    else:
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    t = T[:3, 3]
    return float(t[0]), float(t[1]), float(t[2]), roll, pitch, yaw


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
