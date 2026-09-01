"""Roll/pitch/yaw is how the GUI lets a mount be typed, matrices are what runs.

A mount typed in one form and applied in the other only stays the same mount
while the two conversions are inverses, and the extrinsic is the setting whose
mistakes are invisible: a wrong one silently rotates every complementary
displacement and comes back out as a scale.
"""

import numpy as np
import pytest

from replay_scale.core.se3 import matrix_to_rpy, rpy_to_matrix
from replay_scale.settings import rotation_from_sequence


ANGLES = [
    (0.0, 0.0, 0.0),
    (0.3, -0.2, 1.1),
    (np.pi, 0.0, 0.0),
    (-2.5, 0.9, 3.0),
    (0.0, 0.0, np.pi / 2),
]


@pytest.mark.parametrize("roll,pitch,yaw", ANGLES)
def test_a_mount_survives_the_trip_through_angles_and_back(roll, pitch, yaw):
    T = rpy_to_matrix(0.1, -0.2, 0.3, roll, pitch, yaw)
    back = rpy_to_matrix(*matrix_to_rpy(T))
    np.testing.assert_allclose(back, T, atol=1e-12)


def test_the_rotation_built_is_rz_ry_rx():
    """The ROS convention, so a mount means the same here as in a URDF."""
    roll, pitch, yaw = 0.4, -0.7, 2.0
    Rx = rpy_to_matrix(0, 0, 0, roll, 0, 0)[:3, :3]
    Ry = rpy_to_matrix(0, 0, 0, 0, pitch, 0)[:3, :3]
    Rz = rpy_to_matrix(0, 0, 0, 0, 0, yaw)[:3, :3]
    np.testing.assert_allclose(rpy_to_matrix(0, 0, 0, roll, pitch, yaw)[:3, :3],
                               Rz @ Ry @ Rx, atol=1e-12)


def test_yaw_90_is_the_matrix_the_config_would_spell():
    """The extrinsic in the bundled config, typed as an angle instead."""
    expected = rotation_from_sequence([0, -1, 0, 1, 0, 0, 0, 0, 1])
    np.testing.assert_allclose(rpy_to_matrix(0, 0, 0, 0, 0, np.pi / 2)[:3, :3],
                               expected, atol=1e-12)


@pytest.mark.parametrize("pitch", [np.pi / 2, -np.pi / 2])
def test_gimbal_lock_still_rebuilds_the_same_rotation(pitch):
    """A sensor looking straight down: roll and yaw are one rotation there.

    Which of the two the split is reported in is a choice; reproducing the
    matrix is not, and the naive atan2 pair silently drops the coupled angle.
    """
    T = rpy_to_matrix(0.0, 0.0, 0.0, 0.6, pitch, -1.3)
    tx, ty, tz, roll, reported_pitch, yaw = matrix_to_rpy(T)
    assert roll == 0.0
    assert reported_pitch == pytest.approx(pitch)
    np.testing.assert_allclose(rpy_to_matrix(tx, ty, tz, roll, reported_pitch, yaw),
                               T, atol=1e-9)


def test_the_translation_is_carried_through_untouched():
    T = rpy_to_matrix(-0.31, 0.0, 0.159, 0.0, 0.0, np.pi)
    assert matrix_to_rpy(T)[:3] == pytest.approx((-0.31, 0.0, 0.159))
