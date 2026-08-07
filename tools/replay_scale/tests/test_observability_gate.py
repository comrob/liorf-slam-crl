"""What `scaleMinNonDegenerateSpeed` is measured on.

An intentional divergence from the node, so it is worth pinning precisely: the
gate asks whether the *LiDAR* observed enough motion to measure a ratio against.
Nothing from the complementary odometry may decide the answer -- neither the
projection onto its direction nor its matched interval -- because those are the
quantity under test.
"""

import numpy as np
import pytest

from replay_scale.core.estimator import build_additional_odom_scale_sample


def _sample(lidar, comp, *, dt_lidar=1.0, dt_comp=1.0, min_speed=0.0):
    """One frame with the given LiDAR and complementary displacements (3-vectors)."""
    T_anchor = np.eye(4)
    T_latest = np.eye(4)
    T_latest[:3, 3] = lidar
    T_lidar_rel = np.eye(4)
    T_lidar_rel[:3, 3] = lidar
    T_comp_rel = np.eye(4)
    T_comp_rel[:3, 3] = comp
    return build_additional_odom_scale_sample(
        T_anchor, T_latest, T_lidar_rel, T_comp_rel, [], False, dt_comp,
        min_nondegenerate_speed=min_speed, dt_lidar_s=dt_lidar)


def test_a_moving_lidar_stays_observable_however_the_odometry_points():
    """The node's projection would shrink this to cos(60 deg) and reject it."""
    lidar = [1.0, 0.0, 0.0]                     # 1 m/s along x
    sideways = [np.cos(np.pi / 3), np.sin(np.pi / 3), 0.0]   # 60 deg off
    assert _sample(lidar, sideways, min_speed=0.9)["gate_observable"]


def test_a_short_odometry_vector_does_not_make_the_lidar_unobservable():
    assert _sample([1.0, 0.0, 0.0], [0.01, 0.0, 0.0], min_speed=0.9)["gate_observable"]


def test_a_stationary_lidar_is_not_observable_however_far_the_odometry_ran():
    assert not _sample([0.01, 0.0, 0.0], [5.0, 0.0, 0.0], min_speed=0.5)["gate_observable"]


def test_the_speed_is_over_the_lidar_window_not_the_odometry_one():
    """1 m over a 2 s LiDAR window is 0.5 m/s, whatever the odometry matched."""
    fast_odom_window = dict(dt_lidar=2.0, dt_comp=0.5)
    assert not _sample([1.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                       min_speed=0.9, **fast_odom_window)["gate_observable"]
    assert _sample([1.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                   min_speed=0.4, **fast_odom_window)["gate_observable"]


def test_the_threshold_is_inclusive():
    assert _sample([1.0, 0.0, 0.0], [1.0, 0.0, 0.0], min_speed=1.0)["gate_observable"]


def test_the_scale_ratio_still_uses_the_projection():
    """Only the gate moved: the estimate itself is unchanged node behaviour.

    LiDAR 2 m along x, odometry 1 m at 60 deg: the projection of the LiDAR
    displacement onto the odometry direction is 2 cos(60 deg) = 1, so the ratio
    is 1.0 -- not the 2.0 an unprojected ratio would give.
    """
    out = _sample([2.0, 0.0, 0.0], [np.cos(np.pi / 3), np.sin(np.pi / 3), 0.0])
    assert out["scale_instant_raw"] == pytest.approx(1.0)


def test_the_lidar_window_defaults_to_the_complementary_one():
    """A caller with only the odometry interval divides by that, not by 1."""
    T_latest = np.eye(4)
    T_latest[0, 3] = 1.0
    T_rel = np.eye(4)
    T_rel[0, 3] = 1.0
    out = build_additional_odom_scale_sample(
        np.eye(4), T_latest, T_rel, T_rel, [], False, 2.0,      # 1 m over 2 s
        min_nondegenerate_speed=0.9)
    assert not out["gate_observable"]
