"""Bounding what a scale sample may contribute to the filter.

The applied scale is a mean over the samples that entered the filter, so what
the bounds do to the *value* is the whole behaviour: an out-of-range sample
enters at the bound rather than being thrown away, and what was measured is
still reported unchanged.
"""

import numpy as np
import pytest

from replay_scale.core.estimator import build_additional_odom_scale_sample
from replay_scale.core.model import ReplayParams
from replay_scale.settings import replay_params_from_mapping, validate_replay_params


def _sample(lidar_x, comp_x, *, scale_min=0.0, scale_max=float("inf"), dt=1.0,
            min_speed=0.0):
    """One frame travelling along +x, so the raw scale is lidar_x / comp_x."""
    T_anchor = np.eye(4)
    T_latest = np.eye(4)
    T_latest[0, 3] = lidar_x
    T_lidar_rel = np.eye(4)
    T_lidar_rel[0, 3] = lidar_x
    T_comp_rel = np.eye(4)
    T_comp_rel[0, 3] = comp_x
    return build_additional_odom_scale_sample(
        T_anchor, T_latest, T_lidar_rel, T_comp_rel, [], False, dt,
        min_nondegenerate_speed=min_speed, scale_min=scale_min, scale_max=scale_max)


def test_the_raw_scale_is_the_ratio_the_bounds_apply_to():
    assert _sample(1.5, 1.0)["scale_instant_raw"] == pytest.approx(1.5)


def test_defaults_clamp_nothing():
    """No node parameter corresponds to this, so off is the parity default."""
    p = ReplayParams()
    assert p.scale_min == 0.0
    assert p.scale_max == float("inf")
    for ratio in (0.001, 1.0, 1000.0):
        out = _sample(ratio, 1.0)
        assert out["gate_observable"]
        assert out["scale_filtered"] == pytest.approx(ratio)


def test_a_sample_above_the_maximum_enters_at_the_maximum():
    """scale_filtered is what reaches the smoothing filter."""
    assert _sample(5.0, 1.0, scale_max=2.0)["scale_filtered"] == pytest.approx(2.0)


def test_a_sample_below_the_minimum_enters_at_the_minimum():
    assert _sample(0.1, 1.0, scale_min=0.5)["scale_filtered"] == pytest.approx(0.5)


def test_a_sample_inside_the_range_is_untouched():
    assert _sample(1.5, 1.0, scale_min=0.5,
                   scale_max=2.0)["scale_filtered"] == pytest.approx(1.5)


def test_the_bounds_are_inclusive():
    assert _sample(2.0, 1.0, scale_min=0.5,
                   scale_max=2.0)["scale_filtered"] == pytest.approx(2.0)
    assert _sample(0.5, 1.0, scale_min=0.5,
                   scale_max=2.0)["scale_filtered"] == pytest.approx(0.5)


def test_a_clamped_sample_is_still_reported_as_measured():
    """The trace and the Scale tab must show the frame's real ratio."""
    out = _sample(5.0, 1.0, scale_max=2.0)
    assert out["scale_instant_raw"] == pytest.approx(5.0)


def test_clamping_does_not_change_observability():
    """A capped sample still counts as a sample; only its value is limited."""
    assert _sample(5.0, 1.0, scale_max=2.0)["gate_observable"]
    assert _sample(0.01, 1.0, scale_min=0.5)["gate_observable"]


def test_an_unobservable_sample_is_not_clamped_into_the_filter():
    """Failing the speed gate means no sample at all, bounds or no bounds."""
    out = _sample(5.0, 1.0, dt=100.0, min_speed=0.5, scale_max=2.0)
    assert not out["gate_observable"]
    assert np.isnan(out["scale_filtered"])


def test_the_speed_gate_is_untouched_by_the_bounds():
    """Setting bounds must not accidentally admit a frame that was too slow."""
    out = _sample(1.0, 1.0, dt=100.0, min_speed=0.5, scale_min=0.5, scale_max=2.0)
    assert out["scale_instant_raw"] == pytest.approx(1.0)   # in range
    assert not out["gate_observable"]                       # but far too slow


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def test_the_bounds_load_from_the_ros_parameter_tree():
    params = replay_params_from_mapping(
        {"complementaryOdom": {"scaleMin": 0.5, "scaleMax": 2.0}})
    assert (params.scale_min, params.scale_max) == (0.5, 2.0)


def test_an_infinite_maximum_survives_the_yaml_spelling():
    params = replay_params_from_mapping(
        {"complementaryOdom": {"scaleMax": float("inf")}})
    assert params.scale_max == float("inf")


def test_a_maximum_below_the_minimum_is_rejected():
    with pytest.raises(ValueError, match="scaleMax"):
        validate_replay_params(ReplayParams(scale_min=2.0, scale_max=1.0))


def test_a_negative_minimum_is_rejected():
    with pytest.raises(ValueError, match="scaleMin"):
        validate_replay_params(ReplayParams(scale_min=-1.0))
