"""Which statistic the smoothing window collapses to.

The scale samples are a ratio of two short displacements, so their distribution
has a long tail. What the window does with that tail is the difference between
the two modes, and it is what these check -- not merely that both compute
something.
"""

import numpy as np
import pytest

from replay_scale.core.estimator import smoothed_scale
from replay_scale.core.model import SMOOTHING_MODES, ReplayParams
from replay_scale.settings import (
    config_to_mapping,
    replay_params_from_mapping,
    validate_replay_params,
)


def test_the_offered_modes():
    assert SMOOTHING_MODES == ("mean", "median", "trimmed")


def test_mean_is_the_default_because_it_is_the_nodes():
    assert ReplayParams().scale_smoothing_mode == "mean"
    assert smoothed_scale([1.0, 2.0, 6.0]) == pytest.approx(3.0)


def test_median_is_the_middle_sample():
    assert smoothed_scale([1.0, 2.0, 6.0], "median") == pytest.approx(2.0)


def test_one_outlier_moves_the_mean_and_not_the_median():
    """A single 20x sample in a 50-wide window is why the mode is selectable."""
    window = [1.0] * 49 + [20.0]
    assert smoothed_scale(window, "mean") == pytest.approx(1.38)
    assert smoothed_scale(window, "median") == pytest.approx(1.0)


def test_the_median_steps_towards_a_persistent_change():
    """Being robust must not mean being stuck: a real shift still comes through."""
    window = [1.0] * 50
    medians = []
    for _ in range(30):                       # a genuine change, sample by sample
        window = window[1:] + [2.0]
        medians.append(smoothed_scale(window, "median"))

    assert medians[0] == pytest.approx(1.0)    # one sample moves it nowhere
    assert medians[-1] == pytest.approx(2.0)   # a sustained change arrives in full
    assert medians == sorted(medians)          # and it only ever moves that way


def test_the_median_moves_by_one_order_statistic_per_sample():
    """Each new observation moves it towards its own side, not to it."""
    window = [1.0, 1.0, 1.0, 3.0, 5.0]
    before = smoothed_scale(window, "median")
    after = smoothed_scale(window[1:] + [9.0], "median")
    assert before == pytest.approx(1.0)
    assert after == pytest.approx(3.0)        # one step up, not to 9


def test_a_single_sample_window_is_that_sample_either_way():
    assert smoothed_scale([1.7], "mean") == pytest.approx(1.7)
    assert smoothed_scale([1.7], "median") == pytest.approx(1.7)


def test_an_unknown_mode_is_rejected():
    with pytest.raises(ValueError, match="scale_smoothing_mode"):
        smoothed_scale([1.0], "average")


# ---------------------------------------------------------------------------
# Trimmed: quartiles to find the outliers, mean over the rest
# ---------------------------------------------------------------------------

def test_trimmed_is_the_plain_mean_of_a_clean_window():
    """The fence must not cost anything when there is nothing to cut."""
    window = [1.0, 1.1, 0.9, 1.05, 0.95]
    assert smoothed_scale(window, "trimmed") == pytest.approx(np.mean(window))


def test_trimmed_drops_the_tail_and_averages_the_rest():
    window = [1.0, 1.1, 0.9, 1.05, 0.95, 20.0]
    assert smoothed_scale(window, "mean") == pytest.approx(4.1667, abs=1e-4)
    assert smoothed_scale(window, "trimmed") == pytest.approx(1.0)


def test_trimmed_uses_more_of_the_window_than_the_median():
    """Its point over the median: the inliers all contribute, not just the middle."""
    window = [0.90, 0.95, 1.00, 1.30, 1.35, 40.0]
    assert smoothed_scale(window, "median") == pytest.approx(1.15)
    assert smoothed_scale(window, "trimmed") == pytest.approx(1.10)


def test_trimmed_keeps_a_wide_but_genuine_spread():
    """A broad window is not a tail: nothing lies outside 1.5 IQR here."""
    window = [0.5, 0.8, 1.0, 1.2, 1.5, 2.0]
    assert smoothed_scale(window, "trimmed") == pytest.approx(np.mean(window))


def test_trimmed_falls_back_to_the_median_on_a_short_window():
    """Quartiles of three samples cannot tell a tail from the data."""
    assert smoothed_scale([1.0, 3.0], "trimmed") == pytest.approx(2.0)
    assert smoothed_scale([1.0, 1.0, 9.0], "trimmed") == pytest.approx(1.0)


def test_trimmed_survives_a_window_with_no_spread():
    """iqr == 0 makes the fence a point; every sample is on it, not outside."""
    assert smoothed_scale([2.0] * 10, "trimmed") == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def test_the_mode_loads_from_the_ros_parameter_tree():
    params = replay_params_from_mapping(
        {"complementaryOdom": {"scaleSmoothingMode": "median"}})
    assert params.scale_smoothing_mode == "median"


def test_the_mode_survives_a_round_trip():
    params = ReplayParams(scale_smoothing_mode="median")
    from replay_scale.settings import ReplayToolSettings

    mapping = config_to_mapping(ReplayToolSettings(), params)
    back = replay_params_from_mapping(mapping["/**"]["ros__parameters"])
    assert back.scale_smoothing_mode == "median"


def test_an_unknown_mode_is_rejected_by_the_config():
    with pytest.raises(ValueError, match="scaleSmoothingMode"):
        validate_replay_params(ReplayParams(scale_smoothing_mode="average"))


def test_the_bundled_config_selects_the_trimmed_mean():
    """The tail is real on these runs, so the shipped config opts out of the mean."""
    from replay_scale.settings import DEFAULT_CONFIG_PATH, load_config

    _, params = load_config(DEFAULT_CONFIG_PATH)
    assert params.scale_smoothing_mode == "trimmed"
