"""A run carrying its own configuration.

Most of what a replay needs is a method setting you sweep across runs; a few
things -- the mounts, the reference trajectory and the clock offset that lines
it up -- are facts about one run and belong with it. So a run may keep its own
file, merged over the base configuration, holding only what differs.

The merge happens on the raw mappings, before anything is parsed, which is why
none of the loaders or validators below had to learn that a configuration can
come from two files.
"""

import os

import numpy as np
import pytest

from replay_scale.core.model import ReplayParams
from replay_scale.settings import (
    DEFAULT_CONFIG_PATH,
    RUN_CONFIG_NAME,
    ReplayToolSettings,
    config_from_mapping,
    config_to_mapping,
    config_sources_for_run,
    deep_merge,
    load_config_for_run,
    mapping_delta,
    run_config_path,
    save_run_config,
)

yaml = pytest.importorskip("yaml")


def _run(tmp_path, run_config=None):
    csv_path = tmp_path / "scale_replay_frames.csv"
    csv_path.write_text("")
    if run_config is not None:
        (tmp_path / RUN_CONFIG_NAME).write_text(yaml.safe_dump(run_config))
    return str(csv_path)


def _tool(**kw):
    return {"replay_scale_tool": kw}


def _params(**kw):
    return {"/**": {"ros__parameters": {"complementaryOdom": kw}}}


# ---------------------------------------------------------------------------
# The merge itself
# ---------------------------------------------------------------------------

def test_a_named_key_does_not_discard_its_siblings():
    """The reason this merges rather than replaces top-level sections.

    A run file naming one parameter must not silently reset every other
    parameter in the same block to its default.
    """
    base = {"a": {"x": 1, "y": 2}, "b": 3}
    merged = deep_merge(base, {"a": {"y": 9}})
    assert merged == {"a": {"x": 1, "y": 9}, "b": 3}


def test_delta_is_the_inverse_of_merge():
    base = {"a": {"x": 1, "y": 2}, "b": 3}
    edited = {"a": {"x": 1, "y": 9}, "b": 3, "c": 4}
    delta = mapping_delta(edited, base)
    assert delta == {"a": {"y": 9}, "c": 4}
    assert deep_merge(base, delta) == edited


def test_an_unchanged_configuration_has_an_empty_delta():
    base = {"a": {"x": 1}}
    assert mapping_delta({"a": {"x": 1}}, base) == {}


# ---------------------------------------------------------------------------
# Layering one run's configuration
# ---------------------------------------------------------------------------

def test_no_run_file_is_exactly_the_base_configuration(tmp_path):
    csv_path = _run(tmp_path)
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    base_settings, base_params = config_from_mapping(sources.base_mapping)

    assert sources.run_path == ""
    assert config_to_mapping(settings, params) == config_to_mapping(base_settings, base_params)


def test_the_run_file_wins_over_the_base(tmp_path):
    csv_path = _run(tmp_path, {
        **_params(estimationFrame="complementary"),
        **_tool(reference_trajectory={"path": "/data/ts.tum", "time_offset_s": 1786388866.07}),
    })
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)

    assert params.estimation_frame == "complementary"
    assert settings.reference_trajectory.time_offset_s == 1786388866.07
    # ...and everything it did not name is still the base's.
    base_params = config_from_mapping(sources.base_mapping)[1]
    assert params.complementary_correction == base_params.complementary_correction
    assert params.scale_smoothing_window_size == base_params.scale_smoothing_window_size


def test_it_can_be_ignored_for_one_replay(tmp_path):
    """For sweeping one configuration across runs that each carry their own."""
    csv_path = _run(tmp_path, _params(estimationFrame="complementary"))
    _, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH,
                                             use_run_config=False)
    assert sources.run_path == ""
    assert params.estimation_frame == "lidar"


def test_a_run_file_may_not_name_another_run(tmp_path):
    """The run is the directory the file is in; naming another is at best noise."""
    csv_path = _run(tmp_path, _tool(input_path="/somewhere/else", base_dir="/elsewhere"))
    settings, _, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)

    assert settings.input_path == "" and settings.base_dir == ""
    assert set(sources.ignored_keys) == {"input_path", "base_dir"}
    assert "input_path" in sources.describe()


def test_the_layering_is_reported_rather_than_silent(tmp_path):
    """A run directory that changes what a replay does must never do it quietly."""
    csv_path = _run(tmp_path, _params(estimationFrame="complementary"))
    _, _, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)

    described = sources.describe()
    assert os.path.basename(DEFAULT_CONFIG_PATH) in described
    assert RUN_CONFIG_NAME in described
    assert "complementaryOdom" in described        # not the raw "/**"


# ---------------------------------------------------------------------------
# Writing one
# ---------------------------------------------------------------------------

def test_saving_writes_only_what_differs(tmp_path):
    from dataclasses import replace

    csv_path = _run(tmp_path)
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    path = save_run_config(csv_path, settings,
                           replace(params, estimation_frame="complementary"),
                           sources.base_mapping)

    written = yaml.safe_load(open(path))
    assert written == {"/**": {"ros__parameters": {
        "complementaryOdom": {"estimationFrame": "complementary"}}}}


def test_what_was_saved_is_what_comes_back(tmp_path):
    from dataclasses import replace

    csv_path = _run(tmp_path)
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    edited_settings = settings.evolve(
        reference_trajectory=replace(settings.reference_trajectory,
                                     path="/data/ts.tum", time_offset_s=1786388866.07))
    edited_params = replace(params, estimation_frame="complementary",
                            scale_smoothing_window_size=7)
    save_run_config(csv_path, edited_settings, edited_params, sources.base_mapping)

    reloaded_settings, reloaded_params, _ = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    assert config_to_mapping(reloaded_settings, reloaded_params) == \
        config_to_mapping(edited_settings, edited_params)


def test_saving_twice_writes_the_same_file(tmp_path):
    """Values that came *from* the run file must not accumulate on a re-save."""
    from dataclasses import replace

    csv_path = _run(tmp_path)
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    path = save_run_config(csv_path, settings,
                           replace(params, estimation_frame="complementary"),
                           sources.base_mapping)
    first = open(path).read()

    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    save_run_config(csv_path, settings, params, sources.base_mapping)
    assert open(path).read() == first


def test_a_saved_run_file_never_names_a_run(tmp_path):
    csv_path = _run(tmp_path)
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    path = save_run_config(csv_path,
                           settings.evolve(input_path="/somewhere", base_dir="/else"),
                           params, sources.base_mapping)

    written = yaml.safe_load(open(path)) or {}
    assert "input_path" not in written.get("replay_scale_tool", {})
    assert "base_dir" not in written.get("replay_scale_tool", {})


def test_the_file_lands_where_the_next_replay_looks(tmp_path):
    csv_path = _run(tmp_path)
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    assert save_run_config(csv_path, settings, params, sources.base_mapping) == \
        run_config_path(csv_path) == str(tmp_path / RUN_CONFIG_NAME)


def test_the_mounts_survive_a_round_trip_through_a_run_file(tmp_path):
    """The reason per-run configuration exists: a mount is a fact about a run."""
    from dataclasses import replace
    from replay_scale.settings import RecordedOdometrySettings

    mount = np.eye(4)
    mount[:3, 3] = [-0.31, 0.0, 0.159]
    csv_path = _run(tmp_path)
    settings, params, sources = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    settings = settings.evolve(
        recorded_odometry=RecordedOdometrySettings(extrinsic=mount),
        complementary_source=replace(settings.complementary_source,
                                     path="/tmp/vo.tum", extrinsic="run"))
    save_run_config(csv_path, settings, params, sources.base_mapping)

    reloaded, _, _ = load_config_for_run(csv_path, DEFAULT_CONFIG_PATH)
    np.testing.assert_allclose(reloaded.recorded_odometry.extrinsic, mount, atol=1e-9)
    assert reloaded.complementary_source.extrinsic == "run"
