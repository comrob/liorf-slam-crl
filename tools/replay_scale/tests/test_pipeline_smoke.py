"""Smoke tests for the pipeline seam, against a real recorded run.

Skipped when the run directory is not present, so the suite still runs on a
machine without the logs. Point REPLAY_SCALE_TEST_RUN at any run_* folder
containing scale_replay_frames.csv to exercise these.
"""

import os
import pathlib

import pytest

from replay_scale import DEFAULT_CONFIG_PATH, load_config, run_replay
from replay_scale.plotting import build_trajectory_figure, complementary_only_curve

RUN_DIR = os.environ.get(
    "REPLAY_SCALE_TEST_RUN",
    os.path.expanduser("~/.ros/lili_logs/run_20260805_192954_kdtree_lm"))
CSV_PATH = os.path.join(RUN_DIR, "scale_replay_frames.csv")

pytestmark = pytest.mark.skipif(
    not os.path.isfile(CSV_PATH), reason=f"no replay CSV at {CSV_PATH}")


@pytest.fixture(scope="module")
def config():
    return load_config(DEFAULT_CONFIG_PATH)


@pytest.fixture(scope="module")
def result(config):
    settings, params = config
    return run_replay(CSV_PATH, settings, params, write=False)


def test_dry_run_writes_nothing(config, tmp_path):
    settings, params = config
    result = run_replay(CSV_PATH, settings.evolve(output_dir=str(tmp_path)),
                        params, write=False)
    assert result.written == []
    assert list(tmp_path.rglob("*")) == []
    assert result.n_frames > 0
    assert result.replays


def test_write_produces_the_documented_layout(config, tmp_path):
    settings, params = config
    result = run_replay(CSV_PATH, settings.evolve(output_dir=str(tmp_path)),
                        params, write=True)
    assert result.written
    for path in result.written:
        assert os.path.isfile(path)
    traj = pathlib.Path(result.traj_dir)
    assert (traj / "trajectory_recorded_effective.tum").is_file()
    assert traj.parent.name == settings.output_subdir


def test_settings_edits_select_the_replays(config):
    settings, params = config
    result = run_replay(CSV_PATH, settings.evolve(scale_mode="fixed", scales=[0.8, 1.2]),
                        params, write=False)
    assert [tag for tag, _ in result.replays] == ["scale_0.8", "scale_1.2"]


def test_progress_callback_receives_lines(config):
    settings, params = config
    lines = []
    run_replay(CSV_PATH, settings, params, write=False, on_progress=lines.append)
    assert lines
    assert lines[0].startswith("Loaded ")


def test_validation_reports_drift(config):
    settings, params = config
    result = run_replay(CSV_PATH, settings.evolve(validate=True), params, write=False)
    assert result.validation is not None
    assert result.validation.drift.size == result.n_frames
    assert result.validation.rms >= 0.0


def test_figure_builds_from_memory(result, config):
    _, params = config
    curves = result.curves() + [complementary_only_curve(result.frames, params)]
    fig = build_trajectory_figure(curves, title="test")
    assert len(fig.axes[0].lines) == len(curves)


def test_estimated_mode_needs_params(config):
    settings, _ = config
    with pytest.raises(ValueError):
        run_replay(CSV_PATH, settings.evolve(scale_mode="estimated"), None, write=False)


def test_rejects_unknown_modes(config):
    settings, _ = config
    with pytest.raises(ValueError):
        settings.evolve(scale_mode="nonsense").validated()
    with pytest.raises(ValueError):
        settings.evolve(correction_mode="nonsense").validated()
