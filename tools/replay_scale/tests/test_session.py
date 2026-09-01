"""What the viewer remembers between launches.

The session file is an ordinary config file, so these check the round trip and
the refusals -- a missing or unreadable one must never be fatal, since the
viewer's only use for it is convenience.
"""

import dataclasses
import os

from replay_scale.gui.session import (
    clear_session,
    load_session,
    save_session,
    session_path,
    state_dir,
)
from replay_scale.gui.app import resolve_startup
from replay_scale.settings import DEFAULT_CONFIG_PATH, load_config, save_config


def _config():
    return load_config(DEFAULT_CONFIG_PATH)


def test_the_session_lives_under_the_xdg_state_directory(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path))
    assert state_dir() == str(tmp_path / "replay_scale")
    assert os.path.dirname(session_path()) == state_dir()


def test_what_was_applied_comes_back(tmp_path):
    settings, params = _config()
    settings = settings.evolve(correction_mode="translation")
    params = dataclasses.replace(params, scale_line_history=17)
    path = str(tmp_path / "last_session.yaml")

    assert save_session(settings, params, input_path="/runs/run_7",
                        base_dir="/runs", path=path) == path
    restored_settings, restored_params = load_session(path)
    assert restored_settings.correction_mode == "translation"
    assert restored_params.scale_line_history == 17


def test_the_run_is_remembered_over_what_the_config_named(tmp_path):
    """The run on screen is the one to reopen, not the one the file named."""
    settings, params = _config()
    settings = settings.evolve(input_path="/runs/named_in_the_file", base_dir="/elsewhere")
    path = str(tmp_path / "last_session.yaml")

    save_session(settings, params, input_path="/runs/run_7", base_dir="/runs", path=path)
    restored, _ = load_session(path)
    assert restored.input_path == "/runs/run_7"
    assert restored.base_dir == "/runs"


# ---------------------------------------------------------------------------
# What is deliberately not remembered
# ---------------------------------------------------------------------------

def _with_reference(settings, path="/data/total_station.tum"):
    return settings.evolve(reference_trajectory=dataclasses.replace(
        settings.reference_trajectory, path=path, label="total station",
        time_offset_s=1786388866.07))


def test_the_reference_trajectory_is_not_remembered(tmp_path):
    """It belongs to the run it was recorded with, not to the next one opened.

    The session is the configuration carried to whatever run is loaded next, so
    a reference kept in it would be redrawn beside a run it has nothing to do
    with -- aligned, plausible, and about a different drive.
    """
    settings, params = _config()
    path = str(tmp_path / "last_session.yaml")

    save_session(_with_reference(settings), params, path=path)
    restored, _ = load_session(path)
    assert restored.reference_trajectory.path == ""
    assert restored.reference_trajectory.time_offset_s == 0.0
    # Only the run-scoped part is dropped; the method settings still come back.
    assert restored.correction_mode == settings.correction_mode


def test_a_session_file_still_holding_one_is_rewritten_without_it(tmp_path):
    """Hand-edited, or written before the reference became run-scoped.

    Ignoring it in memory would not be enough: the viewer re-reads this file as
    the base for every load, not only for the first one.
    """
    settings, params = _config()
    path = str(tmp_path / "last_session.yaml")
    save_config(path, _with_reference(settings), params)

    restored, _ = load_session(path)
    assert restored.reference_trajectory.path == ""
    on_disk, _ = load_config(path)
    assert on_disk.reference_trajectory.path == ""


def test_the_run_and_the_method_survive_that_rewrite(tmp_path):
    settings, params = _config()
    path = str(tmp_path / "last_session.yaml")
    save_config(path, _with_reference(settings).evolve(
        input_path="/runs/run_7", base_dir="/runs", correction_mode="translation"), params)

    restored, _ = load_session(path)
    assert (restored.input_path, restored.base_dir) == ("/runs/run_7", "/runs")
    assert restored.correction_mode == "translation"


def test_a_first_launch_has_nothing_to_restore(tmp_path):
    assert load_session(str(tmp_path / "nothing_here.yaml")) is None


def test_an_unreadable_session_is_ignored_rather_than_raised(tmp_path):
    path = tmp_path / "last_session.yaml"
    path.write_text("this: is: not: a config\n")
    assert load_session(str(path)) is None


def test_saving_creates_the_state_directory(tmp_path):
    settings, params = _config()
    path = str(tmp_path / "state" / "replay_scale" / "last_session.yaml")
    assert save_session(settings, params, path=path) == path
    assert os.path.isfile(path)


def test_a_session_that_cannot_be_written_is_not_fatal(tmp_path):
    """A read-only home costs the convenience, not the session."""
    settings, params = _config()
    blocker = tmp_path / "not_a_directory"
    blocker.write_text("")
    assert save_session(settings, params,
                        path=str(blocker / "last_session.yaml")) is None


def test_clearing_reports_whether_there_was_anything_to_clear(tmp_path):
    settings, params = _config()
    path = str(tmp_path / "last_session.yaml")
    save_session(settings, params, path=path)
    assert clear_session(path) is True
    assert clear_session(path) is False
    assert load_session(path) is None


# ---------------------------------------------------------------------------
# What the viewer opens with
# ---------------------------------------------------------------------------

def _remember(tmp_path, monkeypatch, **overrides):
    """Write a session under a throwaway XDG state directory."""
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    settings, params = _config()
    run = tmp_path / "runs" / "run_7"
    run.mkdir(parents=True, exist_ok=True)
    save_session(settings.evolve(**overrides), params,
                 input_path=str(run), base_dir=str(tmp_path / "runs"))
    return str(run)


def test_the_viewer_reopens_the_remembered_configuration_and_run(tmp_path, monkeypatch):
    run = _remember(tmp_path, monkeypatch)
    config_path, base_dir, initial_input = resolve_startup()
    assert config_path == session_path()
    assert base_dir == str(tmp_path / "runs")
    assert initial_input == run


def test_no_session_starts_from_the_configuration_instead(tmp_path, monkeypatch):
    _remember(tmp_path, monkeypatch)
    config_path, _, initial_input = resolve_startup(use_session=False)
    assert config_path == DEFAULT_CONFIG_PATH
    assert initial_input == ""


def test_an_explicit_configuration_file_wins_over_the_session(tmp_path, monkeypatch):
    _remember(tmp_path, monkeypatch)
    chosen = tmp_path / "mine.yaml"
    settings, params = _config()
    save_config(str(chosen), settings, params)

    config_path, _, initial_input = resolve_startup(ros_params_yaml=str(chosen))
    assert config_path == str(chosen)
    assert initial_input == ""


def test_explicit_arguments_win_over_the_remembered_run(tmp_path, monkeypatch):
    _remember(tmp_path, monkeypatch)
    other = tmp_path / "elsewhere" / "run_9"
    other.mkdir(parents=True)

    _, base_dir, initial_input = resolve_startup(input_path=str(other),
                                                 base_dir=str(tmp_path / "elsewhere"))
    assert initial_input == str(other)
    assert base_dir == str(tmp_path / "elsewhere")


def test_a_remembered_run_that_is_gone_falls_back_to_the_newest(tmp_path, monkeypatch):
    """Deleting the run you last looked at must not open the viewer on an error."""
    run = _remember(tmp_path, monkeypatch)
    os.rmdir(run)
    _, base_dir, initial_input = resolve_startup()
    assert initial_input == ""
    assert base_dir == str(tmp_path / "runs")

