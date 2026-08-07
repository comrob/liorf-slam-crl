"""Which run a replay opens when nothing is passed on the command line.

Both frontends go through one resolver, so the GUI and the CLI cannot disagree
about what "the run" means. The precedence is what these pin.
"""

import os

import pytest

from replay_scale.io.frames_csv import CSV_NAME
from replay_scale.io.paths import DEFAULT_BASE_DIR, resolve_run_input
from replay_scale.settings import ReplayToolSettings


def _run_dir(base, name):
    """A directory that looks like a run: it holds the replay CSV."""
    path = base / name
    path.mkdir(parents=True)
    (path / CSV_NAME).write_text("stub\n", encoding="utf-8")
    return path


@pytest.fixture
def runs(tmp_path):
    """Two runs, plus a 'latest' pointing at neither of them."""
    older = _run_dir(tmp_path, "run_20260101_000000")
    newer = _run_dir(tmp_path, "run_20260202_000000")
    os.utime(older, (1_000_000, 1_000_000))
    os.utime(newer, (2_000_000, 2_000_000))
    return tmp_path, older, newer


def test_the_config_names_the_run_when_the_command_line_does_not(runs):
    base, older, _ = runs
    settings = ReplayToolSettings(input_path=str(older), base_dir=str(base))
    assert resolve_run_input(settings) == str(older / CSV_NAME)


def test_an_explicit_path_beats_the_config(runs):
    base, older, newer = runs
    settings = ReplayToolSettings(input_path=str(older), base_dir=str(base))
    assert resolve_run_input(settings, str(newer)) == str(newer / CSV_NAME)


def test_a_bare_run_name_resolves_against_the_base_directory(runs):
    """What anyone writes in a config; relative to the shell's cwd it is nonsense."""
    base, older, _ = runs
    settings = ReplayToolSettings(input_path=older.name, base_dir=str(base))
    assert resolve_run_input(settings) == str(older / CSV_NAME)


def test_latest_ignores_a_configured_input_path(runs):
    """--latest must mean latest, not "whatever the config pinned"."""
    base, older, newer = runs
    settings = ReplayToolSettings(input_path=str(older), base_dir=str(base))
    assert resolve_run_input(settings, latest=True) == str(newer / CSV_NAME)


def test_an_empty_input_path_falls_back_to_the_newest_run(runs):
    base, _, newer = runs
    settings = ReplayToolSettings(base_dir=str(base))
    assert resolve_run_input(settings) == str(newer / CSV_NAME)


def test_a_latest_directory_wins_over_mtime(runs):
    """The node's own symlink is the more deliberate answer to "the last run"."""
    base, _, _ = runs
    latest = _run_dir(base, "latest")
    settings = ReplayToolSettings(base_dir=str(base))
    assert resolve_run_input(settings) == str(latest / CSV_NAME)


def test_a_base_dir_argument_overrides_the_config(runs, tmp_path):
    base, _, _ = runs
    other = tmp_path / "elsewhere"
    only = _run_dir(other, "run_20260303_000000")
    settings = ReplayToolSettings(base_dir=str(base))
    assert resolve_run_input(settings, base_dir=str(other)) == str(only / CSV_NAME)


def test_an_unset_base_dir_means_the_built_in_default():
    """Empty is not "the current directory"; the literal lives in io.paths only."""
    settings = ReplayToolSettings()
    assert settings.base_dir == ""
    with pytest.raises(FileNotFoundError, match="No run directories|not found"):
        resolve_run_input(settings, base_dir=os.path.join(DEFAULT_BASE_DIR, "nope"))


def test_a_missing_configured_run_is_reported_not_silently_replaced(runs):
    base, _, _ = runs
    settings = ReplayToolSettings(input_path=str(base / "run_that_never_was"),
                                  base_dir=str(base))
    with pytest.raises(FileNotFoundError):
        resolve_run_input(settings)
