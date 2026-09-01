"""What the viewer remembers about each run it has worked on.

The point of the whole file: a setting that belongs to one run -- its reference
trajectory above all -- must stay with that run and reach no other. Switching
runs used to mean either losing it or spreading it, depending on which
configuration happened to be the base at the time.
"""

import os

import pytest

from replay_scale.gui.run_state import (
    REPLAYED_KEY,
    config_for_run,
    forget_run,
    load_run_state,
    remembered_runs,
    run_key,
    save_run_state,
    state_path_for,
    was_replayed,
)
from replay_scale.settings import DEFAULT_CONFIG_PATH, RUN_CONFIG_NAME, load_config

yaml = pytest.importorskip("yaml")


def _run(tmp_path, name="run_1", run_config=None):
    """A run directory with a replay CSV in it, and optionally its own config."""
    run = tmp_path / name
    run.mkdir(parents=True, exist_ok=True)
    (run / "scale_replay_frames.csv").write_text("")
    if run_config is not None:
        (run / RUN_CONFIG_NAME).write_text(yaml.safe_dump(run_config))
    return str(run)


def _config(**overrides):
    settings, params = load_config(DEFAULT_CONFIG_PATH)
    return settings.evolve(**overrides), params


def _inherited(run):
    """What the config files say about this run: what a delta is written against.

    Exactly what the viewer passes -- ``ConfigSources.inherited`` -- so these
    exercise the layering the window actually builds.
    """
    from replay_scale.settings import config_sources_for_run

    return config_sources_for_run(os.path.join(run, "scale_replay_frames.csv")).inherited


def _with_reference(path):
    settings, params = _config()
    reference = settings.reference_trajectory
    reference.path = path
    return settings, params


# ---------------------------------------------------------------------------
# One run's settings, and only that run's
# ---------------------------------------------------------------------------

def test_each_run_keeps_its_own_reference(tmp_path):
    state = str(tmp_path / "state")
    a, b = _run(tmp_path, "run_a"), _run(tmp_path, "run_b")
    save_run_state(a, *_with_reference("/data/a.tum"), inherited=_inherited(a),
                   directory=state)
    save_run_state(b, *_with_reference("/data/b.tum"), inherited=_inherited(b),
                   directory=state)

    got_a, _, _ = config_for_run(os.path.join(a, "scale_replay_frames.csv"), directory=state)
    got_b, _, _ = config_for_run(os.path.join(b, "scale_replay_frames.csv"), directory=state)
    assert got_a.reference_trajectory.path == "/data/a.tum"
    assert got_b.reference_trajectory.path == "/data/b.tum"


def test_a_run_nobody_configured_gets_what_the_config_files_say(tmp_path):
    """The other half of it: nothing spreads to a run that was never given one."""
    state = str(tmp_path / "state")
    a, fresh = _run(tmp_path, "run_a"), _run(tmp_path, "run_fresh")
    save_run_state(a, *_with_reference("/data/a.tum"), inherited=_inherited(a),
                   directory=state)

    settings, _, sources = config_for_run(
        os.path.join(fresh, "scale_replay_frames.csv"), directory=state)
    assert settings.reference_trajectory.path == ""
    assert sources.state_path == ""


def test_what_is_written_is_a_delta_so_shared_defaults_still_move(tmp_path):
    """A copy would pin every method setting the day the run was first opened."""
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    settings, params = _with_reference("/data/gt.tum")
    save_run_state(run, settings, params, inherited=_inherited(run), directory=state)

    remembered = load_run_state(run, directory=state)
    tool = remembered["replay_scale_tool"]
    assert set(tool) == {"reference_trajectory", "input_path"}
    assert "/**" not in remembered              # not one parameter was pinned


def test_the_runs_own_file_is_still_underneath(tmp_path):
    """Three layers: the base config, the run's file, then what you applied."""
    state = str(tmp_path / "state")
    run = _run(tmp_path, run_config={"replay_scale_tool": {"correction_mode": "translation"}})
    csv_path = os.path.join(run, "scale_replay_frames.csv")

    # What the viewer applies is the run's resolved configuration with an edit
    # in it, not a fresh copy of the base -- so the run's own file is not
    # overwritten by defaults on its way through.
    settings, params, _ = config_for_run(csv_path, directory=state)
    settings.reference_trajectory.path = "/data/gt.tum"
    save_run_state(run, settings, params, inherited=_inherited(run), directory=state)

    settings, _, sources = config_for_run(csv_path, directory=state)
    assert settings.reference_trajectory.path == "/data/gt.tum"   # from the state
    assert settings.correction_mode == "translation"              # from the run's file
    assert sources.run_path and sources.state_path
    assert "replay_scale_tool" in sources.state_sections
    # ...and only the edit was written: the run's file still owns the rest.
    assert set(load_run_state(run, state)["replay_scale_tool"]) == {
        "reference_trajectory", "input_path"}


def test_a_link_and_what_it_points_at_are_one_run(tmp_path):
    """Replaying through ``latest`` and through the directory is the same run."""
    state = str(tmp_path / "state")
    run = _run(tmp_path, "run_1")
    link = str(tmp_path / "latest")
    os.symlink(run, link)

    save_run_state(link, *_with_reference("/data/gt.tum"), inherited=_inherited(run),
                   directory=state)
    settings, _, _ = config_for_run(os.path.join(run, "scale_replay_frames.csv"),
                                    directory=state)
    assert settings.reference_trajectory.path == "/data/gt.tum"
    assert state_path_for(link, state) == state_path_for(run, state)
    assert run_key(link) == run_key(run)


# ---------------------------------------------------------------------------
# Which runs are listed as replayed
# ---------------------------------------------------------------------------

def test_only_what_was_replayed_is_listed_as_replayed(tmp_path):
    """A run that was previewed and configured is remembered, not listed."""
    state = str(tmp_path / "state")
    previewed, replayed = _run(tmp_path, "run_p"), _run(tmp_path, "run_r")

    save_run_state(previewed, *_with_reference("/data/p.tum"),
                   inherited=_inherited(previewed), directory=state, replayed=False)
    save_run_state(replayed, *_config(), inherited=_inherited(replayed),
                   directory=state, replayed=True)

    listed = [path for path, _ in remembered_runs(state)]
    assert listed == [replayed]
    assert not was_replayed(previewed, state) and was_replayed(replayed, state)
    # ...but its settings came back all the same.
    assert load_run_state(previewed, state)["replay_scale_tool"]["reference_trajectory"]


def test_a_run_does_not_stop_having_been_replayed(tmp_path):
    """Previewing it again is not an un-replaying."""
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    save_run_state(run, *_config(), inherited=_inherited(run), directory=state,
                   replayed=True)
    save_run_state(run, *_with_reference("/data/gt.tum"), inherited=_inherited(run),
                   directory=state, replayed=False)
    assert was_replayed(run, state)


def test_the_replayed_flag_stays_out_of_the_configuration(tmp_path):
    """It is a fact about the viewer's history, not a setting to merge."""
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    save_run_state(run, *_config(), inherited=_inherited(run), directory=state,
                   replayed=True)
    assert REPLAYED_KEY not in load_run_state(run, state)
    _, _, sources = config_for_run(os.path.join(run, "scale_replay_frames.csv"),
                                   directory=state)
    assert REPLAYED_KEY not in sources.merged


def test_merely_looking_at_a_run_writes_nothing(tmp_path):
    """Clicking down a list of runs previews each one; none of that is news."""
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    assert save_run_state(run, *_config(), inherited=_inherited(run), directory=state,
                          replayed=False) is None
    assert not os.path.isdir(state) or os.listdir(state) == []


def test_clearing_a_setting_is_written_even_though_it_is_empty(tmp_path):
    """The file exists, so the erasure has to reach it."""
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    inherited = _inherited(run)
    save_run_state(run, *_with_reference("/data/gt.tum"), inherited=inherited,
                   directory=state)
    assert load_run_state(run, state)["replay_scale_tool"].get("reference_trajectory")

    save_run_state(run, *_config(), inherited=inherited, directory=state)
    assert "reference_trajectory" not in load_run_state(run, state)["replay_scale_tool"]


def test_a_run_that_is_gone_is_not_offered(tmp_path):
    """Its file is kept, though: an unmounted dataset comes back configured."""
    state = str(tmp_path / "state")
    run = _run(tmp_path, "run_gone")
    save_run_state(run, *_with_reference("/data/gt.tum"), inherited=_inherited(run),
                   directory=state, replayed=True)
    os.remove(os.path.join(run, "scale_replay_frames.csv"))
    os.rmdir(run)

    assert remembered_runs(state) == []
    assert os.path.isfile(state_path_for(run, state))


def test_forgetting_a_run_puts_the_config_files_back_in_charge(tmp_path):
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    save_run_state(run, *_with_reference("/data/gt.tum"), inherited=_inherited(run),
                   directory=state, replayed=True)

    assert forget_run(run, state) is True
    assert forget_run(run, state) is False
    settings, _, _ = config_for_run(os.path.join(run, "scale_replay_frames.csv"),
                                    directory=state)
    assert settings.reference_trajectory.path == ""


def test_an_unreadable_file_costs_what_it_remembered_and_no_more(tmp_path):
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    os.makedirs(state, exist_ok=True)
    with open(state_path_for(run, state), "w", encoding="utf-8") as fh:
        fh.write("this: is: not: a config\n")

    assert load_run_state(run, state) == {}
    assert remembered_runs(state) == []
    settings, _, _ = config_for_run(os.path.join(run, "scale_replay_frames.csv"),
                                    directory=state)
    assert settings.reference_trajectory.path == ""


def test_the_viewers_own_layer_is_invisible_to_the_cli(tmp_path):
    """A replay stays reproducible from what is written down.

    ``settings.load_config_for_run`` is what the CLI resolves a run with, and it
    knows nothing about any of this.
    """
    from replay_scale.settings import load_config_for_run

    state = str(tmp_path / "state")
    run = _run(tmp_path)
    save_run_state(run, *_with_reference("/data/gt.tum"), inherited=_inherited(run),
                   directory=state, replayed=True)

    settings, _, sources = load_config_for_run(
        os.path.join(run, "scale_replay_frames.csv"), DEFAULT_CONFIG_PATH)
    assert settings.reference_trajectory.path == ""
    assert sources.state_path == ""


def test_where_the_files_live(tmp_path, monkeypatch):
    """Under the state directory, never in the run folder itself.

    A run directory may sit on a read-only dataset, and what it holds is the
    recording -- not one viewer's opinion about it.
    """
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    run = _run(tmp_path)
    written = save_run_state(run, *_with_reference("/data/gt.tum"), replayed=True)
    assert written.startswith(str(tmp_path / "state" / "replay_scale" / "runs"))
    assert sorted(os.listdir(run)) == ["scale_replay_frames.csv"]


def test_a_state_directory_that_cannot_be_written_is_not_fatal(tmp_path):
    run = _run(tmp_path)
    blocker = tmp_path / "not_a_directory"
    blocker.write_text("")
    assert save_run_state(run, *_config(), directory=str(blocker / "runs"),
                          replayed=True) is None


def test_the_file_is_a_config_fragment_anyone_can_read(tmp_path):
    """No second format: it is the layout every other config file here uses."""
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    path = save_run_state(run, *_with_reference("/data/gt.tum"),
                          inherited=_inherited(run), directory=state, replayed=True)

    raw = yaml.safe_load(open(path, encoding="utf-8").read())
    assert raw["replay_scale_tool"]["reference_trajectory"]["path"] == "/data/gt.tum"
    assert raw["replay_scale_tool"]["input_path"] == run_key(run)
    assert open(path, encoding="utf-8").read().startswith("#")   # and it says what it is


def test_the_viewers_forced_scale_mode_is_not_pinned_on_the_run(tmp_path):
    """The viewer draws per-frame vectors, so it forces "estimated" to get them.

    Writing that back would hand a viewer artifact to the next replay.
    """
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    settings, params = _config(scale_mode="estimated", base_dir="/somewhere/else")
    save_run_state(run, settings, params, inherited=_inherited(run), directory=state,
                   replayed=True)

    tool = load_run_state(run, state)["replay_scale_tool"]
    assert "scale_mode" not in tool and "base_dir" not in tool


def test_without_a_base_to_inherit_from_the_whole_configuration_is_written(tmp_path):
    """Still restores correctly -- it just stops following shared defaults."""
    state = str(tmp_path / "state")
    run = _run(tmp_path)
    save_run_state(run, *_with_reference("/data/gt.tum"), inherited=None,
                   directory=state, replayed=True)

    remembered = load_run_state(run, state)
    assert "/**" in remembered                     # every parameter, pinned
    settings, _, _ = config_for_run(os.path.join(run, "scale_replay_frames.csv"),
                                    directory=state)
    assert settings.reference_trajectory.path == "/data/gt.tum"
