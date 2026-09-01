"""Which runs the viewer offers, under which name, and in what order.

A base directory accumulates every ``run_<timestamp>`` the node ever wrote, and
a handful of symlinks someone made when a recording turned out to matter. The
links are the shortlist, so they are what the list opens on -- and a run reached
through several of them is still one run, and gets one row.
"""

import os

from replay_scale.gui.sources import find_run_dirs, list_runs


def _run(base, name, mtime=None):
    run = os.path.join(base, name)
    os.makedirs(run, exist_ok=True)
    open(os.path.join(run, "scale_replay_frames.csv"), "w").close()
    if mtime is not None:
        os.utime(run, (mtime, mtime))
    return run


def test_named_runs_come_first_then_the_rest_newest_first(tmp_path):
    base = str(tmp_path)
    _run(base, "run_old", mtime=1_000_000)
    named = _run(base, "run_named", mtime=3_000_000)
    _run(base, "run_middle", mtime=2_000_000)
    os.symlink(named, os.path.join(base, "the_good_one"))

    entries = list_runs(base)
    assert [entry.name for entry in entries] == [
        "the_good_one", "run_middle", "run_old"]
    assert [entry.is_link for entry in entries] == [True, False, False]


def test_a_run_reached_by_several_names_gets_one_row(tmp_path):
    """Three rows for one run is what makes a list impossible to pick from."""
    base = str(tmp_path)
    run = _run(base, "run_1")
    os.symlink(run, os.path.join(base, "latest"))
    os.symlink(run, os.path.join(base, "the_good_one"))

    entries = list_runs(base)
    assert len(entries) == 1
    # Under the name someone chose: `latest` moves, and the timestamp is what
    # the node called it. The others travel with the row.
    assert entries[0].name == "the_good_one"
    assert sorted(entries[0].aliases) == ["latest", "run_1"]
    assert "also here as" in entries[0].describe()


def test_a_link_says_what_it_points_at(tmp_path):
    base = str(tmp_path)
    run = _run(base, "run_1")
    plain = _run(base, "run_2")
    os.symlink(run, os.path.join(base, "latest"))

    link, directory = list_runs(base)
    assert link.is_link and link.target == os.path.realpath(run)
    # A plain directory has nothing to point at, and says so with an empty string.
    assert directory.path == plain and directory.target == ""


def test_only_directories_holding_a_replay_csv_are_offered(tmp_path):
    base = str(tmp_path)
    _run(base, "run_1")
    os.makedirs(os.path.join(base, "not_a_run"))
    open(os.path.join(base, "loose_file.txt"), "w").close()

    assert [entry.name for entry in list_runs(base)] == ["run_1"]


def test_a_base_directory_that_is_not_there_offers_nothing(tmp_path):
    assert list_runs(str(tmp_path / "nowhere")) == []
    assert find_run_dirs(str(tmp_path / "nowhere")) == []


def test_the_paths_helper_agrees_with_the_entries(tmp_path):
    base = str(tmp_path)
    run = _run(base, "run_1")
    _run(base, "run_2")
    os.symlink(run, os.path.join(base, "latest"))
    assert find_run_dirs(base) == [entry.path for entry in list_runs(base)]
