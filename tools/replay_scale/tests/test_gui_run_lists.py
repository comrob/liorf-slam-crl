"""The viewer's two run lists, and what switching between runs costs.

Qt-level, like ``test_gui_startup.py``, because the behaviour under test is the
window's: which list a run lands in, what a click on one does, and what comes
back when you return to a run you already opened. None of it is visible below
the window.

Four things are pinned here, all of them things the viewer used to get wrong:

* a configuration applied to one run does not reach another, and *is* still
  there when you come back to the run it was applied to;
* picking a run reads it, and *nothing* in the run panel replays: the estimator
  runs on "Apply & replay" and on no other gesture;
* the two lists are disjoint, so every row names a run exactly once;
* a run already replayed comes back from memory, without running anything.
"""

import csv
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# A plain try/except rather than importorskip: raising Skipped while the module
# is being imported aborts collection under the ROS launch_testing plugin,
# which imports every test file looking for an entry point.
try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication

    from replay_scale.gui.main_window import MainWindow
except ImportError:                             # pragma: no cover
    QApplication = MainWindow = Qt = None

pytestmark = pytest.mark.skipif(QApplication is None, reason="PySide6 not installed")

_AXES = ("vx", "vy", "vz", "wx", "wy", "wz")
_POSE = ("tx", "ty", "tz", "qx", "qy", "qz", "qw")


def _write_frames_csv(path, n=6, dt=0.1, step=0.2):
    """A minimal but real ``scale_replay_frames.csv``: a run driving straight.

    Small enough to replay in milliseconds, which is what makes it usable in a
    test that replays several times.
    """
    columns = ["time", "scale_replay/stamp/lidar_prev_s", "scale_replay/dt/scan_s",
               "scale_replay/flags/degeneracy_detected",
               "scale_replay/flags/has_degeneracy_basis",
               "scale_replay/flags/has_complementary_twist",
               "scale_replay/scale/applied", "scale_replay/basis/size",
               "scale_replay/complementary/dt_s"]
    columns += [f"scale_replay/lidar_increment/{a}" for a in _AXES]
    columns += [f"scale_replay/complementary_twist/{a}" for a in _AXES]
    for prefix in ("pose_prev", "pose_optimized", "pose_effective"):
        columns += [f"scale_replay/{prefix}/{a}" for a in _POSE]

    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        for k in range(n):
            row = dict.fromkeys(columns, 0.0)
            row["time"] = 1000.0 + k * dt
            row["scale_replay/stamp/lidar_prev_s"] = 1000.0 + (k - 1) * dt
            row["scale_replay/dt/scan_s"] = dt
            row["scale_replay/flags/has_complementary_twist"] = 1.0
            row["scale_replay/scale/applied"] = 1.0
            row["scale_replay/complementary/dt_s"] = dt
            row["scale_replay/lidar_increment/vx"] = step / dt
            row["scale_replay/complementary_twist/vx"] = step / dt
            for prefix in ("pose_prev", "pose_optimized", "pose_effective"):
                row[f"scale_replay/{prefix}/tx"] = k * step
                row[f"scale_replay/{prefix}/qw"] = 1.0
            writer.writerow(row)
    return path


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def _isolated_state(tmp_path, monkeypatch):
    """Never touch the developer's own remembered runs."""
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))


@pytest.fixture
def logs(tmp_path):
    """A base directory with two runs and a symlink naming one of them."""
    base = tmp_path / "logs"
    base.mkdir()
    for name in ("run_a", "run_b"):
        (base / name).mkdir()
        _write_frames_csv(str(base / name / "scale_replay_frames.csv"))
    os.symlink(str(base / "run_b"), str(base / "the_good_one"))
    return str(base)


def _settle(app, window, timeout=30.0):
    """Run the event loop until the window is not loading anything."""
    import time

    deadline = time.time() + timeout
    while time.time() < deadline:
        app.processEvents()
        if window._thread is None:
            return True
        time.sleep(0.005)
    raise AssertionError("a load never finished")


def _rows(widget):
    return [widget.item(row) for row in range(widget.count())]


def _listed(widget):
    """The runs a list offers, as resolved paths."""
    return {os.path.realpath(item.data(Qt.UserRole)) for item in _rows(widget)
            if item.data(Qt.UserRole)}


def _row_for(widget, path):
    for item in _rows(widget):
        listed = item.data(Qt.UserRole)
        if listed and os.path.realpath(listed) == os.path.realpath(path):
            return item
    raise AssertionError(f"{path} is not listed")


def _window(app, logs, initial=""):
    window = MainWindow(base_dir=logs, initial_input=initial)
    _settle(app, window)
    return window


def _pick(app, window, path):
    """Pick a run the way a person does: click its row in whichever list holds it."""
    for widget in (window._replayed, window._runs):
        for item in _rows(widget):
            listed = item.data(Qt.UserRole)
            if listed and os.path.realpath(listed) == os.path.realpath(path):
                widget.setCurrentItem(item)
                widget.itemClicked.emit(item)
                _settle(app, window)
                return
    raise AssertionError(f"{path} is in neither list")


def _replay(app, window, path=None):
    """Replay a run the way a person does: pick it, then Apply & replay."""
    if path is not None:
        _pick(app, window, path)
    window._params_panel.applied.emit()
    _settle(app, window)


# ---------------------------------------------------------------------------
# The lists themselves
# ---------------------------------------------------------------------------

def test_named_runs_come_first_and_in_bold(app, logs):
    """A symlink is a run someone stopped to name; the rest is what was recorded."""
    window = _window(app, logs)
    rows = [item for item in _rows(window._runs) if item.data(Qt.UserRole)]

    # The active mark is a prefix on the row it is on; the name is the rest.
    assert [item.text().lstrip("▶ ").strip() for item in rows][0] == "the_good_one"
    assert rows[0].font().bold()
    assert not any(item.font().bold() for item in rows[1:])


def test_the_replayed_list_starts_empty(app, logs, tmp_path):
    """Nothing has been through the tool yet, and the row says so."""
    empty = tmp_path / "no_runs"
    empty.mkdir()
    window = MainWindow(base_dir=str(empty), initial_input="")
    _settle(app, window)
    assert [item.data(Qt.UserRole) for item in _rows(window._replayed)] == [None]


def test_a_run_moves_from_one_list_to_the_other_when_it_is_replayed(app, logs):
    """The two never overlap: that is what makes picking one unambiguous."""
    window = _window(app, logs)
    run_a = os.path.realpath(os.path.join(logs, "run_a"))
    assert _listed(window._replayed) == set()
    assert run_a in _listed(window._runs)

    _replay(app, window, os.path.join(logs, "run_a"))
    assert _listed(window._replayed) == {run_a}
    assert run_a not in _listed(window._runs)
    assert not _listed(window._replayed) & _listed(window._runs)


def test_a_run_only_previewed_stays_in_the_unreplayed_list(app, logs):
    window = _window(app, logs)
    _pick(app, window, os.path.join(logs, "run_b"))

    assert window._data.preview
    assert _listed(window._replayed) == set()
    assert os.path.realpath(os.path.join(logs, "run_b")) in _listed(window._runs)


# ---------------------------------------------------------------------------
# What a click costs
# ---------------------------------------------------------------------------

def test_picking_a_run_reads_it_instead_of_replaying_it(app, logs):
    """The estimator never runs, so there are no per-frame views to scrub."""
    window = _window(app, logs)
    _pick(app, window, os.path.join(logs, "run_b"))

    assert window._data.preview
    assert window._data.geometries == [] and window._data.views == []
    # ...but there is something to look at: what the run recorded.
    assert [label for label, _, _ in window._data.curves]
    assert "preview" in window.statusBar().currentMessage()


def test_replaying_gives_the_frames_a_preview_could_not(app, logs):
    window = _window(app, logs)
    _replay(app, window, os.path.join(logs, "run_b"))

    assert not window._data.preview
    assert window._data.n_frames > 0 and window._data.geometries


def test_nothing_in_the_run_panel_replays(app, logs):
    """Every gesture the list offers is a read; ten seconds is never one click away."""
    window = _window(app, logs)
    for path in (os.path.join(logs, "run_a"), os.path.join(logs, "run_b")):
        _pick(app, window, path)
        assert window._data.preview
    # Including the button beside them, and a double-click on a row.
    window._preview_button.click()
    _settle(app, window)
    assert window._data.preview
    window._runs.itemDoubleClicked.emit(_row_for(window._runs, os.path.join(logs, "run_a")))
    _settle(app, window)
    assert window._data.preview


def test_a_run_already_opened_comes_back_without_loading_anything(app, logs):
    """The reason switching runs stopped costing a replay each way."""
    window = _window(app, logs)
    _replay(app, window, os.path.join(logs, "run_a"))
    replayed = window._data

    _pick(app, window, os.path.join(logs, "run_b"))
    _pick(app, window, os.path.join(logs, "run_a"))

    assert window._thread is None            # nothing was started at all
    assert window._data is replayed          # the very same replay, not a new one


def test_apply_replays_the_previewed_run_even_with_nothing_edited(app, logs):
    """Which is how a preview becomes a replay: there is no other gesture."""
    window = _window(app, logs)
    _pick(app, window, os.path.join(logs, "run_a"))
    assert window._data.preview

    _replay(app, window)
    assert not window._data.preview and window._data.geometries


def test_picking_in_one_list_clears_the_other(app, logs):
    """One selection, so the button is never ambiguous about its target."""
    window = _window(app, logs)
    _replay(app, window, os.path.join(logs, "run_a"))        # now in the replayed list
    assert window._replayed.currentItem() is not None

    _pick(app, window, os.path.join(logs, "run_b"))
    assert window._replayed.currentItem() is None
    assert os.path.realpath(window._selected) == os.path.realpath(os.path.join(logs, "run_b"))


def test_reading_a_run_does_not_rebuild_the_lists(app, logs):
    """Rebuilding drops every row and the selection with it.

    Doing that on each read -- and a read happens every time a run is picked --
    is what made clicking around the lists feel like fighting them.
    """
    window = _window(app, logs)
    window._runs.item(0).setData(Qt.UserRole + 1, "sentinel")

    _pick(app, window, os.path.join(logs, "run_a"))
    _pick(app, window, os.path.join(logs, "run_b"))
    assert window._runs.item(0).data(Qt.UserRole + 1) == "sentinel"

    # A replay does move a run between the lists, and then they are rebuilt.
    _replay(app, window)
    assert window._runs.item(0).data(Qt.UserRole + 1) is None


def test_clicking_the_run_already_on_screen_does_not_reload_it(app, logs):
    """A row that is already current emits no selection change; a click still lands."""
    window = _window(app, logs)
    _pick(app, window, os.path.join(logs, "run_a"))
    shown = window._data

    _pick(app, window, os.path.join(logs, "run_a"))
    assert window._data is shown and window._thread is None


# ---------------------------------------------------------------------------
# What each run remembers
# ---------------------------------------------------------------------------

def _set_reference(app, window, path):
    window._params_panel._reference_path.setText(path)
    window._params_panel.applied.emit()
    _settle(app, window)


def _reference(window):
    return window._data.settings.reference_trajectory.path


def test_a_reference_set_on_one_run_does_not_follow_you_to_another(app, logs, tmp_path):
    reference = tmp_path / "gt.tum"
    reference.write_text("1000.0 0 0 0 0 0 0 1\n1000.5 1 0 0 0 0 0 1\n")

    window = _window(app, logs)
    _replay(app, window, os.path.join(logs, "run_a"))
    _set_reference(app, window, str(reference))
    assert _reference(window) == str(reference)

    _pick(app, window, os.path.join(logs, "run_b"))
    assert _reference(window) == ""


def test_and_it_is_still_there_when_you_come_back(app, logs, tmp_path):
    """The other half: per run means kept, not merely not shared."""
    reference = tmp_path / "gt.tum"
    reference.write_text("1000.0 0 0 0 0 0 0 1\n1000.5 1 0 0 0 0 0 1\n")

    window = _window(app, logs)
    _replay(app, window, os.path.join(logs, "run_a"))
    _set_reference(app, window, str(reference))

    _pick(app, window, os.path.join(logs, "run_b"))
    _replay(app, window, os.path.join(logs, "run_a"))
    assert _reference(window) == str(reference)


def test_it_survives_the_viewer_being_closed(app, logs, tmp_path):
    """Which is what the state directory is for; see gui.run_state."""
    reference = tmp_path / "gt.tum"
    reference.write_text("1000.0 0 0 0 0 0 0 1\n1000.5 1 0 0 0 0 0 1\n")

    window = _window(app, logs)
    _replay(app, window, os.path.join(logs, "run_a"))
    _set_reference(app, window, str(reference))

    reopened = _window(app, logs, initial=os.path.join(logs, "run_a"))
    assert reopened._data.settings.reference_trajectory.path == str(reference)
    # ...and the run it was set on is the only one that has it.
    _pick(app, reopened, os.path.join(logs, "run_b"))
    assert _reference(reopened) == ""


def test_setting_a_reference_does_not_replay_the_run(app, logs, tmp_path):
    """It is drawn, not measured against, so it costs a redraw and no more."""
    reference = tmp_path / "gt.tum"
    reference.write_text("1000.0 0 0 0 0 0 0 1\n1000.5 1 0 0 0 0 0 1\n")

    window = _window(app, logs)
    _replay(app, window, os.path.join(logs, "run_a"))
    replayed = window._data

    _set_reference(app, window, str(reference))
    assert window._thread is None             # no load was started
    assert window._data is replayed           # the same replay, redrawn
    assert [kind for _, _, kind in window._data.curves].count("external") == 1


def test_clicking_faster_than_the_reads_finish_lands_on_the_last_click(app, logs):
    """The list and the window must not end up pointing at different runs."""
    window = _window(app, logs, initial=os.path.join(logs, "run_a"))

    window._thread = object()                       # pretend a load is in flight
    window._open(os.path.join(logs, "run_b"))
    assert window._data.run_name == "run_a"         # nothing has changed yet
    assert window._pending is not None              # ...and the click was not dropped

    window._thread = None
    window._open_pending()
    _settle(app, window)
    assert os.path.realpath(os.path.dirname(window._data.csv_path)) == \
        os.path.realpath(os.path.join(logs, "run_b"))
