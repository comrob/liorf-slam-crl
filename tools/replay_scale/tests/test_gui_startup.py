"""The viewer coming up when there is nothing to open.

Qt-level, and the only test here that is: the failure it pins is one of
construction order, and there is no seam below the window to see it through.
Skipped where PySide6 is not installed, like the rest of the GUI.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# A plain try/except rather than importorskip: raising Skipped while the module
# is being imported aborts collection for the whole session under the ROS
# launch_testing plugin, which imports every test file looking for an entry
# point. A marker skips just this file.
try:
    from PySide6.QtWidgets import QApplication

    from replay_scale.gui.main_window import MainWindow
except ImportError:                             # pragma: no cover
    QApplication = MainWindow = None

pytestmark = pytest.mark.skipif(QApplication is None, reason="PySide6 not installed")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def _isolated_state(tmp_path, monkeypatch):
    """Never touch the developer's own remembered session."""
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))


def test_a_base_directory_with_no_runs_opens_an_empty_viewer(app, tmp_path):
    """Rather than a "no run directories found" dialog with nothing behind it.

    The run list always holds a row -- "(none found)" when it is empty -- so a
    window that asks it "is there anything to open" gets yes, tries to load a
    row carrying no path, and falls all the way through to resolving the newest
    run under a directory that has none.
    """
    empty = tmp_path / "logs"
    empty.mkdir()

    window = MainWindow(base_dir=str(empty), initial_input="")

    assert window._data is None
    assert window._thread is None              # nothing was ever started
    assert window._runs.count() == 1           # the placeholder, and only it
    assert window._runs.item(0).data(0x0100) is None    # Qt.UserRole


def test_a_base_directory_that_does_not_exist_is_the_same(app, tmp_path):
    window = MainWindow(base_dir=str(tmp_path / "nowhere"), initial_input="")
    assert window._data is None and window._thread is None
