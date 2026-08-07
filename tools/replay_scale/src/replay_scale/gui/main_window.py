"""The viewer window: run list, then a tab per view of the loaded replay.

Three tabs today -- the anchor-frame scrubber, the whole-run trajectory plot and
the scale history -- fed by one load. The run list, the scrubber under the tabs
and the configuration dock sit outside them because they act on the session, not
on one view of it.

Nothing here writes: the replay runs with ``write=False`` and the only path to
disk is "Save trajectories…", which goes back through the pipeline so what lands
is what the CLI would have written.
"""

import os

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtCore import Qt, QThread
import numpy as np
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDockWidget,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..core.local_view import (
    DEFAULT_HISTORY,
    DEFAULT_HISTORY_STEP,
    EXTENT_LADDER,
    GATE_OBSERVABLE,
    NORMALIZED_EXTENT,
    build_local_frame_views,
    compute_axis_extent,
    snap_extent,
)
from ..plotting import draw_local_frame
from ..settings import DEFAULT_CONFIG_PATH, load_config, save_config
from .coverage import CoverageStrip
from .params_panel import ParamsPanel
from .scale_view import ScaleView
from .sources import find_run_dirs
from .trajectory_view import TrajectoryView
from .worker import LoadWorker, SaveWorker


class MainWindow(QMainWindow):
    def __init__(self, base_dir="~/.ros/lili_logs", initial_input="",
                 config_path=DEFAULT_CONFIG_PATH):
        super().__init__()
        self.setWindowTitle("replay_scale — replay viewer")
        self.resize(1180, 800)

        self._base_dir = base_dir
        # The file a load reads its configuration from until the panel is edited
        # or another file is opened. Not necessarily the bundled default.
        self._config_path = config_path
        self._data = None
        self._index = 0
        self._visible = np.zeros(0, dtype=int)
        self._filter_note = ""
        self._restore_index = None
        self._thread = None
        self._worker = None

        self._runs = QListWidget()
        self._runs.itemDoubleClicked.connect(
            lambda item: self._start_load(item.data(Qt.UserRole)))
        self._runs.itemSelectionChanged.connect(self._update_load_button)

        # Selecting a run and loading it are separate: selection is cheap and
        # reversible, a load re-runs the replay. Double-click still does both.
        self._load_button = QPushButton("Load")
        self._load_button.setToolTip("Replay the selected run.")
        self._load_button.clicked.connect(self._load_selected)
        self._load_button.setEnabled(False)
        open_button = QPushButton("Open other…")
        open_button.setToolTip("Load a run directory from anywhere on disk.")
        open_button.clicked.connect(self._browse)
        run_buttons = QHBoxLayout()
        run_buttons.addWidget(self._load_button, 1)
        run_buttons.addWidget(open_button)

        # The viewer replays with write=False, so nothing exists on disk until
        # this is pressed. It re-runs through the same writing path the CLI
        # uses, so what lands is what the CLI would have written.
        self._save_button = QPushButton("Save trajectories…")
        self._save_button.setToolTip(
            "Write this replay's trajectories and traces to a directory\n"
            "you choose, in the tool's usual layout.")
        self._save_button.clicked.connect(self._save_trajectories)
        self._save_button.setEnabled(False)

        self._active_label = QLabel("No run loaded")
        self._active_label.setWordWrap(True)
        self._active_label.setStyleSheet("QLabel { color: palette(mid); }")

        left = QVBoxLayout()
        left.addWidget(QLabel(f"Runs under {base_dir}"))
        left.addWidget(self._runs, 1)
        left.addLayout(run_buttons)
        left.addWidget(self._active_label)
        left.addWidget(self._save_button)
        left_panel = QWidget()
        left_panel.setLayout(left)
        left_panel.setMaximumWidth(320)

        self._figure = Figure(figsize=(7, 7))
        self._ax = self._figure.add_subplot(111)
        self._canvas = FigureCanvasQTAgg(self._figure)

        self._coverage = CoverageStrip()
        self._coverage.seeked.connect(self._set_index)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.valueChanged.connect(self._set_index)
        self._spin = QSpinBox()
        self._spin.valueChanged.connect(self._set_index)
        self._frame_status = QLabel("")

        # Fixed by default: the same axis limits on every frame, so scrubbing
        # never rescales and vector lengths are comparable across frames.
        # "auto" re-snaps per frame, which reads better on the near-stationary
        # stretches but makes frames incomparable.
        self._zoom = QComboBox()
        self._zoom.addItem("Zoom: fixed (whole run)", "run")
        self._zoom.addItem("Zoom: auto (per frame)", None)
        for step in EXTENT_LADDER:
            self._zoom.addItem(f"Zoom: fixed ±{step:g} m", step)
        self._zoom.currentIndexChanged.connect(lambda _i: self._redraw())

        # Complementary-forward by default: it removes the run's heading changes
        # and its out-and-back reversal, so the degenerate direction can be
        # compared across frames instead of spinning with the robot.
        self._frame = QComboBox()
        self._frame.addItem("Frame: robot (comp forward)", "comp")
        self._frame.addItem("Frame: map", "map")
        self._frame.addItem("Frame: robot (anchor)", "anchor")
        self._frame.currentIndexChanged.connect(lambda _i: self._rebuild_views())

        # On by default: a non-observable frame failed the estimator's speed
        # gate, so its vectors are noise-dominated and its direction means
        # nothing. Scrubbing past them is what makes the view look erratic.
        self._observable_only = QCheckBox("Observable only")
        self._observable_only.setChecked(True)
        self._observable_only.setToolTip(
            "Restrict the slider — and what the history overlay looks back at —\n"
            "to frames whose scale sample passed the observability gate\n"
            "(complementaryOdom.scaleMinNonDegenerateSpeed).")
        # Also restricts what the history overlay may look back at, which
        # needs the views rebuilt, not just the slider re-snapped.
        self._observable_only.toggled.connect(lambda _c: self._rebuild_views())

        # Each overlaid line belongs to an earlier frame's anchor, so they are
        # re-referenced onto this frame's origin before being drawn.
        self._history_spin = QSpinBox()
        self._history_spin.setRange(0, 100)
        self._history_spin.setValue(DEFAULT_HISTORY)
        self._history_spin.setPrefix("history: ")
        self._history_spin.setToolTip(
            "Overlay this many earlier degenerate lines behind the current one.")
        self._history_spin.valueChanged.connect(lambda _v: self._rebuild_views())

        # Consecutive frames barely differ, so a stride makes the overlay span a
        # useful interval rather than redrawing almost the same line N times.
        self._history_step_spin = QSpinBox()
        self._history_step_spin.setRange(1, 100)
        self._history_step_spin.setValue(DEFAULT_HISTORY_STEP)
        self._history_step_spin.setPrefix("step: ")
        self._history_step_spin.setToolTip(
            "Take every Nth earlier degenerate frame for the overlay.")
        self._history_step_spin.valueChanged.connect(lambda _v: self._rebuild_views())

        # On by default: dividing the frame through by |comp| makes the LiDAR
        # displacement read straight off the axes as a multiple of the
        # complementary one -- which is the ratio the scale estimate is made of.
        self._normalize = QCheckBox("|comp| = 1")
        self._normalize.setChecked(True)
        self._normalize.setToolTip(
            "Scale the axes so the complementary vector has unit length.\n"
            "The distance to the latest LiDAR position then reads directly\n"
            "as a multiple of the complementary displacement.")
        self._normalize.toggled.connect(lambda _c: self._redraw())

        # Only the anchor view's own drawing options live inside its tab; the
        # scrubber below is shared, so what it selects means the same thing on
        # every tab.
        view_options = QHBoxLayout()
        view_options.addWidget(self._history_spin)
        view_options.addWidget(self._history_step_spin)
        view_options.addWidget(self._normalize)
        view_options.addWidget(self._frame)
        view_options.addWidget(self._zoom)
        view_options.addStretch(1)

        frame_tab = QVBoxLayout()
        frame_tab.addWidget(self._canvas, 1)
        frame_tab.addLayout(view_options)
        frame_panel = QWidget()
        frame_panel.setLayout(frame_tab)

        # Same replay, whole-run views: where the scrubbed frame sits, and what
        # the estimator made of it.
        self._trajectory = TrajectoryView()
        self._scale = ScaleView()

        self._tabs = QTabWidget()
        self._tabs.addTab(frame_panel, "Anchor frame")
        self._tabs.addTab(self._trajectory, "Trajectory")
        self._tabs.addTab(self._scale, "Scale")

        # One scrubber under the tabs rather than one per tab: the frame index
        # is a property of the session, not of the view looking at it, so
        # switching tabs keeps you on the frame you were reading about.
        scrubber = QHBoxLayout()
        scrubber.addWidget(QLabel("frame"))
        scrubber.addWidget(self._slider, 1)
        scrubber.addWidget(self._spin)
        scrubber.addWidget(self._observable_only)

        right = QVBoxLayout()
        right.addWidget(self._tabs, 1)
        right.addWidget(self._coverage)
        right.addLayout(scrubber)
        right.addWidget(self._frame_status)
        right_panel = QWidget()
        right_panel.setLayout(right)

        layout = QHBoxLayout()
        layout.addWidget(left_panel)
        layout.addWidget(right_panel, 1)
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        # Editing settings and re-running is the same path a CLI invocation
        # takes; the panel only ever produces objects load_config could.
        self._params_panel = ParamsPanel()
        self._params_panel.applied.connect(self._apply_config)
        self._params_panel.save_requested.connect(self._save_config)
        self._params_panel.load_requested.connect(self._load_config_file)
        self._params_panel.set_source(config_path)
        dock = QDockWidget("Configuration", self)
        dock.setWidget(self._params_panel)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

        self._set_controls_enabled(False)
        self._populate_runs()
        self.statusBar().showMessage("Select a run to load.")
        if initial_input or self._runs.count():
            self._start_load(initial_input or self._runs.item(0).data(Qt.UserRole))

    # -- loading ------------------------------------------------------------

    def _populate_runs(self):
        self._runs.clear()
        for path in find_run_dirs(self._base_dir):
            self._add_run_item(path)
        if self._runs.count() == 0:
            item = QListWidgetItem("(no runs with a replay CSV found)")
            item.setFlags(Qt.NoItemFlags)
            self._runs.addItem(item)

    def _add_run_item(self, path):
        item = QListWidgetItem(os.path.basename(os.path.normpath(path)))
        item.setData(Qt.UserRole, path)
        item.setToolTip(path)
        self._runs.addItem(item)
        return item

    def _item_for(self, path):
        """The list row for a run path, added if it is not listed yet.

        A run opened from elsewhere on disk is not under the base directory, so
        it has no row -- but it is still the active run and has to be able to
        show as one.
        """
        target = os.path.normpath(os.path.expanduser(path))
        for row in range(self._runs.count()):
            item = self._runs.item(row)
            listed = item.data(Qt.UserRole)
            if listed and os.path.normpath(os.path.expanduser(listed)) == target:
                return item
        return self._add_run_item(path)

    def _mark_active(self, path):
        """Show which run the viewer is currently showing.

        Selection alone would not survive clicking around the list, and the run
        on screen is the one every other panel describes -- so it is marked in
        the row itself: bold, marked, and scrolled to.
        """
        active = self._item_for(path)
        for row in range(self._runs.count()):
            item = self._runs.item(row)
            font = item.font()
            font.setBold(item is active)
            item.setFont(font)
            name = os.path.basename(os.path.normpath(item.data(Qt.UserRole) or ""))
            if name:
                item.setText(f"▶  {name}" if item is active else f"    {name}")
        self._runs.setCurrentItem(active)
        self._runs.scrollToItem(active)
        self._active_label.setText(f"Showing: {os.path.dirname(self._data.csv_path)}"
                                   if self._data else "No run loaded")

    def _update_load_button(self):
        item = self._runs.currentItem()
        self._load_button.setEnabled(
            self._thread is None and item is not None and bool(item.data(Qt.UserRole)))

    def _load_selected(self):
        item = self._runs.currentItem()
        if item is not None and item.data(Qt.UserRole):
            self._start_load(item.data(Qt.UserRole))

    def _browse(self):
        path = QFileDialog.getExistingDirectory(
            self, "Open run directory", os.path.expanduser(self._base_dir))
        if path:
            self._start_load(path)

    def _start_load(self, input_path, *, settings=None, params=None):
        if self._thread is not None:
            return
        self._set_controls_enabled(False)
        self._params_panel.set_busy(True)
        self._load_button.setEnabled(False)
        self._save_button.setEnabled(False)
        self.statusBar().showMessage(f"Loading {input_path} …")

        self._thread = QThread(self)
        self._worker = LoadWorker(input_path, self._base_dir,
                                  settings=settings, params=params,
                                  config_path=self._config_path,
                                  frame=self._frame.currentData(),
                                  history=self._history_spin.value(),
                                  history_step=self._history_step_spin.value(),
                                  observable_only=self._observable_only.isChecked())
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.loaded.connect(self._on_loaded)
        self._worker.failed.connect(self._on_failed)
        self._worker.loaded.connect(self._thread.quit)
        self._worker.failed.connect(self._thread.quit)
        self._thread.finished.connect(self._on_thread_finished)
        self._thread.start()

    def _on_thread_finished(self):
        self._thread.deleteLater()
        self._thread = None
        self._worker = None
        self._update_load_button()

    def _on_loaded(self, data):
        self._data = data
        last = max(0, data.n_frames - 1)
        for widget in (self._slider, self._spin):
            widget.blockSignals(True)
            widget.setRange(0, last)
            widget.setValue(0)
            widget.blockSignals(False)
        self._coverage.set_views(data.views)
        self._trajectory.set_curves(
            data.curves, title=f"Replay trajectories — {data.run_name}")
        self._scale.set_geometries(data.geometries, params=data.params,
                                   title=f"Estimated scale — {data.run_name}")
        self._set_controls_enabled(data.n_frames > 0)
        self._params_panel.set_busy(False)
        # Adopt the configuration that actually ran, so Revert returns here.
        self._params_panel.set_config(data.settings, data.params)
        self._mark_active(os.path.dirname(data.csv_path))
        self._update_load_button()
        self._save_button.setEnabled(True)
        self.statusBar().showMessage(data.provenance)
        # Re-running with edited settings should not throw away where you were.
        target = 0 if self._restore_index is None else self._restore_index
        self._restore_index = None
        self._index = max(0, min(int(target), max(0, data.n_frames - 1)))
        self._refresh_visible()

    # -- which frames the slider may land on -------------------------------

    def _refresh_visible(self):
        """Recompute the reachable frames, then re-snap onto them."""
        if self._data is None:
            return
        views = self._data.views
        if self._observable_only.isChecked():
            visible = [i for i, v in enumerate(views) if v.window_state == GATE_OBSERVABLE]
            if not visible:
                # Nothing passed the gate; showing an empty scrubber would be
                # worse than showing everything with a note.
                visible = list(range(len(views)))
                self._filter_note = "no observable frames — showing all"
            else:
                self._filter_note = f"{len(visible)}/{len(views)} observable"
        else:
            visible = list(range(len(views)))
            self._filter_note = f"all {len(views)} frames"
        self._visible = np.array(visible, dtype=int)
        self._set_index(self._index)

    def _snap_to_visible(self, index):
        """Nearest reachable frame to ``index``."""
        if self._visible.size == 0:
            return index
        pos = int(np.searchsorted(self._visible, index))
        candidates = [p for p in (pos - 1, pos) if 0 <= p < self._visible.size]
        best = min(candidates, key=lambda p: abs(int(self._visible[p]) - index))
        return int(self._visible[best])

    def _on_failed(self, message):
        self.statusBar().showMessage("Load failed.")
        self._params_panel.set_busy(False)
        self._set_controls_enabled(self._data is not None)
        self._save_button.setEnabled(self._data is not None)
        self._update_load_button()
        QMessageBox.critical(self, "Load failed", message)

    # -- configuration ------------------------------------------------------

    def _edited_config_or_warn(self):
        try:
            return self._params_panel.edited_config()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid configuration", str(exc))
            return None

    def _apply_config(self):
        """Re-run the replay with the edited settings."""
        if self._data is None:
            return
        edited = self._edited_config_or_warn()
        if edited is None:
            return
        settings, params = edited
        self._restore_index = self._index
        self._start_load(self._data.csv_path, settings=settings, params=params)

    def _load_config_file(self):
        """Read a configuration file in and replay the current run with it.

        The alternative -- loading the values but waiting for Apply -- leaves
        the window showing one configuration and holding another. Reading a file
        is an explicit act, so it takes effect.
        """
        if self._thread is not None:
            return
        start = os.path.dirname(self._config_path) or os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self, "Load configuration", start, "YAML (*.yaml *.yml);;All files (*)")
        if not path:
            return

        try:
            settings, params = load_config(path)
        except Exception as exc:      # a bad file must not take the session down
            QMessageBox.critical(self, "Load failed", f"{type(exc).__name__}: {exc}")
            return

        self._config_path = path
        self._params_panel.set_config(settings, params, source=path)
        if self._data is None:
            # Nothing loaded yet: the file decides the next load, nothing to re-run.
            self.statusBar().showMessage(f"Configuration loaded from {path}")
            return
        self._restore_index = self._index
        self._start_load(self._data.csv_path, settings=settings, params=params)

    def _save_trajectories(self):
        """Write this replay's trajectories and traces to a chosen directory.

        The viewer replays with ``write=False``, so this is the only thing that
        puts a file on disk. It saves the *edited* configuration, not the one
        the current view was loaded with, so what is written matches what the
        panel shows -- and it goes through ``run_replay`` again to get there,
        which is what keeps the layout identical to the CLI's.
        """
        if self._data is None or self._thread is not None:
            return
        edited = self._edited_config_or_warn()
        if edited is None:
            return
        settings, params = edited

        directory = QFileDialog.getExistingDirectory(
            self, "Save trajectories to", os.path.dirname(self._data.csv_path))
        if not directory:
            return

        # Rooted at the chosen directory rather than under another "replay"
        # level: the user already picked where this should go.
        settings = settings.evolve(output_dir=directory, output_subdir="")
        self._save_button.setEnabled(False)
        self.statusBar().showMessage(f"Writing to {directory} …")

        self._thread = QThread(self)
        self._worker = SaveWorker(self._data.csv_path, settings, params)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.saved.connect(self._on_saved)
        self._worker.failed.connect(self._on_save_failed)
        self._worker.saved.connect(self._thread.quit)
        self._worker.failed.connect(self._thread.quit)
        self._thread.finished.connect(self._on_thread_finished)
        self._thread.start()

    def _on_saved(self, paths):
        self._save_button.setEnabled(True)
        self._update_load_button()
        self.statusBar().showMessage(
            f"Wrote {len(paths)} file(s) to {os.path.dirname(os.path.dirname(paths[0]))}"
            if paths else "Nothing to write.")

    def _on_save_failed(self, message):
        self._save_button.setEnabled(True)
        self._update_load_button()
        self.statusBar().showMessage("Save failed.")
        QMessageBox.critical(self, "Save failed", message)

    def _save_config(self):
        """Write the edited settings out as a config the CLI can load."""
        edited = self._edited_config_or_warn()
        if edited is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save configuration", self._config_path, "YAML (*.yaml *.yml)")
        if not path:
            return
        try:
            save_config(path, *edited)
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        # What was just written is what the panel holds, so it becomes the file
        # this session is working from.
        self._config_path = path
        self._params_panel.set_source(path)
        self.statusBar().showMessage(
            f"Wrote {path} — replay-scale-trajectory <run> --ros-params-yaml {path}")

    # -- scrubbing ----------------------------------------------------------

    def _set_controls_enabled(self, enabled):
        self._slider.setEnabled(enabled)
        self._spin.setEnabled(enabled)

    def _set_index(self, index):
        if self._data is None or not self._data.views:
            return
        index = max(0, min(int(index), self._data.n_frames - 1))
        index = self._snap_to_visible(index)
        for widget in (self._slider, self._spin):
            if widget.value() != index:
                widget.blockSignals(True)
                widget.setValue(index)
                widget.blockSignals(False)
        self._index = index
        self._redraw()

    def _rebuild_views(self):
        """Re-project the loaded geometry into the selected frame and history.

        Cheap enough to do on a toggle -- the replay itself is not repeated.
        Changing a *config* value does need a re-run; that goes via _apply_config.
        """
        if self._data is None:
            return
        frame = self._frame.currentData()
        self._data.views = build_local_frame_views(
            self._data.geometries, frame=frame, history=self._history_spin.value(),
            history_step=self._history_step_spin.value(),
            observable_only=self._observable_only.isChecked())
        self._data.axis_extent = snap_extent(
            compute_axis_extent(self._data.geometries, frame=frame), margin=1.0)
        self._coverage.set_views(self._data.views)
        self._refresh_visible()

    def _redraw(self):
        if self._data is None or not self._data.views:
            return
        view = self._data.views[self._index]
        normalize = self._normalize.isChecked()
        choice = self._zoom.currentData()
        if choice == "run":
            # The whole-run extent is in metres; normalized axes need their own.
            extent = NORMALIZED_EXTENT if normalize else self._data.axis_extent
        else:
            extent = choice          # None means auto-snap per frame
        draw_local_frame(self._ax, view, extent=extent, n_frames=self._data.n_frames,
                         normalize=normalize)
        self._canvas.draw_idle()
        self._coverage.set_index(self._index)
        # The other tabs mark the same frame: its replayed position, and where
        # its sample sits in the run. The anchor view's own origin is the
        # anchor, not the latest pose, hence latest_p here.
        self._trajectory.set_position(self._data.geometries[self._index].latest_p)
        self._scale.set_index(self._index)
        self._frame_status.setText(f"{view.status()}    [{self._filter_note}]")
