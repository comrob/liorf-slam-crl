"""The viewer window: run list, canvas, slider, coverage strip."""

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
from ..settings import save_config
from .coverage import CoverageStrip
from .params_panel import ParamsPanel
from .sources import find_run_dirs
from .worker import LoadWorker


class MainWindow(QMainWindow):
    def __init__(self, base_dir="~/.ros/lili_logs", initial_input=""):
        super().__init__()
        self.setWindowTitle("replay_scale — anchor-frame viewer")
        self.resize(1180, 800)

        self._base_dir = base_dir
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
        open_button = QPushButton("Open run directory…")
        open_button.clicked.connect(self._browse)

        left = QVBoxLayout()
        left.addWidget(QLabel(f"Runs under {base_dir}"))
        left.addWidget(self._runs, 1)
        left.addWidget(open_button)
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

        # Divides the frame through by |comp|, so the LiDAR displacement is
        # read straight off the axes as a multiple of the complementary one --
        # which is the ratio the scale estimate is made of.
        self._normalize = QCheckBox("|comp| = 1")
        self._normalize.setToolTip(
            "Scale the axes so the complementary vector has unit length.\n"
            "The distance to the latest LiDAR position then reads directly\n"
            "as a multiple of the complementary displacement.")
        self._normalize.toggled.connect(lambda _c: self._redraw())

        controls = QHBoxLayout()
        controls.addWidget(QLabel("frame"))
        controls.addWidget(self._slider, 1)
        controls.addWidget(self._spin)
        controls.addWidget(self._observable_only)
        controls.addWidget(self._history_spin)
        controls.addWidget(self._history_step_spin)
        controls.addWidget(self._normalize)
        controls.addWidget(self._frame)
        controls.addWidget(self._zoom)

        right = QVBoxLayout()
        right.addWidget(self._canvas, 1)
        right.addWidget(self._coverage)
        right.addLayout(controls)
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
            item = QListWidgetItem(os.path.basename(path))
            item.setData(Qt.UserRole, path)
            item.setToolTip(path)
            self._runs.addItem(item)
        if self._runs.count() == 0:
            self._runs.addItem("(no runs with a replay CSV found)")

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
        self.statusBar().showMessage(f"Loading {input_path} …")

        self._thread = QThread(self)
        self._worker = LoadWorker(input_path, self._base_dir,
                                  settings=settings, params=params,
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

    def _on_loaded(self, data):
        self._data = data
        last = max(0, data.n_frames - 1)
        for widget in (self._slider, self._spin):
            widget.blockSignals(True)
            widget.setRange(0, last)
            widget.setValue(0)
            widget.blockSignals(False)
        self._coverage.set_views(data.views)
        self._set_controls_enabled(data.n_frames > 0)
        self._params_panel.set_busy(False)
        # Adopt the configuration that actually ran, so Revert returns here.
        self._params_panel.set_config(data.settings, data.params)
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

    def _save_config(self):
        """Write the edited settings out as a config the CLI can load."""
        edited = self._edited_config_or_warn()
        if edited is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save configuration", "replay_scale.yaml", "YAML (*.yaml *.yml)")
        if not path:
            return
        try:
            save_config(path, *edited)
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
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
        self._frame_status.setText(f"{view.status()}    [{self._filter_note}]")
