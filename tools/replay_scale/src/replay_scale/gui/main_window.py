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
    QDoubleSpinBox,
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
    QSplitter,
    QStackedWidget,
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
from ..settings import (
    DEFAULT_CONFIG_PATH,
    load_config,
    run_config_path,
    same_apart_from_run_scoped,
    save_config,
    save_run_config,
)
from .coverage import CoverageStrip
from .params_panel import ParamsPanel
from .scale_view import ScaleView
from .run_state import (
    config_for_run,
    forget_run,
    remembered_runs,
    run_key,
    save_run_state,
)
from .session import clear_session, save_session, session_path
from .sources import list_runs, reload_reference, view_options
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
        # Every run opened this launch, by resolved path. Replaying one costs
        # seconds; coming back to it costs nothing, so nothing is evicted.
        self._cache = {}
        # Guards the two run lists against each other: selecting in one clears
        # the other, and neither may take that for a click of its own.
        self._syncing = False
        # Filled by _populate_runs, before any of it is read.
        self._entries = []
        self._by_run = {}
        self._replayed_keys = set()
        self._index = 0
        self._visible = np.zeros(0, dtype=int)
        self._filter_note = ""
        self._restore_index = None
        self._thread = None
        self._worker = None

        # Two lists, because the two questions are different: "the run I was
        # working on" and "the run I am looking for". The first is short and
        # keeps its settings; the second is everything the node ever wrote.
        # Two lists, and no run in both: "the ones I have replayed" and "the
        # ones I have not". A run that could be picked from either was the
        # thing that made picking confusing -- selecting it in one list left
        # the other one highlighting it too, and clicking that highlighted row
        # then did nothing, because it was already the current row.
        self._replayed = QListWidget()
        self._replayed.setToolTip(
            "Runs this viewer has replayed. Each keeps the configuration it was\n"
            "last given — its reference trajectory above all — across launches.\n"
            "One replayed already this session comes back instantly.")
        self._runs = QListWidget()
        self._runs.setToolTip(
            "Runs under the base directory that have not been replayed here.\n"
            "Symlinks — the ones someone stopped to name — are bold and first.\n"
            "Picking one previews it; nothing here ever starts a replay.")
        # Both signals, because they answer different questions: the selection
        # changed, or this row was clicked. A row that is already current emits
        # only the second, and a click on the run you are looking at should
        # still do the obvious thing.
        self._replayed.itemSelectionChanged.connect(
            lambda: self._pick(self._replayed, self._runs))
        self._replayed.itemClicked.connect(
            lambda item: self._pick(self._replayed, self._runs, item))
        self._runs.itemSelectionChanged.connect(
            lambda: self._pick(self._runs, self._replayed))
        self._runs.itemClicked.connect(
            lambda item: self._pick(self._runs, self._replayed, item))

        # Nothing in this panel replays: picking a run reads it, and the
        # estimator runs when you ask for it in the Configuration dock. One
        # button, one meaning -- and no way to lose ten seconds to a click.
        self._preview_button = QPushButton("Re-read")
        self._preview_button.setToolTip(
            "Read the selected run again: what it recorded, its odometry on its\n"
            "own, anything an earlier replay wrote beside it, and your reference\n"
            "trajectory against them. This is what picking a run already does;\n"
            "the button is for after something on disk has changed.\n"
            "To run the estimator, use Apply & replay in the Configuration dock.")
        self._preview_button.clicked.connect(lambda: self._reopen(self._selected))
        self._preview_button.setEnabled(False)
        open_button = QPushButton("Open other…")
        open_button.setToolTip("Open a run directory from anywhere on disk.")
        open_button.clicked.connect(self._browse)
        run_buttons = QHBoxLayout()
        run_buttons.addWidget(self._preview_button, 1)
        run_buttons.addWidget(open_button)
        #: The run the buttons act on: whichever list was last picked in.
        self._selected = ""
        #: A run asked for while another was loading; see _open.
        self._pending = None

        # The viewer replays with write=False, so nothing exists on disk until
        # this is pressed. It re-runs through the same writing path the CLI
        # uses, so what lands is what the CLI would have written.
        self._save_button = QPushButton("Save trajectories…")
        self._save_button.setToolTip(
            "Write this run's trajectories and traces to a directory you choose,\n"
            "in the tool's usual layout. Runs the estimator with the settings on\n"
            "screen to produce them, exactly as the CLI would.")
        self._save_button.clicked.connect(self._save_trajectories)
        self._save_button.setEnabled(False)

        self._active_label = QLabel("No run loaded")
        self._active_label.setWordWrap(True)
        self._active_label.setStyleSheet("QLabel { color: palette(mid); }")

        left = QVBoxLayout()
        left.addWidget(QLabel("Replayed — settings remembered"))
        left.addWidget(self._replayed, 1)
        left.addWidget(QLabel(f"Not replayed yet — under {base_dir}"))
        left.addWidget(self._runs, 2)
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
            "(complementaryOdom.scaleMinNonDegenerateSpeed).\n"
            "Checked, the overlay draws exactly the lines the estimator fitted\n"
            "through, so the meeting point is the one the estimate was read from.")
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

        # Off by default: it is a second population on a plot that already has
        # lines, an arrow and an ellipse on it. On, it answers "is the point the
        # correction reads jittering or drifting", which one frame cannot.
        self._meet_history = QCheckBox("meet trail")
        self._meet_history.setToolTip(
            "Scatter the meeting point the estimator fitted on each frame the\n"
            "history looks back at — the trail of what the correction has been\n"
            "reading, one point per frame rather than one per line.\n"
            "Each point is in units of its own frame's |comp|, so it is only\n"
            "drawn in the complementary-forward frame with |comp| = 1.")
        self._meet_history.toggled.connect(lambda _c: self._redraw())

        # Where the view stops at the bottom and on the left. The picture is
        # centred on the anchor, so half of it is often empty -- cropping is
        # what turns a scrubbing view into a figure. Both read "auto" at the far
        # end of their travel, which is the symmetric view the extent decides.
        self._min_x = self._crop_spin(
            "min x: ", "Lowest x drawn — the bottom edge of the view.\n"
                       "Slide fully left for auto: symmetric about the anchor.")
        self._max_y = self._crop_spin(
            "max y: ", "Highest y drawn — the left-hand edge, since +y is drawn\n"
                       "to the left. Slide fully left for auto.")

        # Only the anchor view's own drawing options live inside its tab; the
        # scrubber below is shared, so what it selects means the same thing on
        # every tab.
        view_options = QHBoxLayout()
        view_options.addWidget(self._history_spin)
        view_options.addWidget(self._history_step_spin)
        view_options.addWidget(self._normalize)
        view_options.addWidget(self._meet_history)
        view_options.addWidget(self._frame)
        view_options.addWidget(self._zoom)
        view_options.addWidget(self._min_x)
        view_options.addWidget(self._max_y)
        view_options.addStretch(1)

        frame_tab = QVBoxLayout()
        frame_tab.addWidget(self._canvas, 1)
        frame_tab.addLayout(view_options)
        frame_panel = QWidget()
        frame_panel.setLayout(frame_tab)

        # Same replay, whole-run views: where the scrubbed frame sits, and what
        # the estimator made of it.
        self._frame_panel = frame_panel
        self._trajectory = TrajectoryView()
        self._scale = ScaleView()
        self._panels = ((frame_panel, "Anchor frame"),
                        (self._trajectory, "Trajectory"),
                        (self._scale, "Scale"))

        self._tabs = QTabWidget()

        # The same three views, side by side instead of one at a time. Reading
        # the scale curve against the trajectory it produced is the whole
        # argument for it; on a wide screen there is no reason to alternate.
        # The two anchor/trajectory plots are square and share the top row; the
        # scale curve is wide and short, so it gets the bottom.
        self._tiles_top = QSplitter(Qt.Horizontal)
        self._tiles = QSplitter(Qt.Vertical)
        self._tiles.addWidget(self._tiles_top)
        self._tiles.setStretchFactor(0, 3)
        self._tiles.setStretchFactor(1, 2)

        self._views = QStackedWidget()
        self._views.addWidget(self._tabs)
        self._views.addWidget(self._tiles)

        self._show_all = QCheckBox("All views")
        self._show_all.setToolTip(
            "Show the anchor frame, the trajectory and the scale at once\n"
            "instead of one tab at a time. The scrubber is shared either way.")
        self._show_all.toggled.connect(self._set_view_layout)
        self._set_view_layout(False)

        # One scrubber under the tabs rather than one per tab: the frame index
        # is a property of the session, not of the view looking at it, so
        # switching tabs keeps you on the frame you were reading about.
        scrubber = QHBoxLayout()
        scrubber.addWidget(QLabel("frame"))
        scrubber.addWidget(self._slider, 1)
        scrubber.addWidget(self._spin)
        scrubber.addWidget(self._observable_only)
        scrubber.addWidget(self._show_all)

        right = QVBoxLayout()
        right.addWidget(self._views, 1)
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
        self._params_panel.save_run_requested.connect(self._save_run_config)
        self._params_panel.load_requested.connect(self._load_config_file)
        self._params_panel.reset_requested.connect(self._reset_config)
        # A restored session is a file nobody chose to open, so it says so.
        self._params_panel.set_source(
            config_path,
            note="remembered from last session" if config_path == session_path() else "")
        dock = QDockWidget("Configuration", self)
        dock.setWidget(self._params_panel)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

        self._set_controls_enabled(False)
        runs = self._populate_runs()
        self.statusBar().showMessage("Pick a run to read.")
        # Read, not replayed: coming up is instant, and the estimator runs when
        # you ask for it -- even on the run you left off on.
        if initial_input or runs:
            self._open(initial_input or runs[0])

    #: How far a crop may be pushed, either way. The view worth cropping is the
    #: normalized one, where everything interesting happens within a couple of
    #: |comp| of the anchor -- a box that can be dragged to 1000 is a box nobody
    #: can aim.
    CROP_LIMIT = 2.0

    #: Value at which a crop box means "no crop", shown as "auto": the low end
    #: of its travel, for both boxes, so one rule covers them. A sentinel rather
    #: than a checkbox each: the number and its off switch are one control.
    CROP_AUTO = -CROP_LIMIT

    def _crop_spin(self, prefix, tooltip):
        """One edge of the view, in whatever unit the axes are currently in."""
        spin = QDoubleSpinBox()
        spin.setRange(self.CROP_AUTO, self.CROP_LIMIT)
        spin.setValue(self.CROP_AUTO)
        spin.setSpecialValueText(f"{prefix}auto")
        spin.setPrefix(prefix)
        spin.setDecimals(2)
        spin.setSingleStep(0.25)
        spin.setToolTip(tooltip)
        spin.valueChanged.connect(lambda _v: self._redraw())
        return spin

    def _crop_value(self, spin):
        """What a crop box says, or None for "let the extent decide"."""
        value = spin.value()
        return None if value <= self.CROP_AUTO else value

    # -- how the views are arranged ----------------------------------------

    def _set_view_layout(self, all_at_once):
        """Move the three views between the tab widget and the tiled splitters.

        The same widget objects either way -- Qt reparents on insert -- so
        neither the figures nor the deferred-draw state is rebuilt, and the
        frame you were on survives the switch.
        """
        if all_at_once:
            while self._tabs.count():
                self._tabs.removeTab(0)
            self._tiles_top.addWidget(self._frame_panel)
            self._tiles_top.addWidget(self._trajectory)
            self._tiles.addWidget(self._scale)
        else:
            for widget, name in self._panels:
                self._tabs.addTab(widget, name)
        self._views.setCurrentWidget(self._tiles if all_at_once else self._tabs)

    # -- loading ------------------------------------------------------------

    def _populate_runs(self):
        """Fill both lists; returns the runs offered, in the order they appear.

        The two are disjoint by construction: a run that has been replayed here
        belongs in the first list and is taken out of the second, so every row
        in the window names a run exactly once and picking one is unambiguous.

        Returns the paths rather than the row count, because an empty list still
        gets a row -- the "(none found)" placeholder -- and a caller asking "is
        there anything to open" must not be told yes by it.
        """
        self._entries = list_runs(self._base_dir)
        # By run rather than by path: the replayed list holds resolved
        # directories, and the name to show one under is the name the base
        # directory offers it under -- the one you gave it.
        self._by_run = {run_key(entry.path): entry for entry in self._entries}
        remembered = remembered_runs()
        self._replayed_keys = {run_key(path) for path, _when in remembered}

        self._syncing = True
        try:
            self._replayed.clear()
            for path, _when in remembered:
                entry = self._by_run.get(run_key(path))
                self._add_run_item(
                    self._replayed, path,
                    bold=entry is not None and entry.is_link,
                    name=None if entry is None else entry.name,
                    tip=f"{entry.describe() if entry else path}\n"
                        f"replayed here before; its settings are remembered")
            if not remembered:
                self._add_placeholder(self._replayed, "(nothing replayed here yet)")

            self._runs.clear()
            offered = [entry for entry in self._entries
                       if run_key(entry.path) not in self._replayed_keys]
            for entry in offered:
                # Bold for the links: the runs someone stopped to name, listed
                # first by list_runs, are the ones worth finding again.
                self._add_run_item(self._runs, entry.path, bold=entry.is_link,
                                   tip=entry.describe())
            if not offered:
                self._add_placeholder(
                    self._runs, "(nothing here that has not been replayed)"
                    if self._entries else "(no runs with a replay CSV found)")
        finally:
            self._syncing = False
        return [path for path, _when in remembered] + [entry.path for entry in offered]

    def _refresh_lists_if_needed(self, run_dir, replayed):
        """Rebuild the lists only when a run has actually moved between them.

        Rebuilding drops and recreates every row, which takes the selection
        with it. Doing that on every read -- and a read happens every time a
        run is picked -- was what made clicking around the lists feel like
        fighting them.
        """
        if not replayed or run_key(run_dir) in self._replayed_keys:
            return
        self._populate_runs()

    def _add_placeholder(self, widget, text):
        item = QListWidgetItem(text)
        item.setFlags(Qt.NoItemFlags)
        widget.addItem(item)
        return item

    def _add_run_item(self, widget, path, *, bold=False, name=None, tip=""):
        item = QListWidgetItem(f"    {name or os.path.basename(os.path.normpath(path))}")
        item.setData(Qt.UserRole, path)
        item.setToolTip(tip or path)
        if bold:
            font = item.font()
            font.setBold(True)
            item.setFont(font)
        widget.addItem(item)
        return item

    def _rows(self):
        """Every row in both lists that names a run."""
        for widget in (self._replayed, self._runs):
            for row in range(widget.count()):
                item = widget.item(row)
                if item.data(Qt.UserRole):
                    yield widget, item

    def _item_for(self, path):
        """The row for a run, added to the base list if it is not listed yet.

        A run opened from elsewhere on disk is not under the base directory, so
        it has no row -- but it is still the active run and has to be able to
        show as one.
        """
        target = run_key(path)
        for _widget, item in self._rows():
            if run_key(item.data(Qt.UserRole)) == target:
                return item
        return self._add_run_item(self._runs, path)

    def _mark_active(self, path):
        """Show which run the viewer is currently showing.

        Selection alone would not survive clicking around the lists, and the run
        on screen is the one every other panel describes -- so it is marked in
        the row itself. With an arrow rather than a bold face: bold says
        "symlink" here, and one weight cannot mean two things.
        """
        active = self._item_for(path)
        self._syncing = True
        try:
            for widget, item in self._rows():
                name = item.text().lstrip("▶ ").strip()
                item.setText(f"▶  {name}" if item is active else f"    {name}")
                if item is active:
                    widget.setCurrentItem(item)
                    widget.scrollToItem(item)
                elif widget.currentItem() is item:
                    widget.setCurrentItem(None)
        finally:
            self._syncing = False
        self._selected = path
        cached = self._cache.get(run_key(path))
        self._active_label.setText(
            (f"Showing: {os.path.dirname(self._data.csv_path)}"
             + ("  (preview)" if cached is not None and cached.preview else ""))
            if self._data else "No run loaded")

    def _pick(self, source, other, item=None):
        """A run was picked in one of the lists: clear the other, and read it.

        Picking a run *reads* it -- see sources.preview_run_dir -- and nothing
        in this panel does more than that. The estimator runs when you ask for
        it in the Configuration dock, and only then, so no click here can cost
        a pass over the whole recording.

        A run already opened this session comes straight back from memory,
        replayed or previewed, whichever it was.
        """
        if self._syncing:
            return
        item = item if item is not None else source.currentItem()
        path = None if item is None else item.data(Qt.UserRole)
        if not path:
            return
        self._syncing = True
        try:
            other.setCurrentItem(None)
        finally:
            self._syncing = False
        self._selected = path
        self._update_load_button()
        if self._showing(path):
            return              # already on screen; a second click means nothing
        self._open(path)

    def _showing(self, path):
        """Whether this run is what the window currently shows."""
        return (self._data is not None and self._thread is None
                and run_key(os.path.dirname(self._data.csv_path)) == run_key(path))

    def _open(self, path):
        """Show a run: from the cache if it is there, otherwise read it.

        The cache is why coming back to a run is free -- and why a run replayed
        earlier this session comes back replayed, rather than as the preview a
        fresh read would give.
        """
        if not path:
            return
        if self._thread is not None:
            # Clicking down a list faster than the reads finish is the normal
            # way to use it. The last thing asked for is the thing wanted, so
            # it is queued rather than dropped -- dropping it would leave the
            # list pointing at one run and the window showing another.
            self._pending = path
            return
        cached = self._cache.get(run_key(path))
        if cached is not None:
            self._show(cached)
            return
        self._start_load(path, preview=True)

    def _reopen(self, path):
        """Read a run again from disk, past the cache.

        For after something under the run changed -- an earlier replay's
        trajectories written beside it, say. It re-reads; it does not replay.
        """
        if not path or self._thread is not None:
            return
        self._cache.pop(run_key(path), None)
        self._start_load(path, preview=True)

    def _browse(self):
        path = QFileDialog.getExistingDirectory(
            self, "Open run directory", os.path.expanduser(self._base_dir))
        if path:
            self._selected = path
            self._open(path)

    def _update_load_button(self):
        self._preview_button.setEnabled(self._thread is None and bool(self._selected))

    def _start_load(self, input_path, *, settings=None, params=None, preview=False):
        """Read a run on the worker thread; ``preview`` skips the estimator."""
        if self._thread is not None:
            return
        self._set_controls_enabled(False)
        self._params_panel.set_busy(True)
        self._preview_button.setEnabled(False)
        self._save_button.setEnabled(False)
        self.statusBar().showMessage(
            f"{'Reading' if preview else 'Replaying'} {input_path} …")

        self._thread = QThread(self)
        self._worker = LoadWorker(input_path, self._base_dir,
                                  settings=settings, params=params,
                                  config_path=self._config_path, preview=preview,
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
        self._open_pending()

    def _open_pending(self):
        """Open whatever was asked for while the loader was busy; see _open."""
        if self._pending is None:
            return
        path, self._pending = self._pending, None
        self._open(path)

    def _on_loaded(self, data):
        """A load finished: keep it, remember it, show it."""
        self._params_panel.set_busy(False)
        self._cache[run_key(os.path.dirname(data.csv_path))] = data
        # Remembered either way, so the settings a run was given survive a
        # switch; only a replay is *listed* as one, which is what that list
        # means. See run_state.REPLAYED_KEY.
        self._remember_run(data)
        self._show(data)
        # Remember what worked, so the next launch opens here. Done on a
        # successful load rather than on Apply: a configuration that failed to
        # run is not one to come back to.
        self._remember_session()

    def _show(self, data):
        """Put a loaded run on screen -- freshly loaded, or back from the cache.

        Everything below is redrawing, not computing: the replay (or the read)
        already happened. The one thing that can still be stale is the
        projection into the anchor frame, which was built for whatever the view
        controls said at the time.
        """
        self._data = data
        if data.geometries and data.view_options != self._view_options():
            self._reproject(data)
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
        # Adopt the configuration that actually ran, so Revert returns here.
        # The resolved mounts go in first: set_config decides what the mount
        # boxes show, and with no override configured that is the run's own.
        self._params_panel.set_mounts(data.active_odometry, data.run_mount)
        self._params_panel.set_config(data.settings, data.params)
        self._show_config_source(data)
        self._mark_active(os.path.dirname(data.csv_path))
        self._update_load_button()
        self._save_button.setEnabled(True)
        self.statusBar().showMessage(data.provenance)
        # Re-running with edited settings should not throw away where you were.
        target = 0 if self._restore_index is None else self._restore_index
        self._restore_index = None
        self._index = max(0, min(int(target), max(0, data.n_frames - 1)))
        self._refresh_visible()
        if not data.views:
            self._draw_no_frames()

    def _show_config_source(self, data):
        """Name the file the configuration on screen came from, and the layering."""
        sources = data.config_sources
        if sources is None:
            return
        if sources.state_path:
            note = "what you last applied to this run"
            if sources.run_path:
                note += f", over {os.path.basename(sources.run_path)}"
            self._params_panel.set_source(sources.state_path, note=note)
        elif sources.run_path:
            self._params_panel.set_source(
                sources.run_path,
                note=f"the run's own, merged over {os.path.basename(sources.base_path)}")
        else:
            self._params_panel.set_source(sources.base_path)

    def _view_options(self):
        """What the per-frame views would be built for right now."""
        return view_options(self._frame.currentData(), self._history_spin.value(),
                            self._history_step_spin.value(),
                            self._observable_only.isChecked())

    def _reproject(self, data):
        """Rebuild one run's views for the current controls; no replay.

        A run comes back from the cache projected the way it was left. Changing
        the frame or the history while looking at another run must not make it
        come back wrong, and re-projecting costs a fraction of a replay.
        """
        options = self._view_options()
        data.views = build_local_frame_views(
            data.geometries, frame=options[0], history=options[1],
            history_step=options[2], observable_only=options[3])
        data.axis_extent = snap_extent(
            compute_axis_extent(data.geometries, frame=options[0]), margin=1.0)
        data.view_options = options

    def _draw_no_frames(self):
        """The anchor view for a run with no per-frame geometry: say why.

        A preview has none -- it never ran the estimator -- and leaving the
        previous run's vectors on screen would be a lie about the run named
        above them.
        """
        self._ax.clear()
        self._ax.set_axis_off()
        self._ax.text(0.5, 0.5,
                      "preview — \"Apply & replay\" to see the per-frame vectors"
                      if self._data is not None and self._data.preview
                      else "nothing to draw",
                      ha="center", va="center", fontsize=11, color="#666666",
                      transform=self._ax.transAxes)
        self._canvas.draw_idle()
        self._frame_status.setText("")

    def _remember_run(self, data):
        """Keep this configuration with this run, and list the run as replayed.

        Written as a delta against what the config files say, so a run whose
        settings were never touched keeps following the base configuration --
        and one whose reference you set keeps that reference, and only it.
        """
        run_dir = os.path.dirname(data.csv_path)
        sources = data.config_sources
        save_run_state(run_dir, data.settings, data.params,
                       inherited=None if sources is None else sources.inherited,
                       replayed=not data.preview)
        self._refresh_lists_if_needed(run_dir, replayed=not data.preview)
        self._mark_active(run_dir)

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
        """Replay the run with the edited settings. The only path to a replay.

        Nothing else in the window runs the estimator: picking a run reads it,
        and this is where you ask for the rest. So pressing it having changed
        nothing is not a no-op -- it is how a run you are previewing gets
        replayed.

        The one edit that does not need a replay is the reference trajectory.
        The replay never reads it -- it is drawn, not measured against -- so it
        is refitted and redrawn on the spot, which is what a replay would have
        drawn anyway.
        """
        if self._data is None:
            return
        edited = self._edited_config_or_warn()
        if edited is None:
            return
        settings, params = edited
        # The reference is the only run-scoped section, so "nothing else
        # changed" is exactly the case a redraw can serve -- and it has to have
        # actually changed, or this would answer "replay it" with a redraw.
        reference_only = (
            settings.reference_trajectory != self._data.settings.reference_trajectory
            and same_apart_from_run_scoped(
                (settings, params), (self._data.settings, self._data.params)))
        if reference_only and self._data.frames is not None:
            self._apply_reference(settings)
            return
        self._restore_index = self._index
        self._start_load(self._data.csv_path, settings=settings, params=params)

    def _apply_reference(self, settings):
        """Redraw the trajectory plot against another reference; no replay.

        The fitted alignments still turn it onto the replay on screen, so what
        lands is what a re-run would have drawn -- it just does not run one.
        A file that cannot be read leaves the plot as it was.
        """
        try:
            curves, status = reload_reference(self._data, settings)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Reference trajectory",
                                f"{type(exc).__name__}: {exc}")
            return
        self._data.curves = curves
        # Adopted like a load's, so Revert comes back here and "Save to run"
        # writes what is on screen. Nothing else in the session moved: the
        # views, the geometries and what is remembered are all unchanged --
        # the session never carries a reference anyway (see gui.session).
        self._data.settings = settings
        # Kept with the run like any other applied configuration -- this is the
        # edit that most needs to be, since a reference belongs to one run.
        self._remember_run(self._data)
        self._params_panel.set_config(settings, self._data.params)
        self._trajectory.set_curves(
            curves, title=f"Replay trajectories — {self._data.run_name}")
        if self._data.geometries:
            self._trajectory.set_position(self._data.geometries[self._index].latest_p)
        self.statusBar().showMessage(
            " · ".join(line.strip() for line in status.splitlines()) if status
            else "No reference trajectory — the replayed curves are as they were.")

    def _remember_session(self):
        """Write the configuration that just ran, and the run it ran on.

        Best-effort by design -- a home directory that cannot be written to
        costs the convenience and nothing else, so nothing is reported.
        """
        if self._data is None:
            return
        save_session(self._data.settings, self._data.params,
                     input_path=os.path.dirname(self._data.csv_path),
                     base_dir=self._base_dir)

    def _reset_config(self):
        """Forget the remembered session and go back to the bundled defaults.

        Revert undoes edits back to what is loaded; this undoes the loading
        too, including what an earlier session left behind -- so an edit that
        made the viewer unusable is one button away from gone, this launch and
        the next.
        """
        if self._thread is not None:
            return
        run_dir = "" if self._data is None else os.path.dirname(self._data.csv_path)
        confirm = QMessageBox.question(
            self, "Reset to defaults",
            f"Forget the remembered session ({session_path()})"
            + (f" and what was applied to {os.path.basename(run_dir)}" if run_dir else "")
            + ", and go back to the bundled configuration?")
        if confirm != QMessageBox.Yes:
            return

        forgotten = clear_session()
        # The run's own remembered layer too: it is the other thing that would
        # otherwise put the corner you edited yourself into straight back.
        if run_dir:
            forget_run(run_dir)
            self._populate_runs()
        try:
            settings, params = load_config(DEFAULT_CONFIG_PATH)
        except Exception as exc:      # the bundled file, so this is a bug, not a typo
            QMessageBox.critical(self, "Reset failed", f"{type(exc).__name__}: {exc}")
            return

        self._config_path = DEFAULT_CONFIG_PATH
        self._params_panel.set_config(settings, params, source=DEFAULT_CONFIG_PATH)
        note = "Forgot the remembered session. " if forgotten else "Nothing was remembered. "
        if self._data is None:
            self.statusBar().showMessage(note + f"Back to {DEFAULT_CONFIG_PATH}")
            return
        # The re-run remembers again on success, with the defaults now in it:
        # the settings are reset, the run you are looking at stays yours.
        self.statusBar().showMessage(
            note + f"Re-running with {DEFAULT_CONFIG_PATH} (this run stays remembered) …")
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

    def _save_run_config(self):
        """Write the panel's settings into the loaded run's own config file.

        Only what differs from the base configuration, so the run keeps what is
        specific to it and inherits the rest -- a full copy would pin every
        method setting, and a later change to a shared default would never reach
        this run. Written where the next replay of it will find it, by the
        viewer and by the CLI alike.
        """
        if self._data is None:
            return
        edited = self._edited_config_or_warn()
        if edited is None:
            return

        sources = self._data.config_sources
        path = run_config_path(self._data.csv_path)
        if os.path.exists(path) and QMessageBox.question(
                self, "Overwrite the run's configuration?",
                f"{path}\n\nalready exists. Replace it with the settings on screen?"
        ) != QMessageBox.Yes:
            return
        try:
            save_run_config(self._data.csv_path, *edited,
                            base_mapping=None if sources is None else sources.base_mapping)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        # The run's own file now says it, so the viewer's remembered layer for
        # this run has nothing left to add -- and leaving a stale copy of it
        # there would quietly outrank the file just written. Re-resolved from
        # disk and written back, so the run stays listed as replayed.
        run_dir = os.path.dirname(self._data.csv_path)
        forget_run(run_dir)
        _, _, resolved = config_for_run(self._data.csv_path, self._config_path)
        self._data.config_sources = resolved
        self._remember_run(self._data)
        self._params_panel.set_source(path, note="the run's own")
        self.statusBar().showMessage(
            f"Wrote {path} — this run now replays with these settings by default")

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
        A run in the cache is re-projected when it next comes back on screen,
        not now: the options it was built for travel with it.
        """
        if self._data is None or not self._data.geometries:
            return
        self._reproject(self._data)
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
                         normalize=normalize,
                         line_fit_norm=self._data.params.scale_line_fit_norm,
                         meet_history=self._meet_history.isChecked(),
                         x_min=self._crop_value(self._min_x),
                         y_max=self._crop_value(self._max_y))
        self._canvas.draw_idle()
        self._coverage.set_index(self._index)
        # The other tabs mark the same frame: its replayed position, and where
        # its sample sits in the run. The anchor view's own origin is the
        # anchor, not the latest pose, hence latest_p here.
        self._trajectory.set_position(self._data.geometries[self._index].latest_p)
        self._scale.set_index(self._index)
        self._frame_status.setText(f"{view.status()}    [{self._filter_note}]")
