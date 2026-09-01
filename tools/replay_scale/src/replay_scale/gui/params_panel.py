"""Editor for the settings a replay runs with.

Edits ``ReplayToolSettings`` / ``ReplayParams`` objects in memory and hands them
back to ``run_replay`` -- the same objects the YAML loader produces, so the GUI
cannot express a configuration the CLI could not. "Load YAML" reads a file into
those objects and "Save YAML" writes them back out, which is how a session
starts from someone else's configuration and stays reproducible headlessly.

Grouped the way the config file is: what the complementary displacement is
corrected by, then the settings belonging to each method, then what every method
shares. Each group folds, and the ones the selected method does not read fold
themselves and grey out -- most of this form describes a method that is not
running. Folding never edits: a folded section still contributes its values.

``scale_mode`` is not editable: the viewer needs the estimator's per-frame
vectors, which only the estimated path produces.
"""

import math
import os

import numpy as np

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..core.lines import LINE_FIT_NORMS
from ..core.model import (
    COMPLEMENTARY_CORRECTIONS,
    CORRECTION_MODES,
    ESTIMATION_FRAMES,
    LINES_MEET_CORRECTIONS,
    REFERENCE_ALIGNMENTS,
    SMOOTHING_MODES,
)
from ..core.odom_source import DRIFT_AXES, MATCH_MODES
from ..core.se3 import matrix_to_rpy, rpy_to_matrix
from ..settings import validate_replay_params
from .collapsible import CollapsibleSection

#: Largest finite bound the spin boxes offer; an infinite scaleMax or
#: scaleLateralMax shows as 0 ("unbounded") instead, since a spin box cannot
#: hold infinity.
UNBOUNDED_SCALE = 1000.0


class _MountEditor:
    """The six boxes describing one ``T_x_to_lidar``, and what they stand for.

    There are two mounts in a configuration -- the recorded odometry's and an
    alternative stream's -- and they are different sensors, so they get one
    editor each rather than one editor whose meaning shifts. This holds the
    widgets and the bookkeeping they share; which mount it edits, and when it
    applies, is the panel's business.

    Metres and degrees, since that is how a mount is measured, written out as
    the node's ``extrinsicTrans`` / ``extrinsicRot``.
    """

    def __init__(self, panel, trans_tip, rpy_tip):
        self.trans = [panel._double(-10.0, 10.0, 0.01, 4) for _ in range(3)]
        self.rpy = [panel._double(-180.0, 180.0, 0.5, 4) for _ in range(3)]
        self.trans_row = panel._row(self.trans)
        self.rpy_row = panel._row(self.rpy)
        self.trans_row.setToolTip(trans_tip)
        self.rpy_row.setToolTip(rpy_tip)
        # The matrix the boxes were last filled from, and the values they held
        # then. Applying without touching them hands back that matrix itself
        # rather than one rebuilt from six rounded spin boxes, so a mount loaded
        # from a file survives a re-run unchanged.
        self._shown = (None, self.values())

    def rows(self, prefix=""):
        return [(f"{prefix}extrinsicTrans (x y z, m)", self.trans_row),
                (f"{prefix}extrinsicRot (roll pitch yaw, deg)", self.rpy_row)]

    def values(self):
        return tuple(w.value() for w in self.trans + self.rpy)

    def fill(self, matrix):
        """Show a 4x4, remembering that it is what the boxes stand for."""
        if matrix is None:
            self._shown = (None, self.values())
            return
        tx, ty, tz, roll, pitch, yaw = matrix_to_rpy(matrix)
        for widget, value in zip(self.trans, (tx, ty, tz)):
            widget.setValue(value)
        for widget, value in zip(self.rpy, (roll, pitch, yaw)):
            widget.setValue(math.degrees(value))
        self._shown = (matrix, self.values())

    def matrix(self):
        """The 4x4 the boxes currently describe."""
        shown_matrix, shown_values = self._shown
        if shown_matrix is not None and self.values() == shown_values:
            return shown_matrix
        tx, ty, tz, roll, pitch, yaw = self.values()
        return rpy_to_matrix(tx, ty, tz, math.radians(roll),
                             math.radians(pitch), math.radians(yaw))

    def set_enabled(self, form, on):
        for row in (self.trans_row, self.rpy_row):
            ParamsPanel._set_row_enabled(form, row, on)


class ParamsPanel(QWidget):
    """Form over the estimator parameters and the replay tool settings."""

    applied = Signal()          # user wants the replay re-run with these values
    save_requested = Signal()
    save_run_requested = Signal()   # ...into the loaded run's own config file
    load_requested = Signal()   # user wants a config file read in
    reset_requested = Signal()  # user wants the bundled defaults back

    def __init__(self, parent=None):
        super().__init__(parent)
        self._settings = None
        self._params = None
        # What the last replay actually resolved: the mount the run recorded,
        # and which odometry ended up being corrected with which mount. Shown on
        # screen rather than only in the log -- an unticked override box used to
        # leave whatever was last typed sitting there, which reads as if it
        # applied, and a mount silently becoming the identity comes back out as
        # a scale.
        self._run_mount = None
        self._active = None

        # Which file the values on screen came from. The panel edits objects,
        # not a document, so once anything is touched this is only provenance --
        # but "which config am I looking at" is the first question a second
        # config file raises.
        self._source_label = QLabel("")
        self._source_label.setWordWrap(True)
        self._source_label.setStyleSheet("QLabel { color: palette(mid); }")

        # Open: what a session changes most. Folded: the rest, which is either
        # method-specific (and folded/unfolded by the selector) or rarely touched.
        self._lines_section = CollapsibleSection(
            "lines_meet_x / lines_meet_xy", self._lines_box())
        self._sections = [
            CollapsibleSection("correction", self._correction_box()),
            CollapsibleSection("estimation frame", self._frame_box(), expanded=False),
            CollapsibleSection("recorded odometry", self._recorded_box(), expanded=False),
            self._lines_section,
            CollapsibleSection("smoothing and bounds", self._smoothing_box()),
            CollapsibleSection("complementaryOdom (general)", self._general_box(),
                               expanded=False),
            CollapsibleSection("replay_scale_tool", self._tool_box(), expanded=False),
            CollapsibleSection("simulated complementary drift", self._drift_box(),
                               expanded=False),
            CollapsibleSection("reference trajectory", self._reference_box(),
                               expanded=False),
        ]

        form = QVBoxLayout()
        form.addWidget(self._source_label)
        for section in self._sections:
            form.addWidget(section)
        form.addStretch(1)
        inner = QWidget()
        inner.setLayout(form)

        # Folding is what usually keeps this inside the dock, but a screen can
        # still be too short for the sections that are open.
        scroll = QScrollArea()
        scroll.setWidget(inner)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)

        layout = QVBoxLayout()
        layout.addWidget(scroll, 1)
        layout.addLayout(self._button_rows())
        self.setLayout(layout)
        self.setEnabled(False)
        self._update_enabled()

    # -- groups -------------------------------------------------------------

    @staticmethod
    def _form(rows):
        """A widget holding a label/field form, for a collapsible section."""
        layout = QFormLayout()
        layout.setContentsMargins(6, 0, 0, 0)
        for row in rows:
            layout.addRow(*row) if isinstance(row, tuple) else layout.addRow(row)
        widget = QWidget()
        widget.setLayout(layout)
        return widget

    def _correction_box(self):
        """What the complementary displacement is corrected by, and when."""
        self._correction = QComboBox()
        for name in COMPLEMENTARY_CORRECTIONS:
            self._correction.addItem(name, name)
        self._correction.setToolTip(
            "ratio — the node's: |lidar_nondeg projected on comp| / |comp_nondeg|,\n"
            "one frame measured against its own odometry.\n"
            "line_x_axis — where this frame's own degenerate line crosses the\n"
            "complementary axis. The same measurement as the ratio, read off the\n"
            "picture and signed: where the ratio reports the size of a negative\n"
            "crossing, this reports no sample.\n"
            "lines_meet_x — the along-complementary coordinate of the point where\n"
            "the recent degenerate lines meet, in units of |comp|. Needs the lines\n"
            "to have turned relative to each other.\n"
            "lines_meet_xy — that whole point. The displacement is moved onto where\n"
            "the lines say the robot ended up, so it is turned as well as stretched;\n"
            "the only mode that can express a lateral odometry error.")
        self._correction.currentIndexChanged.connect(lambda _i: self._update_enabled())

        self._apply_scale = QCheckBox("scaleEstimationApply")
        self._apply_scale.setToolTip(
            "Off estimates and logs the correction without applying it.")
        self._baseline_lag = self._int(1, 10000)
        self._min_speed = self._double(0.0, 100.0, 0.01, 3)
        self._min_speed.setToolTip(
            "Below this LiDAR speed [m/s] over the lag window a frame produces no\n"
            "sample. Measured on the LiDAR displacement alone — see the README.")

        return self._form([
            ("complementaryCorrection", self._correction),
            ("scaleBaselineFrameLag", self._baseline_lag),
            ("scaleMinNonDegenerateSpeed", self._min_speed),
            self._apply_scale,
        ])

    def _lines_box(self):
        """Settings of the two lines-meet methods."""
        self._line_history = self._int(0, 100000)
        self._line_history.setToolTip(
            "How many earlier frames' degenerate lines the meeting point is fitted\n"
            "through. More span means more turn, which is what makes them meet.\n"
            "0 fits nothing, and also turns off the trail the anchor view draws.")
        self._line_history_step = self._int(1, 1000)
        self._line_history_step.setToolTip(
            "Take every Nth earlier line. Consecutive frames' lines are nearly\n"
            "identical, so a stride buys span at the same cost: the window reaches\n"
            "back scaleLineHistory x this many frames.")
        self._line_fit_norm = QComboBox()
        for name in LINE_FIT_NORMS:
            self._line_fit_norm.addItem(name, name)
        self._line_fit_norm.setToolTip(
            "l2 — least squares: a wrong line pulls the point in proportion to\n"
            "how wrong it is.\n"
            "l1 — least absolute deviations, by IRLS: each line gets one vote\n"
            "whatever its error. Also decides how the anchor view draws it.")
        self._lateral_max = self._double(0.0, UNBOUNDED_SCALE, 0.05, 3)
        self._lateral_max.setSpecialValueText("unbounded")
        self._lateral_max.setToolTip(
            "Symmetric bound on the cross-track coordinate, in units of |comp|,\n"
            "before it enters the smoothing filter: 0.2 lets the correction turn\n"
            "the displacement by at most about 11 degrees. Clamps, does not drop.\n"
            "Only lines_meet_xy applies this coordinate. Set to 0 for no bound.")
        self._line_max_sigma = self._double(0.0, UNBOUNDED_SCALE, 0.01, 3)
        self._line_max_sigma.setSpecialValueText("any fit")
        self._line_max_sigma.setToolTip(
            "Largest uncertainty along the scale axis, in units of |comp|, that\n"
            "still counts as a sample: 0.05 means \"only answer when the lines pin\n"
            "the scale to better than 5 percent\". A rejected frame contributes\n"
            "nothing, so the smoothing window keeps applying what the last frames\n"
            "that did answer said. Set to 0 to take whatever the fit returns.")
        self._line_min_leg_lines = self._int(0, 100000)
        self._line_min_leg_lines.setToolTip(
            "How many of the fitted lines must come from the other group of\n"
            "directions — the opposite leg of a zig-zag. This is geometry the\n"
            "sigma above cannot see: under l1 a bundle of near-parallel lines that\n"
            "agree closely with each other reports a small spread while crossing at\n"
            "a glancing angle. 0 turns the test off.")
        self._line_min_leg_lines.valueChanged.connect(self._update_enabled)
        self._line_min_leg_sep = self._double(0.0, 179.0, 1.0, 1)
        self._line_min_leg_sep.setToolTip(
            "How far apart, in degrees, the two groups of line directions must be\n"
            "to count as two. Enters as a deadband of half this on either side of\n"
            "the lines' mean orientation, so lines nearer the middle than that\n"
            "belong to neither group.")

        lines = self._form([
            ("scaleLineHistory", self._line_history),
            ("scaleLineHistoryStep", self._line_history_step),
            ("scaleLineFitNorm", self._line_fit_norm),
            ("scaleLineMaxScaleSigma", self._line_max_sigma),
            ("scaleLineMinLegLines", self._line_min_leg_lines),
            ("scaleLineMinLegSeparationDeg", self._line_min_leg_sep),
            ("scaleLateralMax", self._lateral_max),
        ])
        self._lines_form = lines.layout()
        return lines

    def _smoothing_box(self):
        """What every method's samples go through before being applied."""
        self._smoothing = self._int(1, 100000)
        # Which statistic the smoothing window collapses to. The samples are a
        # ratio with a heavy tail, so the choice is not cosmetic: a mean carries
        # an outlier's magnitude for the whole window, a median only its side.
        self._smoothing_mode = QComboBox()
        for mode in SMOOTHING_MODES:
            self._smoothing_mode.addItem(mode, mode)
        self._smoothing_mode.setToolTip(
            "mean — the node's own: the arithmetic mean of the window, so one\n"
            "20x sample shifts the applied scale for the next N frames.\n"
            "median — the middle sample: a new observation moves it one step\n"
            "towards itself whatever its magnitude.\n"
            "trimmed — drop what falls outside a 1.5 × IQR fence, average the\n"
            "rest: the mean on a clean window, the mean of the inliers on a\n"
            "window with a tail.")

        # A sample outside [scaleMin, scaleMax] enters the filter clamped to the
        # bound. 0 on the upper bound reads as "unbounded", which is also how an
        # absent scaleMax loads.
        self._scale_min = self._double(0.0, UNBOUNDED_SCALE, 0.05, 3)
        self._scale_max = self._double(0.0, UNBOUNDED_SCALE, 0.05, 3)
        self._scale_max.setSpecialValueText("unbounded")
        range_tip = ("Range the along-track sample is clamped into before it\n"
                     "enters the smoothing filter. An out-of-range sample is\n"
                     "capped, not dropped, so it still counts — it just cannot\n"
                     "pull the average past the bound. Not a node parameter.")
        self._scale_min.setToolTip(range_tip)
        self._scale_max.setToolTip(range_tip + "\nSet to 0 for no upper bound.")

        return self._form([
            ("scaleSmoothingWindowSize", self._smoothing),
            ("scaleSmoothingMode", self._smoothing_mode),
            ("scaleMin", self._scale_min),
            ("scaleMax", self._scale_max),
        ])

    def _general_box(self):
        self._translation_scale = self._double(0.0, 100.0, 0.01, 3)
        self._ignore_dz = QCheckBox("ignore_dz")
        return self._form([
            ("translationScale", self._translation_scale),
            self._ignore_dz,
        ])

    def _tool_box(self):
        self._correction_mode = QComboBox()
        for mode in CORRECTION_MODES:
            self._correction_mode.addItem(mode, mode)
        self._correction_mode.setToolTip(
            "How the corrected prediction is substituted into the degenerate\n"
            "directions — a separate question from what corrected it.")
        self._source_path = QLineEdit()
        self._source_path.setPlaceholderText("(use the odometry recorded online)")
        browse = QPushButton("…")
        browse.setMaximumWidth(32)
        browse.clicked.connect(self._browse_source)
        clear = QPushButton("clear")
        clear.setMaximumWidth(52)
        clear.clicked.connect(lambda: self._source_path.setText(""))
        self._source_path.textChanged.connect(lambda _t: self._update_enabled())
        source_row = QHBoxLayout()
        source_row.addWidget(self._source_path, 1)
        source_row.addWidget(browse)
        source_row.addWidget(clear)
        source_widget = QWidget()
        source_widget.setLayout(source_row)
        self._max_match_dt = self._double(0.0, 10.0, 0.01, 3)
        # A source slower than the LiDAR is the case interpolation exists for;
        # with it, max_match_dt_s gates the gap being interpolated across.
        self._match_mode = QComboBox()
        for mode in MATCH_MODES:
            self._match_mode.addItem(mode, mode)
        self._match_mode.setToolTip(
            "nearest — what the online node does: take the closest sample to each\n"
            "LiDAR stamp, and reject it beyond max_match_dt_s.\n"
            "interpolate — evaluate the source pose at the LiDAR stamp itself,\n"
            "between the samples bracketing it, so the twist spans exactly the\n"
            "LiDAR interval. For a source sparser than the LiDAR; max_match_dt_s\n"
            "then limits how wide a gap may be interpolated across.")

        # The stream's own mount, beside the stream. It is never inherited from
        # the recorded odometry -- that is a different sensor -- so a path with
        # no mount is refused rather than quietly given someone else's.
        self._source_mount_mode = QComboBox()
        for label, data in (("(unset)", None), ("run", "run"),
                            ("identity", "identity"), ("explicit", "explicit")):
            self._source_mount_mode.addItem(label, data)
        self._source_mount_mode.setToolTip(
            "The mount of this stream, T_source_to_lidar.\n"
            "run — this stream is the sensor the run recorded, so it shares that\n"
            "mount. complementary_odom_stream.tum is exactly that.\n"
            "identity — the stream is already in the LiDAR frame.\n"
            "explicit — the mount typed below, for a different sensor.\n"
            "(unset) — refused while a path is set: a wrong mount does not fail,\n"
            "it comes back out as a scale, so it has to be stated.")
        self._source_mount_mode.currentIndexChanged.connect(
            lambda _i: self._update_enabled())

        self._source_mount = _MountEditor(
            self,
            "Translation of T_source_to_lidar [m]: where this stream's frame sits\n"
            "in the LiDAR frame. Saved as complementary_source.extrinsicTrans.",
            "The same transform's rotation, as roll/pitch/yaw in degrees applied\n"
            "Rz(yaw) @ Ry(pitch) @ Rx(roll). Saved as extrinsicRot.")

        box = self._form([
            ("correction_mode", self._correction_mode),
            ("complementary source", source_widget),
            ("match_mode", self._match_mode),
            ("max_match_dt_s", self._max_match_dt),
            ("source mount", self._source_mount_mode),
        ] + self._source_mount.rows("source "))
        self._tool_form = box.layout()
        return box

    def _frame_box(self):
        """Which body frame the correction is measured in, and what placed it.

        The mounts themselves live with the odometry each describes -- the
        recorded one in its own section, an alternative stream's beside the path
        it belongs to -- so that neither can be mistaken for the other. This
        section only chooses the frame and reports which mount ended up placing
        it.
        """
        self._estimation_frame = QComboBox()
        for name in ESTIMATION_FRAMES:
            self._estimation_frame.addItem(name, name)
        self._estimation_frame.setToolTip(
            "lidar - the node's own: everything is measured in the LiDAR frame, so\n"
            "a body rotation about any other point translates the LiDAR by the lever\n"
            "arm between them. That translation is read as odometry error and then\n"
            "multiplied by the scale, which no fixed mount can obey.\n"
            "complementary - measure and apply at the odometry sensor's own origin,\n"
            "placed by the mount of whichever odometry is being corrected:\n"
            "base_link for a legged state estimator, the camera for visual odometry.\n"
            "An in-place rotation about that origin carries no translation, so it\n"
            "yields no sample and takes no correction.")
        self._estimation_frame.currentIndexChanged.connect(lambda _i: self._update_enabled())

        self._frame_use_rot = QCheckBox("adopt the extrinsic's rotation too")
        self._frame_use_rot.setToolTip(
            "Off moves only the origin and keeps the LiDAR's axes - the origin is\n"
            "what removes the lever arm, and the axes decide something else: the\n"
            "plane the degenerate lines are fitted in, what ignore_dz drops, and\n"
            "which way a simulated drift points.\n"
            "On adopts the odometry's own axes. Right for a base_link, wrong for a\n"
            "camera optical frame, whose z points forward - the lines would then be\n"
            "fitted in the vertical plane.")

        # Which odometry the last replay corrected, and the mount that placed
        # the frame. Two odometries can be configured at once; only one applies.
        self._active_label = QLabel("")
        self._active_label.setWordWrap(True)
        self._active_label.setStyleSheet("QLabel { color: palette(mid); }")

        box = self._form([
            ("estimationFrame", self._estimation_frame),
            self._frame_use_rot,
            self._active_label,
        ])
        self._frame_form = box.layout()
        return box

    def _recorded_box(self):
        """The mount of the odometry the node recorded into the CSV.

        Its own section because it is its own sensor. Nothing here is ever read
        for an alternative source, and nothing configured for a source is ever
        read here -- a mount taken from the wrong sensor does not fail, it comes
        back out as a scale.
        """
        # "No override" is a state of its own -- it reuses the run's recorded
        # mount -- and is not the same configuration as an identity one, so it
        # gets a checkbox rather than being inferred from six zeroed boxes.
        self._recorded_override = QCheckBox("override the run's extrinsic")
        self._recorded_override.setToolTip(
            "Off reuses the mount the run recorded in complementary_odom_meta.yaml\n"
            "(identity if it recorded none), exactly as omitting it from the config\n"
            "does. On applies the mount below instead — for a run whose meta file is\n"
            "missing or wrong. Saved as recorded_odometry.extrinsicTrans/Rot.")
        self._recorded_override.toggled.connect(self._on_recorded_override_toggled)

        self._recorded_mount = _MountEditor(
            self,
            "Translation of T_complementary_to_lidar [m]: where the recorded\n"
            "odometry's frame sits in the LiDAR frame. This is the lever arm the\n"
            "estimation frame moves by.",
            "The same transform's rotation, as roll/pitch/yaw in degrees applied\n"
            "Rz(yaw) @ Ry(pitch) @ Rx(roll) -- the convention ROS spells them in.\n"
            "A mount loaded from a file is handed on untouched unless edited here.")

        self._recorded_origin_label = QLabel("")
        self._recorded_origin_label.setWordWrap(True)
        self._recorded_origin_label.setStyleSheet("QLabel { color: palette(mid); }")

        box = self._form(
            [self._recorded_override] + self._recorded_mount.rows()
            + [self._recorded_origin_label])
        self._recorded_form = box.layout()
        return box

    def _drift_box(self):
        # Injecting a known error is what makes the correction checkable: you
        # know what should come back out.
        self._drift_alpha = self._double(-1.0, 1.0, 0.01, 4)
        self._drift_axis = QComboBox()
        for name in DRIFT_AXES:
            self._drift_axis.addItem(name, name)
        box = self._form([
            ("alpha (× distance)", self._drift_alpha),
            ("body axis", self._drift_axis),
        ])
        box.setToolTip(
            "Adds alpha × |displacement| along the chosen body axis of the\n"
            "complementary odometry, before any correction. 0 disables.\n"
            "Injected in the estimation frame, so recovering alpha is exact.\n"
            "A lateral alpha is exactly what lines_meet_xy exists to undo.")
        return box

    def _button_rows(self):
        self._apply_button = QPushButton("Apply && replay")
        self._apply_button.setToolTip(
            "Run the estimator over the loaded run with these values and redraw.\n"
            "The only thing in the window that replays: picking a run in the\n"
            "list reads it, and this is where you ask for the rest — so pressing\n"
            "this having changed nothing replays what you are previewing.\n"
            "An edit to the reference trajectory alone replays nothing: the\n"
            "replay never reads it, so it is refitted and redrawn on the spot.")
        self._apply_button.clicked.connect(self.applied.emit)
        self._revert_button = QPushButton("Revert")
        self._revert_button.clicked.connect(self.revert)

        # A config file is the unit a replay is described by, so both directions
        # belong here: read one in to reproduce someone's run, write one out to
        # hand yours on.
        self._load_button = QPushButton("Load YAML…")
        self._load_button.setToolTip(
            "Replace these values with a configuration file's, and re-run.")
        self._load_button.clicked.connect(self.load_requested.emit)
        self._save_button = QPushButton("Save YAML…")
        self._save_button.setToolTip(
            "Write the whole configuration to a file of your choosing — to share,\n"
            "or to hand to the CLI as --ros-params-yaml.")
        self._save_button.clicked.connect(self.save_requested.emit)

        # The other direction a configuration can go: into the run it belongs
        # to, so that replaying this run picks it up without anyone having to
        # remember which file described it.
        self._save_run_button = QPushButton("Save to run")
        self._save_run_button.setToolTip(
            "Write these settings into the loaded run's own replay_scale.yaml.\n"
            "Only what differs from the base configuration is written, so the run\n"
            "keeps what is specific to it — its mounts, its reference trajectory —\n"
            "and inherits the rest. Picked up automatically next time this run is\n"
            "replayed, by the viewer and by the CLI alike.")
        self._save_run_button.clicked.connect(self.save_run_requested.emit)

        # Revert goes back to what is loaded; this goes back to what the tool
        # ships with, and forgets what the viewer remembered from last time --
        # the one way out of a configuration that has been edited into a
        # corner, across launches as well as within one.
        self._reset_button = QPushButton("Reset defaults")
        self._reset_button.setToolTip(
            "Forget the remembered session and go back to the bundled\n"
            "configuration, then re-run.")
        self._reset_button.clicked.connect(self.reset_requested.emit)

        buttons = QHBoxLayout()
        buttons.addWidget(self._apply_button, 1)
        buttons.addWidget(self._revert_button)

        files = QHBoxLayout()
        files.addWidget(self._load_button)
        files.addWidget(self._save_button)
        files.addWidget(self._save_run_button)
        files.addWidget(self._reset_button)

        rows = QVBoxLayout()
        rows.addLayout(buttons)
        rows.addLayout(files)
        return rows

    # -- widget helpers -----------------------------------------------------

    @staticmethod
    def _double(lo, hi, step, decimals):
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setSingleStep(step)
        w.setDecimals(decimals)
        return w

    @staticmethod
    def _int(lo, hi):
        w = QSpinBox()
        w.setRange(lo, hi)
        return w

    @staticmethod
    def _row(widgets):
        """Several fields on one line, as one form row.

        A mount is three numbers that mean nothing apart, so they are read and
        enabled as a unit rather than as three rows that happen to be adjacent.
        """
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        for widget in widgets:
            layout.addWidget(widget, 1)
        holder = QWidget()
        holder.setLayout(layout)
        return holder

    @staticmethod
    def _set_row_enabled(form, field, enabled):
        """Grey a field out together with its label, which is a separate widget."""
        field.setEnabled(enabled)
        label = form.labelForField(field)
        if label is not None:
            label.setEnabled(enabled)

    def _browse_source(self):
        start = os.path.dirname(self._source_path.text()) or os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self, "Complementary odometry (TUM)", start, "TUM trajectories (*.tum);;All files (*)")
        if path:
            self._source_path.setText(path)

    def _reference_box(self):
        """An external trajectory drawn alongside the replayed ones."""
        self._reference_path = QLineEdit()
        self._reference_path.setPlaceholderText("(none)")
        browse = QPushButton("…")
        browse.setMaximumWidth(32)
        browse.clicked.connect(self._browse_reference)
        clear = QPushButton("clear")
        clear.setMaximumWidth(52)
        clear.clicked.connect(lambda: self._reference_path.setText(""))
        row = QHBoxLayout()
        row.addWidget(self._reference_path, 1)
        row.addWidget(browse)
        row.addWidget(clear)
        path_widget = QWidget()
        path_widget.setLayout(row)

        self._reference_label = QLineEdit()
        self._reference_label.setPlaceholderText("(from the filename)")
        self._reference_align = QComboBox()
        for name in REFERENCE_ALIGNMENTS:
            self._reference_align.addItem(name, name)
        self._reference_align.setToolTip(
            "first_position_yaw - anchor its first matched position on the replay's,\n"
            "then turn it about the vertical by the angle that best fits the replayed\n"
            "trajectory. What a total station wants: it reads only positions, which\n"
            "is all such a reference has, and fits only the heading, which is all\n"
            "that is unknown between two levelled frames — so a real tilt still shows.\n"
            "first_position_rotation - the same with the full 3D rotation fitted, for\n"
            "a reference frame that is not levelled. It absorbs tilt error too.\n"
            "first_pose - anchor position and orientation at the replay's first pose.\n"
            "No fit, but it trusts the orientation in the file.\n"
            "none - draw it in its own coordinates.\n"
            "No alignment ever fits a scale: a scale is the quantity under test.")

        # Seconds, but the offset between a separately logged clock and a ROS
        # one is the whole epoch -- 1.8e9 -- so the range has to hold it. The
        # replay prints the value that lines the two starts up.
        self._reference_offset = self._double(-1e12, 1e12, 1.0, 3)
        self._reference_offset.setToolTip(
            "Added to the reference's stamps before it is matched against the\n"
            "replay. A fitted alignment needs correspondences, and a reference on\n"
            "its own clock has none until this brings the two spans together.\n"
            "Run once with it at 0: the replay prints the offset that would fit.")

        box = self._form([
            ("TUM file", path_widget),
            ("label", self._reference_label),
            ("align", self._reference_align),
            ("time_offset_s", self._reference_offset),
        ])
        box.setToolTip(
            "Ground truth, a survey, another system's output — anything in TUM\n"
            "form, drawn on the trajectory plot alongside the replays.\n"
            "The replay never reads it, so it cannot move an estimate.\n"
            "Kept per run: it is not remembered between loads, so use\n"
            "\"Save to run\" to keep it with the run it was recorded with.\n"
            "Loading another run shows that run's own reference, or none.")
        return box

    def _browse_reference(self):
        start = os.path.dirname(self._reference_path.text()) or os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self, "Reference trajectory (TUM)", start, "TUM trajectories (*.tum);;All files (*)")
        if path:
            self._reference_path.setText(path)

    def _update_enabled(self):
        """Fold and grey out the settings the selected correction does not read.

        They keep their values -- switching methods and back must not lose what
        was typed -- so this only says which of them are in play.
        """
        correction = self._correction.currentData()
        self._lines_section.set_relevant(correction in LINES_MEET_CORRECTIONS)
        lateral = correction == "lines_meet_xy"
        self._set_row_enabled(self._lines_form, self._lateral_max, lateral)
        # The deadband the separation sets only exists once the legs are counted.
        self._set_row_enabled(self._lines_form, self._line_min_leg_sep,
                              self._line_min_leg_lines.value() > 0)

        # Only the complementary frame reads a mount's rotation; the mounts
        # themselves stay live either way, since a source swap needs one.
        self._frame_use_rot.setEnabled(
            self._estimation_frame.currentData() == "complementary")

        self._recorded_mount.set_enabled(self._recorded_form,
                                         self._recorded_override.isChecked())
        self._source_mount.set_enabled(
            self._tool_form, self._source_mount_mode.currentData() == "explicit")
        self._set_row_enabled(self._tool_form, self._source_mount_mode,
                              bool(self._source_path.text().strip()))
        self._update_mount_labels()

    # -- state --------------------------------------------------------------

    def set_config(self, settings, params, *, source=""):
        """Adopt a configuration as the new baseline and show it.

        ``source`` is the file it came from, for the provenance line; pass it
        whenever a file was involved, omit it to leave the line as it was.
        """
        self._settings, self._params = settings, params
        if source:
            self.set_source(source)
        self.revert()
        self.setEnabled(True)

    def set_source(self, path, note=""):
        """Note which configuration file the shown values came from.

        ``note`` qualifies it -- how a file the user never chose came to be the
        one on screen.
        """
        text = f"From {path}" if path else ""
        if text and note:
            text += f" ({note})"
        self._source_label.setText(text)
        self._source_label.setToolTip(path)

    def revert(self):
        """Discard edits, restoring the last applied configuration."""
        if self._settings is None:
            return
        s, p = self._settings, self._params
        self._correction.setCurrentIndex(
            max(0, self._correction.findData(p.complementary_correction)))
        self._baseline_lag.setValue(p.scale_baseline_frame_lag)
        self._min_speed.setValue(p.scale_min_nondegenerate_speed)
        self._apply_scale.setChecked(p.scale_estimation_apply)

        self._line_history.setValue(p.scale_line_history)
        self._line_history_step.setValue(p.scale_line_history_step)
        self._line_fit_norm.setCurrentIndex(
            max(0, self._line_fit_norm.findData(p.scale_line_fit_norm)))
        self._lateral_max.setValue(
            0.0 if p.scale_lateral_max > UNBOUNDED_SCALE else p.scale_lateral_max)
        self._line_max_sigma.setValue(
            0.0 if p.scale_line_max_scale_sigma > UNBOUNDED_SCALE
            else p.scale_line_max_scale_sigma)
        self._line_min_leg_lines.setValue(p.scale_line_min_leg_lines)
        self._line_min_leg_sep.setValue(p.scale_line_min_leg_separation_deg)

        self._smoothing.setValue(p.scale_smoothing_window_size)
        self._smoothing_mode.setCurrentIndex(
            max(0, self._smoothing_mode.findData(p.scale_smoothing_mode)))
        self._scale_min.setValue(min(p.scale_min, UNBOUNDED_SCALE))
        self._scale_max.setValue(0.0 if p.scale_max > UNBOUNDED_SCALE else p.scale_max)

        self._translation_scale.setValue(p.translation_scale)
        self._ignore_dz.setChecked(p.ignore_dz)

        self._estimation_frame.setCurrentIndex(
            max(0, self._estimation_frame.findData(p.estimation_frame)))
        self._frame_use_rot.setChecked(p.estimation_frame_use_extrinsic_rot)

        self._correction_mode.setCurrentIndex(
            max(0, self._correction_mode.findData(s.correction_mode)))
        self._source_path.setText(s.complementary_source.path)
        self._match_mode.setCurrentIndex(
            max(0, self._match_mode.findData(s.complementary_source.match_mode)))
        self._max_match_dt.setValue(s.complementary_source.max_match_dt_s)
        self._drift_alpha.setValue(s.complementary_drift.alpha)
        self._drift_axis.setCurrentIndex(
            max(0, self._drift_axis.findData(s.complementary_drift.axis)))
        self._reference_path.setText(s.reference_trajectory.path)
        self._reference_label.setText(s.reference_trajectory.label)
        self._reference_align.setCurrentIndex(
            max(0, self._reference_align.findData(s.reference_trajectory.align)))
        self._reference_offset.setValue(s.reference_trajectory.time_offset_s)
        self._show_mounts(s)
        self._update_enabled()

    def _show_mounts(self, settings):
        """Put both mounts on screen from a configuration.

        A mount the config leaves out means "use what the run recorded", which
        is a different statement from an identity mount and cannot be inferred
        from six zeroed boxes -- so it is a checkbox, and the boxes underneath
        then show what the run actually recorded rather than whatever was last
        typed. A stale value in a disabled box reads as if it applied.
        """
        configured = settings.recorded_odometry.extrinsic
        self._recorded_override.setChecked(configured is not None)
        self._recorded_mount.fill(configured if configured is not None else self._run_mount)

        source = settings.complementary_source.extrinsic
        mode = source if isinstance(source, str) else ("explicit" if source is not None
                                                       else None)
        self._source_mount_mode.setCurrentIndex(
            max(0, self._source_mount_mode.findData(mode)))
        if not isinstance(source, str):
            self._source_mount.fill(source)
        self._update_mount_labels()

    def _on_recorded_override_toggled(self, on):
        """Unticking puts the run's own mount back on screen."""
        if not on:
            self._recorded_mount.fill(self._run_mount)
        self._update_enabled()

    def set_mounts(self, active, run_mount):
        """Note what the last replay resolved: the run's mount, and what applied.

        The panel cannot work either out for itself -- the run's mount comes
        from a meta file only the pipeline reads, and which odometry applies
        depends on it. Called after a successful replay.
        """
        self._active = active
        self._run_mount = None if run_mount is None else np.asarray(run_mount, dtype=float)
        if not self._recorded_override.isChecked():
            self._recorded_mount.fill(self._run_mount)
        self._update_mount_labels()

    def _update_mount_labels(self):
        if self._recorded_override.isChecked():
            self._recorded_origin_label.setText(
                "overriding — the values above are used, and are saved to the config")
        elif self._run_mount is None:
            self._recorded_origin_label.setText(
                "not overriding — the run's own mount is used "
                "(run a replay to see which)")
        else:
            offset = self._run_mount[:3, 3]
            self._recorded_origin_label.setText(
                f"not overriding — showing the run's own mount; origin "
                f"{offset[0]:+.3f}, {offset[1]:+.3f}, {offset[2]:+.3f} m "
                f"({float(np.linalg.norm(offset)):.3f} m of lever arm)")

        if self._active is None:
            self._active_label.setText("")
            return
        offset = np.asarray(self._active.extrinsic, dtype=float)[:3, 3]
        self._active_label.setText(
            f"corrected last run: {self._active.label} — mount from "
            f"{self._active.origin} ({float(np.linalg.norm(offset)):.3f} m of lever arm)")

    def _edited_source_extrinsic(self):
        """What complementary_source.extrinsic should be, from the widgets.

        None when there is no stream: a mount describes a stream, and one left
        configured for a stream that is not being replayed is exactly the leak
        the two editors exist to prevent.
        """
        if not self._source_path.text().strip():
            return None
        mode = self._source_mount_mode.currentData()
        if mode == "explicit":
            return self._source_mount.matrix()
        return mode        # "run", "identity", or None for (unset)

    def edited_config(self):
        """(settings, params) reflecting the current widget values.

        Returns fresh objects; the baseline is only replaced once a run using
        them succeeds, so a failed run leaves Revert meaningful.
        """
        from dataclasses import replace

        source = replace(
            self._settings.complementary_source,
            path=self._source_path.text().strip(),
            max_match_dt_s=self._max_match_dt.value(),
            match_mode=self._match_mode.currentData(),
            extrinsic=self._edited_source_extrinsic(),
        )
        recorded = replace(
            self._settings.recorded_odometry,
            extrinsic=(self._recorded_mount.matrix()
                       if self._recorded_override.isChecked() else None),
        )
        drift = replace(
            self._settings.complementary_drift,
            alpha=self._drift_alpha.value(),
            axis=self._drift_axis.currentData(),
        )
        reference = replace(
            self._settings.reference_trajectory,
            path=self._reference_path.text().strip(),
            label=self._reference_label.text().strip(),
            align=self._reference_align.currentData(),
            time_offset_s=self._reference_offset.value(),
        )
        settings = self._settings.evolve(
            correction_mode=self._correction_mode.currentData(),
            recorded_odometry=recorded,
            complementary_source=source,
            complementary_drift=drift,
            reference_trajectory=reference,
        ).validated()

        scale_max = self._scale_max.value()
        lateral_max = self._lateral_max.value()
        max_sigma = self._line_max_sigma.value()
        params = validate_replay_params(replace(
            self._params,
            complementary_correction=self._correction.currentData(),
            estimation_frame=self._estimation_frame.currentData(),
            estimation_frame_use_extrinsic_rot=self._frame_use_rot.isChecked(),
            scale_baseline_frame_lag=self._baseline_lag.value(),
            scale_min_nondegenerate_speed=self._min_speed.value(),
            scale_estimation_apply=self._apply_scale.isChecked(),
            scale_line_history=self._line_history.value(),
            scale_line_history_step=self._line_history_step.value(),
            scale_line_fit_norm=self._line_fit_norm.currentData(),
            scale_lateral_max=lateral_max if lateral_max > 0.0 else float("inf"),
            scale_line_max_scale_sigma=max_sigma if max_sigma > 0.0 else float("inf"),
            scale_line_min_leg_lines=self._line_min_leg_lines.value(),
            scale_line_min_leg_separation_deg=self._line_min_leg_sep.value(),
            scale_smoothing_window_size=self._smoothing.value(),
            scale_smoothing_mode=self._smoothing_mode.currentData(),
            scale_min=self._scale_min.value(),
            scale_max=scale_max if scale_max > 0.0 else float("inf"),
            translation_scale=self._translation_scale.value(),
            ignore_dz=self._ignore_dz.isChecked(),
        ))
        return settings, params

    def set_busy(self, busy):
        self._apply_button.setEnabled(not busy)
        self._revert_button.setEnabled(not busy)
        # Loading a file re-runs the replay, so it waits for the current one.
        self._load_button.setEnabled(not busy)
        # So does resetting.
        self._reset_button.setEnabled(not busy)
