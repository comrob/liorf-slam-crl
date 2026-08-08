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

import os

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
    LINES_MEET_CORRECTIONS,
    SMOOTHING_MODES,
)
from ..core.odom_source import DRIFT_AXES, MATCH_MODES
from ..settings import validate_replay_params
from .collapsible import CollapsibleSection

#: Largest finite bound the spin boxes offer; an infinite scaleMax or
#: scaleLateralMax shows as 0 ("unbounded") instead, since a spin box cannot
#: hold infinity.
UNBOUNDED_SCALE = 1000.0


class ParamsPanel(QWidget):
    """Form over the estimator parameters and the replay tool settings."""

    applied = Signal()          # user wants the replay re-run with these values
    save_requested = Signal()
    load_requested = Signal()   # user wants a config file read in

    def __init__(self, parent=None):
        super().__init__(parent)
        self._settings = None
        self._params = None

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
            self._lines_section,
            CollapsibleSection("smoothing and bounds", self._smoothing_box()),
            CollapsibleSection("complementaryOdom (general)", self._general_box(),
                               expanded=False),
            CollapsibleSection("replay_scale_tool", self._tool_box(), expanded=False),
            CollapsibleSection("simulated complementary drift", self._drift_box(),
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

        lines = self._form([
            ("scaleLineHistory", self._line_history),
            ("scaleLineHistoryStep", self._line_history_step),
            ("scaleLineFitNorm", self._line_fit_norm),
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

        return self._form([
            ("correction_mode", self._correction_mode),
            ("complementary source", source_widget),
            ("match_mode", self._match_mode),
            ("max_match_dt_s", self._max_match_dt),
        ])

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
            "A lateral alpha is exactly what lines_meet_xy exists to undo.")
        return box

    def _button_rows(self):
        self._apply_button = QPushButton("Apply && re-run")
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
        self._save_button.clicked.connect(self.save_requested.emit)

        buttons = QHBoxLayout()
        buttons.addWidget(self._apply_button, 1)
        buttons.addWidget(self._revert_button)

        files = QHBoxLayout()
        files.addWidget(self._load_button)
        files.addWidget(self._save_button)

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

    def _browse_source(self):
        start = os.path.dirname(self._source_path.text()) or os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self, "Complementary odometry (TUM)", start, "TUM trajectories (*.tum);;All files (*)")
        if path:
            self._source_path.setText(path)

    def _update_enabled(self):
        """Fold and grey out the settings the selected correction does not read.

        They keep their values -- switching methods and back must not lose what
        was typed -- so this only says which of them are in play.
        """
        correction = self._correction.currentData()
        self._lines_section.set_relevant(correction in LINES_MEET_CORRECTIONS)
        lateral = correction == "lines_meet_xy"
        self._lateral_max.setEnabled(lateral)
        label = self._lines_form.labelForField(self._lateral_max)
        if label is not None:
            label.setEnabled(lateral)

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

    def set_source(self, path):
        """Note which configuration file the shown values came from."""
        self._source_label.setText(f"From {path}" if path else "")
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

        self._smoothing.setValue(p.scale_smoothing_window_size)
        self._smoothing_mode.setCurrentIndex(
            max(0, self._smoothing_mode.findData(p.scale_smoothing_mode)))
        self._scale_min.setValue(min(p.scale_min, UNBOUNDED_SCALE))
        self._scale_max.setValue(0.0 if p.scale_max > UNBOUNDED_SCALE else p.scale_max)

        self._translation_scale.setValue(p.translation_scale)
        self._ignore_dz.setChecked(p.ignore_dz)

        self._correction_mode.setCurrentIndex(
            max(0, self._correction_mode.findData(s.correction_mode)))
        self._source_path.setText(s.complementary_source.path)
        self._match_mode.setCurrentIndex(
            max(0, self._match_mode.findData(s.complementary_source.match_mode)))
        self._max_match_dt.setValue(s.complementary_source.max_match_dt_s)
        self._drift_alpha.setValue(s.complementary_drift.alpha)
        self._drift_axis.setCurrentIndex(
            max(0, self._drift_axis.findData(s.complementary_drift.axis)))
        self._update_enabled()

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
        )
        drift = replace(
            self._settings.complementary_drift,
            alpha=self._drift_alpha.value(),
            axis=self._drift_axis.currentData(),
        )
        settings = self._settings.evolve(
            correction_mode=self._correction_mode.currentData(),
            complementary_source=source,
            complementary_drift=drift,
        ).validated()

        scale_max = self._scale_max.value()
        lateral_max = self._lateral_max.value()
        params = validate_replay_params(replace(
            self._params,
            complementary_correction=self._correction.currentData(),
            scale_baseline_frame_lag=self._baseline_lag.value(),
            scale_min_nondegenerate_speed=self._min_speed.value(),
            scale_estimation_apply=self._apply_scale.isChecked(),
            scale_line_history=self._line_history.value(),
            scale_line_history_step=self._line_history_step.value(),
            scale_line_fit_norm=self._line_fit_norm.currentData(),
            scale_lateral_max=lateral_max if lateral_max > 0.0 else float("inf"),
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
