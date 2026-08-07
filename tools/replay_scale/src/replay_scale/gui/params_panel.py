"""Editor for the settings a replay runs with.

Edits ``ReplayToolSettings`` / ``ReplayParams`` objects in memory and hands them
back to ``run_replay`` -- the same objects the YAML loader produces, so the GUI
cannot express a configuration the CLI could not. "Load YAML" reads a file into
those objects and "Save YAML" writes them back out, which is how a session
starts from someone else's configuration and stays reproducible headlessly.

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
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..core.model import CORRECTION_MODES, SMOOTHING_MODES
from ..core.odom_source import DRIFT_AXES, MATCH_MODES
from ..settings import validate_replay_params

#: Largest finite scale bound the spin boxes offer; an infinite scaleMax shows
#: as 0 ("unbounded") instead, since a spin box cannot hold infinity.
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

        self._translation_scale = self._double(0.0, 100.0, 0.01, 3)
        self._min_speed = self._double(0.0, 100.0, 0.01, 3)
        self._baseline_lag = self._int(1, 10000)
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

        self._apply_scale = QCheckBox("scaleEstimationApply")
        self._ignore_dz = QCheckBox("ignore_dz")

        # A sample outside [scaleMin, scaleMax] enters the filter clamped to the
        # bound. 0 on the upper bound reads as "unbounded", which is also how an
        # absent scaleMax loads.
        self._scale_min = self._double(0.0, UNBOUNDED_SCALE, 0.05, 3)
        self._scale_max = self._double(0.0, UNBOUNDED_SCALE, 0.05, 3)
        self._scale_max.setSpecialValueText("unbounded")
        range_tip = ("Range a scale sample is clamped into before it enters the\n"
                     "smoothing filter. An out-of-range sample is capped, not\n"
                     "dropped, so it still counts — it just cannot pull the average\n"
                     "past the bound. Not a node parameter — replay-time only.")
        self._scale_min.setToolTip(range_tip)
        self._scale_max.setToolTip(range_tip + "\nSet to 0 for no upper bound.")

        estimator = QFormLayout()
        estimator.addRow("translationScale", self._translation_scale)
        estimator.addRow("scaleMinNonDegenerateSpeed", self._min_speed)
        estimator.addRow("scaleBaselineFrameLag", self._baseline_lag)
        estimator.addRow("scaleSmoothingWindowSize", self._smoothing)
        estimator.addRow("scaleSmoothingMode", self._smoothing_mode)
        estimator.addRow("scaleMin", self._scale_min)
        estimator.addRow("scaleMax", self._scale_max)
        estimator.addRow(self._apply_scale)
        estimator.addRow(self._ignore_dz)
        estimator_box = QGroupBox("complementaryOdom")
        estimator_box.setLayout(estimator)

        self._correction_mode = QComboBox()
        for mode in CORRECTION_MODES:
            self._correction_mode.addItem(mode, mode)
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

        tool = QFormLayout()
        tool.addRow("correction_mode", self._correction_mode)
        tool.addRow("complementary source", source_widget)
        tool.addRow("match_mode", self._match_mode)
        tool.addRow("max_match_dt_s", self._max_match_dt)
        tool_box = QGroupBox("replay_scale_tool")
        tool_box.setLayout(tool)

        # Injecting a known error is what makes the estimated scale checkable:
        # you know what should come back out.
        self._drift_alpha = self._double(-1.0, 1.0, 0.01, 4)
        self._drift_axis = QComboBox()
        for name in DRIFT_AXES:
            self._drift_axis.addItem(name, name)
        drift = QFormLayout()
        drift.addRow("alpha (× distance)", self._drift_alpha)
        drift.addRow("body axis", self._drift_axis)
        drift_box = QGroupBox("simulated complementary drift")
        drift_box.setToolTip(
            "Adds alpha × |displacement| along the chosen body axis of the\n"
            "complementary odometry, before any scaling. 0 disables.")
        drift_box.setLayout(drift)

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

        layout = QVBoxLayout()
        layout.addWidget(self._source_label)
        layout.addWidget(estimator_box)
        layout.addWidget(tool_box)
        layout.addWidget(drift_box)
        layout.addLayout(buttons)
        layout.addLayout(files)
        layout.addStretch(1)
        self.setLayout(layout)
        self.setEnabled(False)

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
        self._translation_scale.setValue(p.translation_scale)
        self._min_speed.setValue(p.scale_min_nondegenerate_speed)
        self._baseline_lag.setValue(p.scale_baseline_frame_lag)
        self._smoothing.setValue(p.scale_smoothing_window_size)
        self._smoothing_mode.setCurrentIndex(
            max(0, self._smoothing_mode.findData(p.scale_smoothing_mode)))
        self._scale_min.setValue(min(p.scale_min, UNBOUNDED_SCALE))
        self._scale_max.setValue(0.0 if p.scale_max > UNBOUNDED_SCALE else p.scale_max)
        self._apply_scale.setChecked(p.scale_estimation_apply)
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
        params = validate_replay_params(replace(
            self._params,
            scale_min=self._scale_min.value(),
            scale_max=scale_max if scale_max > 0.0 else float("inf"),
            translation_scale=self._translation_scale.value(),
            scale_min_nondegenerate_speed=self._min_speed.value(),
            scale_baseline_frame_lag=self._baseline_lag.value(),
            scale_smoothing_window_size=self._smoothing.value(),
            scale_smoothing_mode=self._smoothing_mode.currentData(),
            scale_estimation_apply=self._apply_scale.isChecked(),
            ignore_dz=self._ignore_dz.isChecked(),
        ))
        return settings, params

    def set_busy(self, busy):
        self._apply_button.setEnabled(not busy)
        self._revert_button.setEnabled(not busy)
        # Loading a file re-runs the replay, so it waits for the current one.
        self._load_button.setEnabled(not busy)
