"""Editor for the settings a replay runs with.

Edits ``ReplayToolSettings`` / ``ReplayParams`` objects in memory and hands them
back to ``run_replay`` -- the same objects the YAML loader produces, so the GUI
cannot express a configuration the CLI could not. "Save YAML" writes exactly
that configuration back out, which is how a session stays reproducible
headlessly.

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
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..core.model import CORRECTION_MODES
from ..core.odom_source import DRIFT_AXES


class ParamsPanel(QWidget):
    """Form over the estimator parameters and the replay tool settings."""

    applied = Signal()          # user wants the replay re-run with these values
    save_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._settings = None
        self._params = None

        self._translation_scale = self._double(0.0, 100.0, 0.01, 3)
        self._min_speed = self._double(0.0, 100.0, 0.01, 3)
        self._baseline_lag = self._int(1, 10000)
        self._smoothing = self._int(1, 100000)
        self._apply_scale = QCheckBox("scaleEstimationApply")
        self._ignore_dz = QCheckBox("ignore_dz")

        estimator = QFormLayout()
        estimator.addRow("translationScale", self._translation_scale)
        estimator.addRow("scaleMinNonDegenerateSpeed", self._min_speed)
        estimator.addRow("scaleBaselineFrameLag", self._baseline_lag)
        estimator.addRow("scaleSmoothingWindowSize", self._smoothing)
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

        tool = QFormLayout()
        tool.addRow("correction_mode", self._correction_mode)
        tool.addRow("complementary source", source_widget)
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
        save_button = QPushButton("Save YAML…")
        save_button.clicked.connect(self.save_requested.emit)

        buttons = QHBoxLayout()
        buttons.addWidget(self._apply_button, 1)
        buttons.addWidget(self._revert_button)
        buttons.addWidget(save_button)

        layout = QVBoxLayout()
        layout.addWidget(estimator_box)
        layout.addWidget(tool_box)
        layout.addWidget(drift_box)
        layout.addLayout(buttons)
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

    def set_config(self, settings, params):
        """Adopt a configuration as the new baseline and show it."""
        self._settings, self._params = settings, params
        self.revert()
        self.setEnabled(True)

    def revert(self):
        """Discard edits, restoring the last applied configuration."""
        if self._settings is None:
            return
        s, p = self._settings, self._params
        self._translation_scale.setValue(p.translation_scale)
        self._min_speed.setValue(p.scale_min_nondegenerate_speed)
        self._baseline_lag.setValue(p.scale_baseline_frame_lag)
        self._smoothing.setValue(p.scale_smoothing_window_size)
        self._apply_scale.setChecked(p.scale_estimation_apply)
        self._ignore_dz.setChecked(p.ignore_dz)
        self._correction_mode.setCurrentIndex(
            max(0, self._correction_mode.findData(s.correction_mode)))
        self._source_path.setText(s.complementary_source.path)
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

        params = replace(
            self._params,
            translation_scale=self._translation_scale.value(),
            scale_min_nondegenerate_speed=self._min_speed.value(),
            scale_baseline_frame_lag=self._baseline_lag.value(),
            scale_smoothing_window_size=self._smoothing.value(),
            scale_estimation_apply=self._apply_scale.isChecked(),
            ignore_dz=self._ignore_dz.isChecked(),
        )
        return settings, params

    def set_busy(self, busy):
        self._apply_button.setEnabled(not busy)
        self._revert_button.setEnabled(not busy)
