"""Scale-over-the-run tab: what the estimator produced, frame by frame.

The anchor-frame tab shows the geometry one scale sample is computed from; this
one shows the sequence those samples form -- the raw ratio, the smoothed mean
of the observable ones, and what the trajectory was actually built with. Reading
them together is how a suspicious frame gets traced back to its cause: scrub to
where the applied curve steps, and the anchor tab shows why.
"""

from PySide6.QtWidgets import QCheckBox

from ..plotting import draw_scale_history
from .plot_view import MARKER_COLOR, MarkerPlotView


class ScaleView(MarkerPlotView):
    """Time series of the estimator's scale, with the scrubbed frame marked."""

    def __init__(self, parent=None):
        super().__init__(parent, figsize=(9, 5))
        self._marker = None
        self._times = []
        self._index = 0
        self._geometries = []
        self._params = None
        self._title = ""

        # Linear by default: a deviation reads as the number it is. Log is
        # there for a run whose estimate spans decades, where it is the axis a
        # ratio deserves -- 2 and 0.5 the same distance from 1.
        self._log_y = QCheckBox("log scale")
        self._log_y.setToolTip(
            "Logarithmic y-axis. A scale of 2 and a scale of 0.5 are the same\n"
            "error in opposite directions, and only a log axis draws them so.")
        self._log_y.toggled.connect(lambda _c: self._draw())
        self.layout().addWidget(self._log_y)

    def set_geometries(self, geometries, *, params=None, title=""):
        """Redraw for a freshly loaded (or re-run) replay."""
        self._geometries, self._params, self._title = geometries, params, title
        self._draw()

    def _draw(self):
        geometries = self._geometries
        if not geometries:
            return
        draw_scale_history(self.ax, geometries, title=self._title,
                           scale_min=getattr(self._params, "scale_min", None),
                           scale_max=getattr(self._params, "scale_max", None),
                           log_y=self._log_y.isChecked())
        # Times are relative to the first frame, as the axis is.
        t0 = geometries[0].time
        self._times = [g.time - t0 for g in geometries]
        # After the curves, so it survives their ax.clear(). Redrawing keeps the
        # frame you were on -- toggling the axis is not a seek.
        self._marker = self.ax.axvline(self._time_at(self._index),
                                       color=MARKER_COLOR, linewidth=1.0, zorder=5)
        self.tight_layout()
        self.request_draw()

    def _time_at(self, index):
        if not self._times:
            return 0.0
        return self._times[max(0, min(int(index), len(self._times) - 1))]

    def set_index(self, index):
        """Move the marker to a frame index.

        A no-op before the first :meth:`set_geometries`, so the scrubber may
        signal a frame while nothing is loaded.
        """
        self._index = int(index)
        if self._marker is None or not self._times:
            return
        t = self._time_at(index)
        self._marker.set_xdata([t, t])
        self.request_draw()
