"""Scale-over-the-run tab: what the estimator produced, frame by frame.

The anchor-frame tab shows the geometry one sample is computed from; this one
shows the sequence those samples form -- the raw measurement, the smoothed
statistic over the observable ones, and what the trajectory was actually built
with. Reading them together is how a suspicious frame gets traced back to its
cause: scrub to where the applied curve steps, and the anchor tab shows why.

Under a correction that also moves the odometry sideways there are two curves
to read, not one, and they are different quantities -- one centred on 1, one on
0. They get their own stacked plot under a shared time axis rather than sharing
a y range that suits neither.
"""

from PySide6.QtWidgets import QCheckBox

from ..plotting import draw_scale_history, has_lateral
from .plot_view import MARKER_COLOR, MarkerPlotView


class ScaleView(MarkerPlotView):
    """Time series of the estimator's output, with the scrubbed frame marked."""

    def __init__(self, parent=None):
        super().__init__(parent, figsize=(9, 5))
        self._markers = []
        self._times = []
        self._index = 0
        self._geometries = []
        self._params = None
        self._title = ""

        # Linear by default: a deviation reads as the number it is. Log is
        # there for a run whose estimate spans decades, where it is the axis a
        # ratio deserves -- 2 and 0.5 the same distance from 1. It applies to
        # the scale only; the cross-track curve is signed and stays linear.
        self._log_y = QCheckBox("log scale")
        self._log_y.setToolTip(
            "Logarithmic y-axis for the scale. A scale of 2 and a scale of 0.5\n"
            "are the same error in opposite directions, and only a log axis\n"
            "draws them so. The lateral plot is signed and stays linear.")
        self._log_y.toggled.connect(lambda _c: self._draw())
        self.layout().addWidget(self._log_y)

    def set_geometries(self, geometries, *, params=None, title=""):
        """Redraw for a freshly loaded (or re-run) replay."""
        self._geometries, self._params, self._title = geometries, params, title
        self._draw()

    def _draw(self):
        geometries = self._geometries
        if not geometries:
            # A previewed run has none: it never ran the estimator, which is
            # the only thing that produces a scale. Say so rather than leaving
            # the previous run's history under this run's name.
            ax = self.set_rows(1)[0]
            ax.set_axis_off()
            ax.text(0.5, 0.5, "preview — \"Apply & replay\" to see the scale history",
                    ha="center", va="center", fontsize=11, color="#666666",
                    transform=ax.transAxes)
            self.request_draw()
            return
        split = has_lateral(geometries)
        axes = self.set_rows(2, height_ratios=(2, 1)) if split else self.set_rows(1)
        draw_scale_history(axes[0], geometries, title=self._title,
                           scale_min=getattr(self._params, "scale_min", None),
                           scale_max=getattr(self._params, "scale_max", None),
                           lateral_max=getattr(self._params, "scale_lateral_max", None),
                           log_y=self._log_y.isChecked(),
                           lateral_ax=axes[1] if split else None)
        # Times are relative to the first frame, as the axis is.
        t0 = geometries[0].time
        self._times = [g.time - t0 for g in geometries]
        # After the curves, so they survive each ax.clear(). Redrawing keeps the
        # frame you were on -- toggling the axis is not a seek.
        t = self._time_at(self._index)
        self._markers = [ax.axvline(t, color=MARKER_COLOR, linewidth=1.0, zorder=5)
                         for ax in axes]
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
        if not self._markers or not self._times:
            return
        t = self._time_at(index)
        for marker in self._markers:
            marker.set_xdata([t, t])
        self.request_draw()
