"""Whole-run trajectory tab: the same XY plot the plotting CLI writes.

The anchor-frame tab shows one frame at a time; this one shows where those
frames sit in the run. Both are drawn from the *same* in-memory replay, so
whatever the configuration panel last applied is what both tabs show -- no
re-reading of ``trajectories/*.tum``, which the GUI never writes.
"""

from ..plotting import draw_trajectories
from .plot_view import MARKER_COLOR, MarkerPlotView


class TrajectoryView(MarkerPlotView):
    """Top-down plot of every trajectory in the loaded replay."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._marker = None

    def set_curves(self, curves, *, title=""):
        """Redraw the plot for a freshly loaded (or re-run) replay."""
        draw_trajectories(self.ax, curves, title=title)
        # Added after the curves so it survives their ax.clear(), and re-legended
        # because draw_trajectories has already built the legend without it.
        self._marker, = self.ax.plot([], [], marker="o", markersize=7,
                                     color=MARKER_COLOR, linestyle="none",
                                     zorder=5, label="current frame")
        self.ax.legend()
        self.tight_layout()
        self.request_draw()

    def set_position(self, position):
        """Move the current-frame marker; ``position`` is a map-frame 3-vector.

        A no-op before the first :meth:`set_curves`, so the scrubber may signal
        a position while nothing is loaded.
        """
        if self._marker is None:
            return
        self._marker.set_data([float(position[0])], [float(position[1])])
        self.request_draw()
