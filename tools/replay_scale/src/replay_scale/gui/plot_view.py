"""Base for the whole-run tabs: a plot drawn once, marked while scrubbing.

Both whole-run views work the same way. The curves are expensive and static --
thousands of poses or samples, changing only when a replay is loaded -- while
the scrubber moves constantly and only ever moves a marker. Redrawing the
figure per slider step would make scrubbing crawl, and redrawing a hidden tab
would waste it entirely, so drawing is deferred until the tab is on screen.

Subclasses draw into :attr:`ax` and call :meth:`request_draw`.
"""

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from PySide6.QtWidgets import QVBoxLayout, QWidget

#: Colour of the current-frame marker, on every tab that has one.
MARKER_COLOR = "black"


class MarkerPlotView(QWidget):
    """A matplotlib canvas with a toolbar and a deferred-redraw policy."""

    def __init__(self, parent=None, *, figsize=(7, 7)):
        super().__init__(parent)
        self._figure = Figure(figsize=figsize)
        self._ax = self._figure.add_subplot(111)
        self._canvas = FigureCanvasQTAgg(self._figure)
        # True when the figure changed while hidden; redrawn on showEvent.
        self._dirty = False

        # Pan/zoom matters on these tabs in a way it does not on the anchor
        # view: the run is long, and the interesting part is a small stretch.
        toolbar = NavigationToolbar2QT(self._canvas, self)

        layout = QVBoxLayout()
        layout.addWidget(toolbar)
        layout.addWidget(self._canvas, 1)
        self.setLayout(layout)

    @property
    def ax(self):
        return self._ax

    def tight_layout(self):
        self._figure.tight_layout()

    def request_draw(self):
        """Draw now if visible, otherwise leave it for the next showEvent."""
        if self.isVisible():
            self._canvas.draw_idle()
            self._dirty = False
        else:
            self._dirty = True

    def showEvent(self, event):
        super().showEvent(event)
        if self._dirty:
            self._canvas.draw_idle()
            self._dirty = False
