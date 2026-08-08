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
        self._axes = [self._figure.add_subplot(111)]
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
        return self._axes[0]

    @property
    def axes(self):
        return self._axes

    def set_rows(self, n, *, height_ratios=None):
        """Rebuild the figure as ``n`` stacked axes sharing their x axis.

        A no-op when the figure already has that many, so a subclass may call it
        on every draw. Rebuilding drops every artist, which is why it is not
        done unconditionally: the marker would go with them.

        ``height_ratios`` is ignored unless it has one entry per row -- a caller
        that decides its row count per draw should not have to keep a matching
        ratio tuple in step with it.
        """
        if len(self._axes) == n:
            return self._axes
        self._figure.clear()
        gridspec = ({"height_ratios": height_ratios}
                    if height_ratios and len(height_ratios) == n else None)
        self._axes = list(self._figure.subplots(
            n, 1, sharex=n > 1, squeeze=False, gridspec_kw=gridspec)[:, 0])
        return self._axes

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
