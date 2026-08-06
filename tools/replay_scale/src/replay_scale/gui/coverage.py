"""Coverage strip: which frames have anything to show, at a glance.

Two stacked rows because the two facts are independent -- a frame can be
degenerate with no complementary window, and with an alternative odometry
source most degenerate frames are. Painting one row would force a precedence
that hides exactly that population.
"""

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QWidget

from ..core.local_view import GATE_OBSERVABLE, NO_WINDOW, WINDOW

_ROW_H = 9
_GAP = 2
_CURSOR_COLOR = QColor("#d62728")

# Ordered states share one hue, light -> dark; degeneracy gets its own row.
_WINDOW_COLORS = {
    NO_WINDOW: QColor("#e8e8e8"),
    WINDOW: QColor("#9ecae1"),
    GATE_OBSERVABLE: QColor("#2171b5"),
}
_DEGENERATE_COLOR = QColor("#e6a23c")
_NOT_DEGENERATE_COLOR = QColor("#f2f2f2")


class CoverageStrip(QWidget):
    """Per-frame coverage bars; click or drag to seek."""

    seeked = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._views = []
        self._index = 0
        self.setMinimumHeight(2 * _ROW_H + _GAP)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip("Top: complementary window state.  Bottom: degeneracy.\n"
                        "Click to jump to a frame.")

    def set_views(self, views):
        self._views = views
        self._index = 0
        self.update()

    def set_index(self, index):
        self._index = int(index)
        self.update()

    def sizeHint(self):
        return QSize(400, 2 * _ROW_H + _GAP)

    def _frame_at(self, x):
        if not self._views:
            return 0
        frac = min(max(x / max(1, self.width()), 0.0), 1.0)
        return min(len(self._views) - 1, int(frac * len(self._views)))

    def mousePressEvent(self, event):
        if self._views:
            self.seeked.emit(self._frame_at(event.position().x()))

    def mouseMoveEvent(self, event):
        if self._views and event.buttons() & Qt.LeftButton:
            self.seeked.emit(self._frame_at(event.position().x()))

    def paintEvent(self, _event):
        painter = QPainter(self)
        width, n = self.width(), len(self._views)
        if n == 0:
            painter.fillRect(0, 0, width, self.height(), QColor("#f7f7f7"))
            return

        # One column per pixel, taking the most interesting frame it covers, so
        # a lone observable frame in a dead stretch stays visible.
        for x in range(width):
            lo = int(x * n / width)
            hi = max(lo + 1, int((x + 1) * n / width))
            chunk = self._views[lo:hi]
            state = max(v.window_state for v in chunk)
            degenerate = any(v.degeneracy_detected for v in chunk)
            painter.fillRect(x, 0, 1, _ROW_H, _WINDOW_COLORS[state])
            painter.fillRect(x, _ROW_H + _GAP, 1, _ROW_H,
                             _DEGENERATE_COLOR if degenerate else _NOT_DEGENERATE_COLOR)

        cursor_x = int((self._index + 0.5) * width / n)
        painter.fillRect(max(0, cursor_x - 1), 0, 2, self.height(), _CURSOR_COLOR)
