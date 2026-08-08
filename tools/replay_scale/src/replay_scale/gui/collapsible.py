"""A section of a form that can be folded away.

The configuration dock holds more parameters than any one replay reads: three
of the four corrections ignore the line-fit settings entirely, and the drift
simulator is off in most sessions. Hiding what is not in play is what keeps the
panel readable, so each group is a section that folds -- and the ones the
selected method does not read fold themselves.

Folding hides, it never edits: a collapsed section keeps its values and still
contributes them to :meth:`ParamsPanel.edited_config`.
"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QToolButton, QVBoxLayout, QWidget


class CollapsibleSection(QWidget):
    """A titled header button with a widget underneath that it shows or hides."""

    def __init__(self, title, content, parent=None, *, expanded=True):
        super().__init__(parent)
        self._content = content

        self._button = QToolButton()
        self._button.setText(title)
        self._button.setCheckable(True)
        self._button.setChecked(expanded)
        self._button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self._button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self._button.setStyleSheet("QToolButton { border: none; font-weight: bold; }")
        self._button.toggled.connect(self._on_toggled)

        # A rule under the header, so sections read as sections rather than as
        # one long list of controls.
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(2)
        layout.addWidget(self._button)
        layout.addWidget(line)
        layout.addWidget(content)
        self.setLayout(layout)
        content.setVisible(expanded)

    def _on_toggled(self, checked):
        self._button.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
        self._content.setVisible(checked)

    def set_expanded(self, expanded):
        self._button.setChecked(bool(expanded))

    def is_expanded(self):
        return self._button.isChecked()

    def set_relevant(self, relevant, *, fold=True):
        """Mark the section as read (or not) by the current configuration.

        Irrelevant sections are greyed out and folded away, but they are never
        emptied: switching methods and back must not lose what was typed. Only
        auto-folds; a section the user has opened stays open unless ``fold``.
        """
        self._content.setEnabled(bool(relevant))
        self._button.setEnabled(True)          # still foldable, to look inside
        if fold:
            self.set_expanded(bool(relevant))
