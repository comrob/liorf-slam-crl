"""PySide6 viewer. A peer of ``cli``: both go through ``pipeline.run_replay``.

Importing this package pulls in PySide6, so nothing below the frontend layer
may import it. ``sources.py`` is deliberately Qt-free and testable on its own.
"""
