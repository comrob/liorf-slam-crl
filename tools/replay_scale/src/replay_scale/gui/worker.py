"""Background work, so a replay never blocks the event loop."""

from PySide6.QtCore import QObject, Signal

from ..pipeline import run_replay
from .sources import load_from_run_dir, preview_run_dir


class LoadWorker(QObject):
    """Runs one load on a worker thread: a replay, or a preview of one.

    A replay of a few thousand frames takes several seconds -- long enough that
    doing it on the event loop would read as a hang, and long enough that the
    viewer offers a preview instead when all you did was click a run. Both
    return the same :class:`~replay_scale.gui.sources.FrameData`, so the window
    shows one the way it shows the other.
    """

    progress = Signal(str)
    loaded = Signal(object)
    failed = Signal(str)

    def __init__(self, input_path, base_dir, *, settings=None, params=None,
                 config_path=None, frame="map", history=None,
                 history_step=None, observable_only=False, preview=False):
        super().__init__()
        self._preview = preview
        self._input_path = input_path
        self._base_dir = base_dir
        self._settings = settings
        self._params = params
        self._config_path = config_path
        self._frame = frame
        self._history = history
        self._history_step = history_step
        self._observable_only = observable_only

    def run(self):
        try:
            if self._preview:
                # None of the view options apply: a preview builds no per-frame
                # views, having nothing to build them from.
                data = preview_run_dir(
                    self._input_path, base_dir=self._base_dir,
                    settings=self._settings, params=self._params,
                    on_progress=self.progress.emit,
                    **({"config_path": self._config_path} if self._config_path else {}))
                self.loaded.emit(data)
                return
            kwargs = {}
            if self._history is not None:
                kwargs["history"] = self._history
            if self._history_step is not None:
                kwargs["history_step"] = self._history_step
            # Ignored when settings/params are given: those already came from a
            # file (or from the editor), and re-reading would contradict them.
            if self._config_path:
                kwargs["config_path"] = self._config_path
            data = load_from_run_dir(self._input_path, base_dir=self._base_dir,
                                     settings=self._settings, params=self._params,
                                     frame=self._frame, observable_only=self._observable_only,
                                     on_progress=self.progress.emit,
                                     **kwargs)
        except Exception as exc:  # surfaced in the UI rather than a traceback
            self.failed.emit(f"{type(exc).__name__}: {exc}")
            return
        self.loaded.emit(data)


class SaveWorker(QObject):
    """Writes one replay's outputs to disk on a worker thread.

    Deliberately re-runs the replay with ``write=True`` rather than dumping the
    curves the viewer already holds: the file names, the trajectories/ and log/
    layout and the source tagging are all decided by the pipeline, and there is
    no second copy of that here to drift from it. What lands on disk is exactly
    what the CLI would have written for this configuration.
    """

    saved = Signal(list)        # paths written
    failed = Signal(str)

    def __init__(self, csv_path, settings, params):
        super().__init__()
        self._csv_path = csv_path
        self._settings = settings
        self._params = params

    def run(self):
        try:
            result = run_replay(self._csv_path, self._settings, self._params, write=True)
        except Exception as exc:  # surfaced in the UI rather than a traceback
            self.failed.emit(f"{type(exc).__name__}: {exc}")
            return
        self.saved.emit(list(result.written))
