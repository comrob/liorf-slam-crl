"""Data sources for the viewer. No Qt here, so this stays testable headless.

One source today: a live replay of a run directory. A reader for
``scale_replay_vectors.csv`` would add a second function returning the same
:class:`FrameData`, adapting to ``FrameGeometry`` and reusing the builder -- not
a second builder. See ``core/local_view.py``.
"""

import os
from dataclasses import dataclass, field

from ..core.local_view import (
    DEFAULT_HISTORY,
    DEFAULT_HISTORY_STEP,
    build_local_frame_views,
    compute_axis_extent,
    geometry_from_replay,
    snap_extent,
)
from ..io.paths import resolve_csv_path
from ..pipeline import run_replay
from ..plotting import complementary_only_curve
from ..settings import DEFAULT_CONFIG_PATH, load_config


@dataclass
class FrameData:
    """One loaded session, ready to scrub."""

    csv_path: str
    views: list
    geometries: list
    #: Half-width covering the whole run, for the "whole run" zoom option.
    axis_extent: float
    provenance: str
    #: The configuration this replay actually ran with, for the editor.
    settings: object = None
    params: object = None
    #: (label, trajectory, kind) triples for the whole-run trajectory plot.
    #: Taken from this replay rather than from the trajectories/ folder, so the
    #: plot always shows the configuration currently applied -- the GUI runs
    #: with write=False, so there may be no files to read.
    curves: list = field(default_factory=list)
    #: Run directory name, for plot titles.
    run_name: str = ""

    @property
    def n_frames(self):
        return len(self.views)


def load_from_run_dir(input_path="", *, latest=False, base_dir="~/.ros/lili_logs",
                      config_path=DEFAULT_CONFIG_PATH, settings=None, params=None,
                      frame="map", history=DEFAULT_HISTORY,
                      history_step=DEFAULT_HISTORY_STEP, observable_only=False,
                      on_progress=None):
    """Replay a run in memory and project it into per-frame views.

    Pass ``settings``/``params`` to replay an edited configuration; otherwise
    they are loaded from ``config_path``.

    The viewer forces ``scale_mode="estimated"``: the per-frame vectors it draws
    are only produced by the estimator path. Whatever the config says about
    scale_mode is therefore overridden, which the caller should surface.
    """
    if settings is None or params is None:
        settings, params = load_config(config_path)
    settings = settings.evolve(scale_mode="estimated")

    csv_path = resolve_csv_path(input_path, latest or not input_path, base_dir)
    result = run_replay(csv_path, settings, params, write=False, on_progress=on_progress)

    _, trajectory = result.replays[0]
    geometries = geometry_from_replay(result.frames, trajectory, result.vector_trace)
    views = build_local_frame_views(geometries, frame=frame, history=history,
                                   history_step=history_step,
                                   observable_only=observable_only)
    # Snapped to a ladder step so the fixed axes land on a round number.
    extent = snap_extent(compute_axis_extent(geometries, frame=frame), margin=1.0)

    source = settings.complementary_source.path
    provenance = (f"{os.path.dirname(csv_path)}  ·  {len(views)} frames  ·  "
                  f"scale mode: estimated (forced by viewer)  ·  "
                  f"correction: {settings.correction_mode}  ·  "
                  f"odom: {os.path.basename(source) if source else 'as recorded'}")

    # The raw complementary curve is recomputed here rather than read back: it
    # is never written to a file, and it is the odometry as replayed -- after
    # any source swap or simulated drift run_replay already applied.
    curves = result.curves() + [complementary_only_curve(result.frames, params)]

    return FrameData(csv_path=csv_path, views=views, geometries=geometries,
                     axis_extent=extent, provenance=provenance,
                     settings=settings, params=params, curves=curves,
                     run_name=os.path.basename(os.path.dirname(csv_path)))


def find_run_dirs(base_dir="~/.ros/lili_logs"):
    """Directories under base_dir that hold a replay CSV, newest first.

    Filters on the CSV rather than on a ``run_*`` name so that ``latest`` and
    any hand-named symlinks show up alongside the raw run directories.
    """
    from ..io.frames_csv import CSV_NAME

    expanded = os.path.expanduser(base_dir)
    if not os.path.isdir(expanded):
        return []
    found = []
    for name in os.listdir(expanded):
        path = os.path.join(expanded, name)
        if os.path.isdir(path) and os.path.isfile(os.path.join(path, CSV_NAME)):
            found.append(path)
    found.sort(key=lambda p: (os.path.basename(p) != "latest", -os.path.getmtime(p)))
    return found
