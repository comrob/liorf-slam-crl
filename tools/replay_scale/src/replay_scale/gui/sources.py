"""Data sources for the viewer. No Qt here, so this stays testable headless.

Two ways to open a run, both returning the same :class:`FrameData`:

``load_from_run_dir``
    replays it in memory -- the estimator over every frame, then the per-frame
    views. Seconds on a long run, which is why the viewer caches the result.
``preview_run_dir``
    reads it. The recorded trajectory, the odometry integrated on its own, any
    trajectories an earlier replay wrote beside it, and the reference drawn
    against them -- a fifth of a second, and enough to see what a run is and
    whether a reference lines up with it. No estimator runs, so there are no
    per-frame vectors and no scale history: those tabs stay empty until the run
    is actually replayed.

A reader for ``scale_replay_vectors.csv`` would be a third, adapting to
``FrameGeometry`` and reusing the builder -- not a second builder. See
``core/local_view.py``.
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
from ..io.frames_csv import CSV_NAME, load_frames
from ..io.paths import resolve_csv_path, resolve_output_dirs
from ..io.tum import load_tum
from ..pipeline import (
    load_reference_trajectory,
    recorded_effective_trajectory,
    resolve_active_odometry,
    run_replay,
)
from ..plotting import complementary_only_curve
from ..settings import (
    DEFAULT_CONFIG_PATH,
    estimation_body_frame,
    load_complementary_odom_meta,
)
from .run_state import config_for_run, run_key


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
    #: Which odometry this replay corrected and the mount that placed it, and
    #: the mount the run itself recorded. The panel shows both: only the
    #: pipeline reads the run's meta file, so the editor cannot work either out.
    active_odometry: object = None
    run_mount: object = None
    #: Where this run's configuration came from, and the base a per-run file is
    #: written as a delta against; see settings.ConfigSources.
    config_sources: object = None
    #: The replayed frames and the trajectory the fitted reference alignments
    #: are turned onto -- what run_replay handed load_reference_trajectory.
    #: Kept so that changing the reference can be redrawn rather than replayed;
    #: see reload_reference. Both are already-built objects, not copies.
    frames: list = None
    fit_target: list = None
    #: The view options ``views`` were built with, so a run coming back from the
    #: cache knows whether they still match the controls; see view_options.
    view_options: tuple = ()
    #: True for a run that was read but not replayed -- see preview_run_dir.
    #: The trajectory tab has something to show; the per-frame tabs do not.
    preview: bool = False

    @property
    def n_frames(self):
        return len(self.views)


def view_options(frame, history, history_step, observable_only):
    """The controls the per-frame views are built for, as one comparable value.

    A run coming back from the viewer's cache was projected for whatever these
    were at the time; comparing them is how it knows whether to reuse the views
    or rebuild them.
    """
    return (frame, history, history_step, bool(observable_only))


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
    csv_path = resolve_csv_path(input_path, latest or not input_path, base_dir)

    # Resolved either way: even when the caller hands over edited settings, the
    # files below are what a per-run file and the viewer's own layer get
    # written as deltas against.
    resolved, resolved_params, sources = config_for_run(csv_path, config_path)
    if settings is None or params is None:
        settings, params = resolved, resolved_params
    settings = settings.evolve(scale_mode="estimated")
    result = run_replay(csv_path, settings, params, write=False, on_progress=on_progress)

    _, trajectory = result.replays[0]
    geometries = geometry_from_replay(result.frames, trajectory, result.vector_trace,
                                      result.body_frame)
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
    curves = result.curves() + [
        complementary_only_curve(result.frames, params, result.body_frame)]

    return FrameData(frames=result.frames, fit_target=trajectory,
                     view_options=view_options(frame, history, history_step,
                                               observable_only),
                     csv_path=csv_path, views=views, geometries=geometries,
                     axis_extent=extent, provenance=provenance,
                     settings=settings, params=params, curves=curves,
                     run_name=os.path.basename(os.path.dirname(csv_path)),
                     active_odometry=result.active_odometry,
                     run_mount=load_complementary_odom_meta(csv_path),
                     config_sources=sources)


def _written_trajectories(csv_path, settings):
    """(label, trajectory, kind) for what an earlier replay left beside this run.

    The preview's only view of what the estimator does with this run: the files
    the CLI or "Save trajectories…" wrote into ``replay/trajectories/``. The
    recorded effective one is skipped -- it is drawn from the CSV itself, which
    is always there and cannot be out of date.
    """
    _, traj_dir, _ = resolve_output_dirs(csv_path, settings)
    if not os.path.isdir(traj_dir):
        return []
    out = []
    for name in sorted(os.listdir(traj_dir)):
        if not name.endswith(".tum") or name == "trajectory_recorded_effective.tum":
            continue
        label = os.path.splitext(name)[0].replace("trajectory_", "").replace("_", " ")
        try:
            trajectory = load_tum(os.path.join(traj_dir, name))
        except (OSError, ValueError):   # a truncated file must not stop a preview
            continue
        if trajectory:
            out.append((f"{label} (written earlier)", trajectory, "replay"))
    return out


def preview_run_dir(input_path="", *, latest=False, base_dir="~/.ros/lili_logs",
                    config_path=DEFAULT_CONFIG_PATH, settings=None, params=None,
                    on_progress=None):
    """Read a run without replaying it; see the module docstring.

    Everything here is a read or a single integration over the frames, so it
    costs about what parsing the CSV costs. What it cannot show is anything the
    estimator produces: per-frame vectors, the scale history, the corrected
    trajectory. It shows what the run *recorded*, what its odometry does on its
    own, whatever an earlier replay wrote beside it, and the reference
    trajectory placed against them -- which is what "is this the run I mean, and
    does my reference line up with it" needs.
    """
    emit = on_progress if on_progress is not None else (lambda _msg: None)
    csv_path = resolve_csv_path(input_path, latest or not input_path, base_dir)

    resolved, resolved_params, sources = config_for_run(csv_path, config_path)
    if settings is None or params is None:
        settings, params = resolved, resolved_params
    settings = settings.evolve(scale_mode="estimated")

    emit(f"Reading {csv_path} …")
    frames = load_frames(csv_path)
    if not frames:
        raise ValueError(f"No frames found in {csv_path}")

    active = resolve_active_odometry(settings, csv_path)
    body_frame = estimation_body_frame(active.extrinsic, params)
    recorded = recorded_effective_trajectory(frames)

    curves = [("recorded effective", recorded, "reference")]
    curves += _written_trajectories(csv_path, settings)
    curves.append(complementary_only_curve(frames, params, body_frame))

    # Fitted onto a replay when there is one on disk, so the placement matches
    # what a replayed view would show; onto the recorded trajectory otherwise,
    # which is the only thing here the reference can be compared with.
    fit_target = next((traj for _, traj, kind in curves if kind == "replay"), recorded)
    reference, reference_status = load_reference_trajectory(settings, frames, fit_target)
    if reference is not None:
        curves.insert(0, (reference[0], reference[1], "external"))
    if reference_status:
        emit(f"  {reference_status}")

    provenance = (f"{os.path.dirname(csv_path)}  ·  {len(frames)} frames  ·  "
                  f"preview: read, not replayed — \"Apply & replay\" runs the "
                  f"estimator  ·  correction: {settings.correction_mode}")

    return FrameData(csv_path=csv_path, views=[], geometries=[], axis_extent=1.0,
                     provenance=provenance, settings=settings, params=params,
                     curves=curves, run_name=os.path.basename(os.path.dirname(csv_path)),
                     active_odometry=active,
                     run_mount=load_complementary_odom_meta(csv_path),
                     config_sources=sources, frames=frames, fit_target=fit_target,
                     preview=True)


def reload_reference(data, settings):
    """``(curves, status)`` for this replay drawn against another reference.

    A reference trajectory is drawn, never measured against -- no estimate can
    depend on it -- so changing one must not cost a replay. The alignment is
    refitted here against the trajectory already loaded, which is the same one
    :func:`~replay_scale.pipeline.run_replay` would hand
    :func:`~replay_scale.pipeline.load_reference_trajectory`, so what is drawn
    is what a re-run would have drawn.

    Raises whatever that loader raises for a file it cannot use; the caller
    keeps the curves it has.
    """
    reference, status = load_reference_trajectory(settings, data.frames, data.fit_target)
    # The old one out, the new one first -- so it reads as the thing the rest is
    # being judged against rather than as one more replay, as run_replay orders it.
    curves = [curve for curve in data.curves if curve[2] != "external"]
    if reference is not None:
        curves.insert(0, (reference[0], reference[1], "external"))
    return curves, status


@dataclass
class RunEntry:
    """One run offered in the viewer's list, under one name."""

    path: str
    #: A symlink under the base directory: a run someone stopped to name --
    #: ``latest``, or the label you gave a recording worth coming back to. The
    #: raw ``run_<timestamp>`` directories are everything the node ever wrote.
    is_link: bool = False
    mtime: float = 0.0
    #: The run's other names under the same base directory, if it has any.
    aliases: tuple = ()

    @property
    def name(self):
        return os.path.basename(os.path.normpath(self.path))

    @property
    def target(self):
        """What a link points at, as text for a tooltip; empty for a directory."""
        return os.path.realpath(self.path) if self.is_link else ""

    def describe(self):
        """Path, what it resolves to, and the run's other names."""
        lines = [self.path]
        if self.is_link:
            lines.append(f"→ {self.target}")
        if self.aliases:
            lines.append(f"also here as: {', '.join(self.aliases)}")
        return "\n".join(lines)


def _preferred(group):
    """Which of a run's names to offer it under.

    A name someone chose beats ``latest``, which moves, and both beat the
    timestamped directory, which is what the node called it. ``group`` is in
    list order, so "the first of the best kind" is also the newest of them.
    """
    links = [entry for entry in group if entry.is_link]
    named = [entry for entry in links if entry.name != "latest"]
    return (named or links or group)[0]


def list_runs(base_dir="~/.ros/lili_logs"):
    """Runs under ``base_dir``: the named ones first, then the rest.

    Filters on the CSV rather than on a ``run_*`` name so that ``latest`` and
    any hand-named symlinks show up alongside the raw run directories -- and
    then puts those links first, newest first within each group. A link is a
    run someone thought worth naming, so it is the one worth finding again;
    the timestamped directories behind them are the pile they were picked from.

    **One row per run.** The same recording reached through ``latest``, through
    the name you gave it and through its own directory is one run, and three
    rows for it is what makes a list impossible to pick from: you cannot tell
    which of them you are looking at, and marking the one on screen marks a row
    you did not click. The other names travel with the entry instead.
    """
    expanded = os.path.expanduser(base_dir)
    if not os.path.isdir(expanded):
        return []
    found = []
    for name in sorted(os.listdir(expanded)):
        path = os.path.join(expanded, name)
        if os.path.isdir(path) and os.path.isfile(os.path.join(path, CSV_NAME)):
            found.append(RunEntry(path=path, is_link=os.path.islink(path),
                                  mtime=os.path.getmtime(path)))
    found.sort(key=lambda entry: (not entry.is_link, -entry.mtime))

    grouped = {}
    for entry in found:
        grouped.setdefault(run_key(entry.path), []).append(entry)
    offered = []
    for group in grouped.values():
        entry = _preferred(group)
        entry.aliases = tuple(other.name for other in group if other is not entry)
        offered.append(entry)
    offered.sort(key=lambda entry: (not entry.is_link, -entry.mtime))
    return offered


def find_run_dirs(base_dir="~/.ros/lili_logs"):
    """The paths :func:`list_runs` offers, in the same order."""
    return [entry.path for entry in list_runs(base_dir)]
