"""What the viewer remembers about each run it has worked on.

The session (``session.py``) remembers one configuration: the one to open the
*next* run with. This remembers one per run, so that the settings a run needs --
its reference trajectory above all -- stay with it and reach no other. Switching
away and back finds the run as it was left; switching to a different run never
inherits what the last one wanted.

Three things are kept per run, all in the XDG state directory rather than in
the run folder, so nothing is written where a replay's outputs live and a
read-only dataset still works:

* the configuration last applied to it, as a **delta** against what the config
  files say (the base configuration plus the run's own ``replay_scale.yaml``).
  A delta rather than a copy for the same reason a run file is one: a full copy
  would pin every method setting, and a later change to a shared default would
  never reach this run again.
* whether it was ever replayed here, as opposed to previewed and configured.
  Only the replayed ones fill the viewer's "replayed before" list; the others
  are remembered just as quietly, so a reference set while previewing is still
  there next time.
* when that last happened, which is the file's own mtime.

Only the viewer reads any of it. A CLI replay depends on its arguments and the
config files it was given, which is what keeps a replay reproducible from what
is written down rather than from what someone last clicked -- so this layer is
deliberately invisible to it.

Qt-free, like ``sources.py`` and ``session.py``, so it is testable on its own.
"""

import hashlib
import os

from ..settings import (
    DEFAULT_CONFIG_PATH,
    config_from_mapping,
    config_sources_for_run,
    config_to_mapping,
    deep_merge,
    dump_mapping,
    mapping_delta,
    read_mapping,
)
from .session import state_dir

#: Subdirectory of the state directory holding one file per run.
RUNS_DIRNAME = "runs"

#: Top-level key recording that this run was actually replayed here, as opposed
#: to previewed and configured. Kept out of the configuration the file also
#: holds -- ``load_run_state`` takes it off before anything merges the rest --
#: because it is a fact about the viewer's history with the run, not a setting.
REPLAYED_KEY = "replayed"

_HEADER = """\
# What the replay_scale viewer last applied to this run.
#
# Read by the viewer and by nothing else: a CLI replay depends on its arguments
# and the config files it was given, not on what someone last clicked. Merged
# over this run's configuration -- the base config plus the run's own
# replay_scale.yaml -- and holding only what differs from it, so a later change
# to a shared default still reaches this run.
#
# Delete this file to forget the run: the viewer then opens it with what the
# config files say, and drops it from the list of runs already replayed.
"""


def runs_dir():
    """Where the per-run files are kept."""
    return os.path.join(state_dir(), RUNS_DIRNAME)


def run_key(run_dir):
    """The identity of a run: its path with symlinks resolved.

    So that ``latest`` and the directory it points at are one run and not two
    -- following the same link the replay itself follows.
    """
    return os.path.realpath(os.path.expanduser(run_dir))


def state_path_for(run_dir, directory=None):
    """The file remembering this run.

    Named after the run and stamped with a digest of its resolved path: the
    name stays readable in a directory listing, and two runs of the same name
    under different roots still get a file each.
    """
    key = run_key(run_dir)
    digest = hashlib.sha1(key.encode("utf-8")).hexdigest()[:8]
    name = os.path.basename(key.rstrip(os.sep)) or "run"
    safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in name)[:64]
    return os.path.join(directory or runs_dir(), f"{safe}-{digest}.yaml")


def load_run_state(run_dir, directory=None):
    """The delta remembered for this run, or ``{}``.

    Never raises: a corrupt or hand-mangled file costs what it was remembering,
    not the run.
    """
    return _read_state(state_path_for(run_dir, directory))[0]


def _read_state(path):
    """``(delta, replayed)`` from one state file; ``({}, False)`` for anything else."""
    if not os.path.isfile(path):
        return {}, False
    try:
        mapping = read_mapping(path)
    except Exception:
        return {}, False
    if not isinstance(mapping, dict):
        return {}, False
    replayed = bool(mapping.pop(REPLAYED_KEY, False))
    return mapping, replayed


def was_replayed(run_dir, directory=None):
    """Whether the estimator was ever run over this run here."""
    return _read_state(state_path_for(run_dir, directory))[1]


def save_run_state(run_dir, settings, params, inherited=None, directory=None,
                   replayed=False):
    """Remember this configuration as this run's own; returns the path or None.

    ``inherited`` is the configuration the files on disk describe for this run
    (``ConfigSources.inherited``); only what differs from it is written. Without
    one the whole configuration is written, which still restores correctly --
    it just stops inheriting.

    ``replayed`` records that this configuration was actually replayed, not
    only applied to a preview. Never unset by a later save: a run does not stop
    having been replayed because you previewed it again.

    Best-effort, like the session: a state directory that cannot be written to
    costs the convenience and nothing else.
    """
    path = state_path_for(run_dir, directory)
    replayed = bool(replayed) or _read_state(path)[1]
    mapping = config_to_mapping(settings, params)
    if inherited is not None:
        # Normalized through the same writer before comparing, exactly as
        # save_run_config does: a hand-written file leaves defaults implicit,
        # and diffing against it would call every default an override.
        mapping = mapping_delta(mapping, config_to_mapping(*config_from_mapping(inherited)))
    tool = mapping.setdefault("replay_scale_tool", {})
    # Neither of these is a fact about the run. The viewer forces scale_mode
    # (only the estimator path produces the per-frame vectors it draws) and
    # base_dir is where runs are looked for, not what this one is -- pinning
    # either would hand a viewer artifact back to the next replay.
    for artifact in ("scale_mode", "base_dir"):
        tool.pop(artifact, None)
    # The run this file is about. Also what lists it back: the file is named by
    # a digest, which cannot be turned back into a path.
    # Nothing to say about a run that was merely looked at: clicking down a
    # list of runs previews each one, and a file per row saying "no
    # differences, never replayed" would be a state directory full of noise.
    # An existing file is still rewritten -- that is how a setting gets cleared.
    if not (replayed or _has_overrides(mapping) or os.path.isfile(path)):
        return None

    tool["input_path"] = run_key(run_dir)
    if replayed:
        mapping[REPLAYED_KEY] = True

    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(dump_mapping(mapping, header=_HEADER))
    except OSError:
        return None
    return path


def _has_overrides(mapping):
    """Whether a delta says anything beyond which run it is about."""
    return bool([key for key in mapping if key != "replay_scale_tool"]
                or [key for key in mapping.get("replay_scale_tool", {})
                    if key != "input_path"])


def forget_run(run_dir, directory=None):
    """Forget one run. True when a file was actually removed."""
    try:
        os.remove(state_path_for(run_dir, directory))
    except OSError:
        return False
    return True


def forget_all_runs(directory=None):
    """Forget every run. Returns how many were forgotten.

    The way back to a viewer that remembers nothing about anything, for when a
    setting has been carried further than intended and finding which run holds
    it is not worth the trouble.
    """
    directory = directory or runs_dir()
    forgotten = 0
    for run_dir, _when in remembered_runs(directory, existing_only=False,
                                          replayed_only=False):
        forgotten += 1 if forget_run(run_dir, directory) else 0
    return forgotten


def remembered_runs(directory=None, existing_only=True, replayed_only=True):
    """Runs replayed here before, newest first, as ``[(run_dir, when)]``.

    ``when`` is the file's mtime -- when that last happened. Runs whose
    directory has since gone are dropped rather than listed: a row that cannot
    be opened is worse than no row. Their files are left alone, so a dataset
    that is merely unmounted comes back with its settings intact.

    ``replayed_only`` is what makes this the list of runs that went *through*
    the tool, rather than every run it happens to remember something about.
    """
    directory = directory or runs_dir()
    if not os.path.isdir(directory):
        return []
    found = []
    for name in sorted(os.listdir(directory)):
        if not name.endswith((".yaml", ".yml")):
            continue
        path = os.path.join(directory, name)
        mapping, replayed = _read_state(path)
        try:
            run_dir = str(mapping["replay_scale_tool"]["input_path"])
        except (KeyError, TypeError):   # not ours: not a run we know
            continue
        if replayed_only and not replayed:
            continue
        if existing_only and not os.path.isdir(run_dir):
            continue
        found.append((run_dir, os.path.getmtime(path)))
    found.sort(key=lambda entry: -entry[1])
    return found


def config_for_run(csv_path, base_path=DEFAULT_CONFIG_PATH, use_run_state=True,
                   directory=None):
    """``(settings, params, sources)`` for one run, as the viewer sees it.

    The CLI's layering (``settings.load_config_for_run``) with one more layer on
    top: what was last applied to this run here. Everything below it stays where
    it was, so a run whose settings were never touched still follows the base
    configuration wherever it moves.
    """
    sources = config_sources_for_run(csv_path, base_path)
    state = load_run_state(os.path.dirname(os.path.abspath(csv_path)), directory) \
        if use_run_state else {}
    if state:
        sources.state_path = state_path_for(os.path.dirname(os.path.abspath(csv_path)),
                                            directory)
        sources.state_sections = tuple(sorted(state))
        sources.merged = deep_merge(sources.merged, state)
    settings, params = config_from_mapping(sources.merged)
    return settings, params, sources
