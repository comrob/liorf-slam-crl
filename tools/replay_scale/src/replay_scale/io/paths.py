"""Run-directory discovery and output path layout."""

import glob
import os

from .frames_csv import CSV_NAME

#: Where run folders live when neither the config nor the command line says.
DEFAULT_BASE_DIR = "~/.ros/lili_logs"


def expand_path(path):
    return os.path.expanduser(path)


def latest_run_dir(base_dir):
    expanded = expand_path(base_dir)
    latest = os.path.join(expanded, "latest")
    if os.path.isdir(latest):
        return latest
    candidates = [d for d in glob.glob(os.path.join(expanded, "run_*")) if os.path.isdir(d)]
    if not candidates:
        raise FileNotFoundError(f"No run directories found under: {base_dir}")
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def resolve_csv_path(input_path, latest, base_dir):
    if latest or not input_path:
        run_dir = latest_run_dir(base_dir)
        csv_path = os.path.join(run_dir, CSV_NAME)
    else:
        expanded = expand_path(input_path)
        csv_path = os.path.join(expanded, CSV_NAME) if os.path.isdir(expanded) else expanded
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"{CSV_NAME} not found: {csv_path}")
    return csv_path


def resolve_run_input(settings, input_path="", latest=False, base_dir=""):
    """Which run to replay, given a config and whatever the caller overrode.

    Precedence is the same for both frontends: an explicit argument wins, then
    the config's ``input_path`` / ``base_dir``, then the built-in default. An
    empty result -- or ``latest`` -- means the newest run under the base
    directory, which is what ``latest_run_dir`` resolves.

    Takes the settings object rather than importing it, so ``io`` keeps its one
    allowed dependency.
    """
    chosen = input_path or getattr(settings, "input_path", "")
    base = base_dir or getattr(settings, "base_dir", "") or DEFAULT_BASE_DIR

    # A bare run name is the obvious thing to write in a config, and it is
    # meaningless relative to whatever directory the tool happens to be started
    # from -- so resolve it against the base directory first.
    if chosen and not os.path.isabs(expand_path(chosen)):
        in_base = os.path.join(expand_path(base), expand_path(chosen))
        if os.path.exists(in_base):
            chosen = in_base

    return resolve_csv_path(chosen, latest or not chosen, base)


def resolve_output_dirs(csv_path, settings):
    """Return (out_dir, trajectories_dir, log_dir) for the given settings."""
    base_out_dir = expand_path(settings.output_dir) if settings.output_dir else os.path.dirname(csv_path)
    out_dir = os.path.join(base_out_dir, settings.output_subdir)
    traj_dir = os.path.join(out_dir, "trajectories")
    log_dir = os.path.join(out_dir, "log")
    return out_dir, traj_dir, log_dir


def complementary_source_tag(settings):
    """Filename suffix identifying the odometry source, empty for the recorded one.

    Keeps replays of different sources side by side in trajectories/ instead of
    overwriting each other, so they can be plotted against one another.
    """
    tag = ""
    path = settings.complementary_source.path
    if path:
        tag += f"_src_{os.path.splitext(os.path.basename(expand_path(path)))[0]}"
        # Only when it differs from the node's own matching, so the file names of
        # a plain source swap are unchanged -- but the two are comparable.
        if settings.complementary_source.match_mode != "nearest":
            tag += f"_{settings.complementary_source.match_mode}"
    if settings.correction_mode != "twist6":
        tag += f"_corr_{settings.correction_mode}"
    drift = getattr(settings, "complementary_drift", None)
    if drift is not None and drift.alpha:
        # Sweeping alpha is the point, so each value needs its own file.
        tag += f"_drift_{drift.axis}{drift.alpha:+g}".replace("+", "p").replace("-", "m")
    return tag
