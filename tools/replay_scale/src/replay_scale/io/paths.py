"""Run-directory discovery and output path layout."""

import glob
import os

from .frames_csv import CSV_NAME


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
    if settings.correction_mode != "twist6":
        tag += f"_corr_{settings.correction_mode}"
    drift = getattr(settings, "complementary_drift", None)
    if drift is not None and drift.alpha:
        # Sweeping alpha is the point, so each value needs its own file.
        tag += f"_drift_{drift.axis}{drift.alpha:+g}".replace("+", "p").replace("-", "m")
    return tag
