#!/usr/bin/env python3
"""Command-line frontends.

Argument parsing, printing and exit codes -- nothing else. All the work happens
in :mod:`replay_scale.pipeline` and :mod:`replay_scale.plotting`, which any
other frontend calls the same way.
"""

import argparse
import os
import sys
from dataclasses import replace

from .core.model import ESTIMATION_FRAMES
from .io.paths import DEFAULT_BASE_DIR, expand_path, resolve_output_dirs, resolve_run_input
from .io.frames_csv import load_frames
from .pipeline import (
    apply_complementary_source,
    describe_body_frame,
    load_reference_trajectory,
    resolve_active_odometry,
    run_replay,
)
from .plotting import build_trajectory_figure, complementary_only_curve, curves_from_tum_dir
from .settings import (
    DEFAULT_CONFIG_PATH,
    estimation_body_frame,
    load_config,
    load_config_for_run,
    validate_replay_params,
)

REPLAY_DESCRIPTION = """Offline replay of complementary-odometry scaling and pose correction.

Reads ``scale_replay_frames.csv`` (produced by LiliDiagnostics when
``log.diagnostics.enable_scale_replay`` is true) and reconstructs map-frame
trajectories without re-running SLAM.

Modes:
- fixed: apply one or more user-provided constant scales.
- recorded: replay with the per-frame scale recorded by the online run.
- estimated: run the C++-style online scale estimator logic over replayed poses,
  using parameters loaded from a ROS YAML config.
"""

PLOT_DESCRIPTION = """2D (top-down XY) plot comparing replay trajectories for one run.

Plots every ``*.tum`` file already written to the run's ``trajectories/``
folder (run replay-scale-trajectory first, one or more times with
different replay_scale_tool.scale_mode settings, to populate it), plus the
complementary/wheel odometry integrated on its own (no LiDAR fusion or
estimated-scale correction, scaled only by complementaryOdom.translationScale)
recomputed live from the CSV since it is not written to a file.
"""


def _add_common_arguments(parser):
    parser.add_argument("input", nargs="?", default="",
                        help="Path to scale_replay_frames.csv or a run directory. Omit to "
                             "use replay_scale_tool.input_path, and the newest run under "
                             "the base directory if that is empty too.")
    parser.add_argument("--latest", action="store_true",
                        help="Use the newest run under the base directory, ignoring any "
                             "configured input_path.")
    parser.add_argument("--base-dir", default="",
                        help="Base directory holding run_* folders. Overrides "
                             f"replay_scale_tool.base_dir; default {DEFAULT_BASE_DIR}.")
    parser.add_argument("--ros-params-yaml", default=DEFAULT_CONFIG_PATH,
                        help="YAML config with complementaryOdom and replay_scale_tool settings "
                             "(default: bundled config/default.yaml).")
    parser.add_argument("--no-run-config", action="store_true",
                        help="Ignore the run's own replay_scale.yaml, replaying it with "
                             "the base configuration alone. For sweeping one "
                             "configuration across runs that each carry their own.")
    parser.add_argument("--estimation-frame", default="", choices=("",) + ESTIMATION_FRAMES,
                        help="Body frame the correction is measured and applied in, "
                             "overriding complementaryOdom.estimationFrame. "
                             "'complementary' places it at the odometry sensor's own "
                             "origin, so a body rotation about that origin is not read "
                             "as odometry error; 'lidar' is the node's own behaviour.")
    return parser


def _with_estimation_frame(params, args):
    """Apply the --estimation-frame override, if the user gave one."""
    if not args.estimation_frame:
        return params
    return validate_replay_params(
        replace(params, estimation_frame=args.estimation_frame))


def main(argv=None):
    parser = argparse.ArgumentParser(description=REPLAY_DESCRIPTION,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    _add_common_arguments(parser)
    args = parser.parse_args(argv)

    # Two stages, because the run has to be known before its own config can be
    # read: the base config says which run, the run's file then refines how.
    base_path = expand_path(args.ros_params_yaml)
    settings, _ = load_config(base_path)
    csv_path = resolve_run_input(settings, args.input, args.latest, args.base_dir)
    settings, params, sources = load_config_for_run(
        csv_path, base_path, use_run_config=not args.no_run_config)
    print(sources.describe())
    params = _with_estimation_frame(params, args)

    try:
        run_replay(csv_path, settings, params, write=True, on_progress=print)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


def main_plot(argv=None):
    parser = argparse.ArgumentParser(description=PLOT_DESCRIPTION,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    _add_common_arguments(parser)
    parser.add_argument("--output", default="",
                        help="Path to save the plot PNG "
                             "(default: <replay output dir>/trajectories/trajectories_2d.png).")
    parser.add_argument("--show", action="store_true",
                        help="Also open an interactive window.")
    args = parser.parse_args(argv)

    try:
        import matplotlib
        if not args.show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is required for plotting. Install it with: pip install matplotlib",
              file=sys.stderr)
        return 1

    base_path = expand_path(args.ros_params_yaml)
    settings, _ = load_config(base_path)
    csv_path = resolve_run_input(settings, args.input, args.latest, args.base_dir)
    settings, params, sources = load_config_for_run(
        csv_path, base_path, use_run_config=not args.no_run_config)
    print(sources.describe())
    params = _with_estimation_frame(params, args)

    frames = load_frames(csv_path)
    if not frames:
        print(f"No frames found in {csv_path}", file=sys.stderr)
        return 1

    _, traj_dir, _ = resolve_output_dirs(csv_path, settings)
    try:
        curves = curves_from_tum_dir(traj_dir)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print(f"Loaded {len(frames)} frames from {csv_path}")
    print(f"  found {len(curves)} trajectory file(s) in {traj_dir}")

    active = resolve_active_odometry(settings, csv_path)
    body_frame = estimation_body_frame(active.extrinsic, params)
    if active.note:
        print(f"  note: {active.note}")
    print(f"  {describe_body_frame(body_frame, params, active)}")

    # Affects only the additional-odometry overlay; the .tum curves are already written.
    source_status = apply_complementary_source(frames, settings, csv_path, active)
    if source_status:
        print(f"  {source_status}")

    # The trajectories/ folder is what this command draws, so a fitted
    # alignment is turned onto the first replay curve found there.
    replayed = next((traj for _, traj, kind in curves if kind == "replay"), None)
    reference, reference_status = load_reference_trajectory(settings, frames, replayed)
    if reference_status:
        print(f"  {reference_status}")
    if reference is not None:
        curves.insert(0, (reference[0], reference[1], "external"))

    curves.append(complementary_only_curve(frames, params, body_frame))
    fig = build_trajectory_figure(
        curves, title=f"Replay trajectories — {os.path.basename(os.path.dirname(csv_path))}")

    os.makedirs(traj_dir, exist_ok=True)
    out_path = expand_path(args.output) if args.output else os.path.join(traj_dir, "trajectories_2d.png")
    fig.savefig(out_path, dpi=150)
    print(f"  wrote {out_path}")

    if args.show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
