#!/usr/bin/env python3
"""2D (top-down XY) plot comparing replay trajectories for one run.

Plots every ``*.tum`` file already written to the run's ``trajectories/``
folder (run replay_scale_trajectory.py first, one or more times with
different replay_scale_tool.scale_mode settings, to populate it), plus the
complementary/wheel odometry integrated on its own (no LiDAR fusion or
estimated-scale correction, scaled only by complementaryOdom.translationScale)
recomputed live from the CSV since it is not written to a file.
"""

import argparse
import glob
import os
import sys

from .replay_io import (
    DEFAULT_CONFIG_PATH,
    apply_complementary_source,
    expand_path,
    load_frames,
    load_replay_params_from_ros_yaml,
    load_replay_tool_settings,
    load_tum,
    resolve_csv_path,
    resolve_output_dirs,
)
from .scale_estimator import reconstruct_complementary_only


def _xy(trajectory):
    xs = [T[0, 3] for _, T in trajectory]
    ys = [T[1, 3] for _, T in trajectory]
    return xs, ys


def _label_from_tum_path(path):
    """Return (label, is_original) derived from a trajectory_*.tum filename."""
    name = os.path.splitext(os.path.basename(path))[0]
    if name.startswith("trajectory_"):
        name = name[len("trajectory_"):]
    is_original = name == "recorded_effective"
    if name.startswith("replay_"):
        name = name[len("replay_"):]
    return name.replace("_", " "), is_original


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", default="",
                        help="Path to scale_replay_frames.csv or a run directory.")
    parser.add_argument("--latest", action="store_true",
                        help="Use the newest run under --base-dir.")
    parser.add_argument("--base-dir", default="~/.ros/lili_logs",
                        help="Base directory holding run_* folders (default: %(default)s).")
    parser.add_argument("--ros-params-yaml", default=DEFAULT_CONFIG_PATH,
                        help="YAML config with complementaryOdom and replay_scale_tool settings "
                             "(default: bundled config/default.yaml).")
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

    config_path = expand_path(args.ros_params_yaml)
    settings = load_replay_tool_settings(config_path)
    params = load_replay_params_from_ros_yaml(config_path)

    csv_path = resolve_csv_path(args.input, args.latest, args.base_dir)
    frames = load_frames(csv_path)
    if not frames:
        print(f"No frames found in {csv_path}", file=sys.stderr)
        return 1

    _, traj_dir, _ = resolve_output_dirs(csv_path, settings)
    tum_paths = sorted(glob.glob(os.path.join(traj_dir, "*.tum")))
    if not tum_paths:
        print(f"No .tum files found in {traj_dir}. "
              "Run replay_scale_trajectory.py first to populate it.", file=sys.stderr)
        return 1

    print(f"Loaded {len(frames)} frames from {csv_path}")
    print(f"  found {len(tum_paths)} trajectory file(s) in {traj_dir}")

    # Affects only the additional-odometry overlay; the .tum curves are already written.
    source_status = apply_complementary_source(frames, settings, csv_path)
    if source_status:
        print(f"  {source_status}")

    fig, ax = plt.subplots(figsize=(9, 9))

    color_cycle = ("tab:orange", "tab:red", "tab:purple", "tab:brown", "tab:pink", "tab:gray", "tab:olive")
    color_idx = 0
    for tum_path in tum_paths:
        label, is_original = _label_from_tum_path(tum_path)
        traj = load_tum(tum_path)
        xs, ys = _xy(traj)
        if is_original:
            ax.plot(xs, ys, label=f"original ({label})", color="tab:blue", linewidth=1.5)
        else:
            color = color_cycle[color_idx % len(color_cycle)]
            color_idx += 1
            ax.plot(xs, ys, label=f"replay ({label})", color=color, linewidth=1.5, linestyle="--")

    additional_odom = reconstruct_complementary_only(
        frames, translation_scale_multiplier=params.translation_scale)
    aox, aoy = _xy(additional_odom)
    ax.plot(aox, aoy, label="additional odometry (complementary, raw)", color="tab:green",
             linewidth=1.0, alpha=0.8)

    os.makedirs(traj_dir, exist_ok=True)
    out_path = expand_path(args.output) if args.output else os.path.join(traj_dir, "trajectories_2d.png")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Replay trajectories — {os.path.basename(os.path.dirname(csv_path))}")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", linewidth=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"  wrote {out_path}")

    if args.show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
