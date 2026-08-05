#!/usr/bin/env python3
"""Offline replay of complementary-odometry scaling and pose correction.

Reads ``scale_replay_frames.csv`` (produced by LiliDiagnostics when
``log.diagnostics.enable_scale_replay`` is true) and reconstructs map-frame
trajectories without re-running SLAM.

Modes:
- fixed: apply one or more user-provided constant scales.
- recorded: replay with the per-frame scale recorded by the online run.
- estimated: run the C++-style online scale estimator logic over replayed poses,
  using parameters loaded from a ROS YAML config.
"""

import argparse
import os
import sys

from .replay_io import (
    DEFAULT_CONFIG_PATH,
    apply_complementary_source,
    complementary_source_tag,
    expand_path,
    load_frames,
    load_replay_params_from_ros_yaml,
    load_replay_tool_settings,
    position_drift,
    reconstruct_replay_trajectories,
    recorded_effective_trajectory,
    resolve_csv_path,
    resolve_output_dirs,
    write_scale_trace_csv,
    write_scale_vector_csv,
    write_tum,
)
from .scale_estimator import reconstruct_fixed


def _print_replay_params(params):
    print("  estimator params:")
    print(f"    translationScale: {params.translation_scale}")
    print(f"    scaleEstimationApply: {params.scale_estimation_apply}")
    print(f"    scaleMinNonDegenerateSpeed: {params.scale_min_nondegenerate_speed}")
    print(f"    scaleBaselineFrameLag: {params.scale_baseline_frame_lag}")
    print(f"    scaleSmoothingWindowSize: {params.scale_smoothing_window_size}")
    print(f"    ignore_dz: {params.ignore_dz}")


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
    args = parser.parse_args(argv)

    config_path = expand_path(args.ros_params_yaml)
    settings = load_replay_tool_settings(config_path)

    csv_path = resolve_csv_path(args.input, args.latest, args.base_dir)
    frames = load_frames(csv_path)
    if not frames:
        print(f"No frames found in {csv_path}", file=sys.stderr)
        return 1

    out_dir, traj_dir, log_dir = resolve_output_dirs(csv_path, settings)
    os.makedirs(traj_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    n_deg = sum(1 for f in frames if f.degeneracy_detected and f.has_basis)
    print(f"Loaded {len(frames)} frames from {csv_path}")
    print(f"  frames with degeneracy override: {n_deg}")
    print(f"  output directory: {out_dir}")
    print(f"  scale mode: {settings.scale_mode}")

    source_status = apply_complementary_source(frames, settings, csv_path)
    if source_status:
        print(f"  {source_status}")

    recorded = recorded_effective_trajectory(frames)
    rec_path = os.path.join(traj_dir, "trajectory_recorded_effective.tum")
    write_tum(rec_path, recorded)
    print(f"  wrote {rec_path}")

    if settings.validate:
        traj_val = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True)
        drift = position_drift(traj_val, recorded)
        val_path = os.path.join(traj_dir, "trajectory_replay_validate.tum")
        write_tum(val_path, traj_val)
        print(f"  wrote {val_path}")
        if drift.size:
            print("  validation drift vs recorded effective: "
                  f"mean={drift.mean():.6f} m  max={drift.max():.6f} m  "
                  f"rms={(float((drift ** 2).mean()) ** 0.5):.6f} m")

    if settings.scale_mode == "estimated":
        params = load_replay_params_from_ros_yaml(config_path)
        _print_replay_params(params)
    else:
        params = None

    source_tag = complementary_source_tag(settings)
    replay_results, scale_trace, vector_trace = reconstruct_replay_trajectories(
        frames, settings, params, collect_traces=True)
    for tag, traj in replay_results:
        path = os.path.join(traj_dir, f"trajectory_replay_{tag}{source_tag}.tum")
        write_tum(path, traj)
        print(f"  wrote {path}")

    if settings.scale_mode == "estimated":
        trace_path = os.path.join(log_dir, f"scale_replay_estimator_trace{source_tag}.csv")
        vector_path = os.path.join(log_dir, f"scale_replay_vectors{source_tag}.csv")
        write_scale_trace_csv(trace_path, scale_trace)
        write_scale_vector_csv(vector_path, vector_trace)
        print(f"  wrote {trace_path}")
        print(f"  wrote {vector_path}")

    if settings.no_correction:
        traj = reconstruct_fixed(frames, lambda f: 1.0, apply_correction=False)
        path = os.path.join(traj_dir, "trajectory_replay_lidar_only.tum")
        write_tum(path, traj)
        print(f"  wrote {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
