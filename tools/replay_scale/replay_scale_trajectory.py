#!/usr/bin/env python3
"""Offline replay of complementary-odometry scaling and pose correction.

Reads ``scale_replay_frames.csv`` (produced by LiorfDiagnostics when
``log.diagnostics.enable_scale_replay`` is true) and reconstructs map-frame
trajectories without re-running SLAM.

Modes:
- fixed: apply one or more user-provided constant scales.
- recorded: replay with the per-frame scale recorded by the online run.
- estimated: run the C++-style online scale estimator logic over replayed poses,
  using parameters loaded from a ROS YAML config.
"""

import argparse
import glob
import os
import sys

from .replay_io import (
    load_frames,
    load_replay_params_from_ros_yaml,
    load_replay_tool_settings,
    position_drift,
    recorded_effective_trajectory,
    write_scale_trace_csv,
    write_scale_vector_csv,
    write_tum,
)
from .scale_estimator import reconstruct_fixed, reconstruct_with_estimator

CSV_NAME = "scale_replay_frames.csv"
_DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "config", "default.yaml")


# ---------------------------------------------------------------------------
# CLI helpers
# ---------------------------------------------------------------------------

def _expand(path):
    return os.path.expanduser(path)


def _latest_run_dir(base_dir):
    expanded = _expand(base_dir)
    latest = os.path.join(expanded, "latest")
    if os.path.isdir(latest):
        return latest
    candidates = [d for d in glob.glob(os.path.join(expanded, "run_*")) if os.path.isdir(d)]
    if not candidates:
        raise FileNotFoundError(f"No run directories found under: {base_dir}")
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def _resolve_csv(input_path, latest, base_dir):
    if latest or not input_path:
        run_dir = _latest_run_dir(base_dir)
        csv_path = os.path.join(run_dir, CSV_NAME)
    else:
        expanded = _expand(input_path)
        csv_path = os.path.join(expanded, CSV_NAME) if os.path.isdir(expanded) else expanded
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"{CSV_NAME} not found: {csv_path}")
    return csv_path


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
    parser.add_argument("--base-dir", default="~/.ros/liorf_logs",
                        help="Base directory holding run_* folders (default: %(default)s).")
    parser.add_argument("--ros-params-yaml", default=_DEFAULT_CONFIG,
                        help="YAML config with complementaryOdom and replay_scale_tool settings "
                             "(default: bundled config/default.yaml).")
    args = parser.parse_args(argv)

    config_path = _expand(args.ros_params_yaml)
    settings = load_replay_tool_settings(config_path)

    csv_path = _resolve_csv(args.input, args.latest, args.base_dir)
    frames = load_frames(csv_path)
    if not frames:
        print(f"No frames found in {csv_path}", file=sys.stderr)
        return 1

    base_out_dir = _expand(settings.output_dir) if settings.output_dir else os.path.dirname(csv_path)
    out_dir = os.path.join(base_out_dir, settings.output_subdir)
    traj_dir = os.path.join(out_dir, "trajectories")
    log_dir = os.path.join(out_dir, "log")
    os.makedirs(traj_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    n_deg = sum(1 for f in frames if f.degeneracy_detected and f.has_basis)
    print(f"Loaded {len(frames)} frames from {csv_path}")
    print(f"  frames with degeneracy override: {n_deg}")
    print(f"  output directory: {out_dir}")
    print(f"  scale mode: {settings.scale_mode}")

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

    if settings.scale_mode == "fixed":
        for scale in settings.scales:
            traj = reconstruct_fixed(frames, lambda f, s=scale: s, apply_correction=True)
            path = os.path.join(traj_dir, f"trajectory_replay_scale_{scale:g}.tum")
            write_tum(path, traj)
            print(f"  wrote {path}  (scale={scale:g})")

    elif settings.scale_mode == "recorded":
        traj = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True)
        path = os.path.join(traj_dir, "trajectory_replay_recorded_scale.tum")
        write_tum(path, traj)
        print(f"  wrote {path}")

    elif settings.scale_mode == "estimated":
        params = load_replay_params_from_ros_yaml(config_path)
        _print_replay_params(params)

        traj, scale_trace, vector_trace = reconstruct_with_estimator(frames, params, collect_vectors=True)
        traj_path = os.path.join(traj_dir, "trajectory_replay_estimated_scale.tum")
        trace_path = os.path.join(log_dir, "scale_replay_estimator_trace.csv")
        vector_path = os.path.join(log_dir, "scale_replay_vectors.csv")
        write_tum(traj_path, traj)
        write_scale_trace_csv(trace_path, scale_trace)
        write_scale_vector_csv(vector_path, vector_trace)
        print(f"  wrote {traj_path}")
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
