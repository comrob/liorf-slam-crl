#!/usr/bin/env python3

import argparse
import glob
import math
import os
import re
import sys
from typing import Dict, List, Optional, Tuple


def _expand(path: str) -> str:
    return os.path.expanduser(path)


def _latest_run_dir(base_dir: str) -> str:
    expanded_base = _expand(base_dir)
    latest_link = os.path.join(expanded_base, "latest")
    if os.path.isdir(latest_link):
        return latest_link

    pattern = os.path.join(expanded_base, "run_*")
    candidates = [d for d in glob.glob(pattern) if os.path.isdir(d)]
    if not candidates:
        raise FileNotFoundError(f"No run directories found under: {base_dir}")
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def _resolve_event_path(input_path: str, latest: bool, base_dir: str) -> str:
    if latest or not input_path:
        run_dir = _latest_run_dir(base_dir)
        event_path = os.path.join(run_dir, "event.txt")
        if not os.path.isfile(event_path):
            raise FileNotFoundError(f"event.txt not found in latest run directory: {run_dir}")
        return event_path

    expanded = _expand(input_path)
    if os.path.isdir(expanded):
        event_path = os.path.join(expanded, "event.txt")
    else:
        event_path = expanded

    if not os.path.isfile(event_path):
        raise FileNotFoundError(f"Event file not found: {event_path}")
    return event_path


def _to_float(value: str) -> float:
    try:
        if value.lower() in {"nan", "+nan", "-nan"}:
            return float("nan")
        if value.lower() in {"inf", "+inf", "infinity", "+infinity"}:
            return float("inf")
        if value.lower() in {"-inf", "-infinity"}:
            return float("-inf")
        return float(value)
    except Exception:
        return float("nan")


def _parse_kv(message: str) -> Dict[str, str]:
    # Parse tokens like key=value where value has no spaces.
    # This is sufficient for COMPLEMENTARY_ODOM_SCALE and scalar COMPLEMENTARY_ODOM_TWIST fields.
    kv = {}
    for k, v in re.findall(r"([A-Za-z0-9_]+)=([^\s]+)", message):
        kv[k] = v.rstrip(",")
    return kv


def _load_event_metrics(event_path: str):
    scale_rows = []
    twist_rows = []

    with open(event_path, "r", encoding="utf-8") as f:
        for line_no, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line:
                continue

            # Expected format: "<stamp_sec> <message>"
            parts = line.split(" ", 1)
            if len(parts) != 2:
                continue

            try:
                stamp = float(parts[0])
            except ValueError:
                continue

            message = parts[1]
            kv = _parse_kv(message)

            if "[COMPLEMENTARY_ODOM_SCALE]" in message:
                scale_rows.append(
                    {
                        "stamp": stamp,
                        "frame_stamp_s": _to_float(kv.get("frame_stamp_s", "nan")),
                        "enabled": _to_float(kv.get("enabled", "nan")),
                        "gate_passed": _to_float(kv.get("gate_passed", "nan")),
                        "scale_applied": _to_float(kv.get("scale_applied", "nan")),
                        "scale_ratio_raw": _to_float(kv.get("scale_ratio_raw", "nan")),
                        "scale_ls_raw": _to_float(kv.get("scale_ls_raw", "nan")),
                        "theta_deg": _to_float(kv.get("theta_deg", "nan")),
                        "lidar_nondeg_m": _to_float(kv.get("lidar_nondeg_m", "nan")),
                        "complementary_nondeg_m": _to_float(
                            kv.get("complementary_nondeg_m", kv.get("add_nondeg_m", "nan"))
                        ),
                        "min_nondeg_m": _to_float(kv.get("min_nondeg_m", "nan")),
                        "complementary_odom_lin_speed_orig_mps": _to_float(kv.get("complementary_odom_lin_speed_orig_mps", "nan")),
                        "lidar_lin_speed_nondeg_mps": _to_float(kv.get("lidar_lin_speed_nondeg_mps", "nan")),
                        "complementary_odom_lin_speed_nondeg_mps": _to_float(kv.get("complementary_odom_lin_speed_nondeg_mps", "nan")),
                        "lidar_lin_speed_proj_scale1_mps": _to_float(kv.get("lidar_lin_speed_proj_scale1_mps", "nan")),
                        "lidar_lin_speed_after_scale_mps": _to_float(kv.get("lidar_lin_speed_after_scale_mps", "nan")),
                        # Backward compatibility with earlier key name.
                        "lidar_lin_speed_reproject_mps": _to_float(kv.get("lidar_lin_speed_reproject_mps", "nan")),
                        "dt_scan_s": _to_float(kv.get("dt_scan_s", "nan")),
                    }
                )

            elif "[COMPLEMENTARY_ODOM_TWIST]" in message:
                twist_rows.append(
                    {
                        "stamp": stamp,
                        "dt_add_s": _to_float(kv.get("dt_add_s", "nan")),
                        "lin_norm": _to_float(kv.get("lin_norm", "nan")),
                        "ang_norm": _to_float(kv.get("ang_norm", "nan")),
                        "prev_match_abs_dt_s": _to_float(kv.get("prev_match_abs_dt_s", "nan")),
                        "curr_match_abs_dt_s": _to_float(kv.get("curr_match_abs_dt_s", "nan")),
                    }
                )

    if not scale_rows:
        raise ValueError(
            "No [COMPLEMENTARY_ODOM_SCALE] events found. Ensure diagnostics_write_event=true and run includes degeneracy frames."
        )

    scale_rows.sort(key=lambda r: r["stamp"])
    twist_rows.sort(key=lambda r: r["stamp"])

    return scale_rows, twist_rows


def _rel_time(values: List[float]) -> List[float]:
    if not values:
        return values
    t0 = values[0]
    return [t - t0 for t in values]


def _nan_safe_div(a: float, b: float) -> float:
    if not math.isfinite(a) or not math.isfinite(b) or abs(b) < 1e-9:
        return float("nan")
    return a / b


def _print_summary(scale_rows, twist_rows) -> None:
    n = len(scale_rows)
    n_gate = sum(1 for r in scale_rows if r["gate_passed"] == 1.0)
    n_enabled = sum(1 for r in scale_rows if r["enabled"] == 1.0)

    def _mean_finite(key: str) -> float:
        vals = [r[key] for r in scale_rows if math.isfinite(r[key])]
        if not vals:
            return float("nan")
        return sum(vals) / len(vals)

    print("COMPLEMENTARY_ODOM_SCALE summary")
    print(f"- samples: {n}")
    print(f"- gate passed: {n_gate}/{n} ({(100.0 * n_gate / n) if n else 0.0:.1f}%)")
    print(f"- scale enabled: {n_enabled}/{n} ({(100.0 * n_enabled / n) if n else 0.0:.1f}%)")
    print(f"- mean applied scale: {_mean_finite('scale_applied'):.4f}")
    print(f"- mean raw ratio scale: {_mean_finite('scale_ratio_raw'):.4f}")
    print(f"- mean raw LS scale: {_mean_finite('scale_ls_raw'):.4f}")
    print(f"- mean theta_deg: {_mean_finite('theta_deg'):.3f}")

    if twist_rows:
        lin_vals = [r["lin_norm"] for r in twist_rows if math.isfinite(r["lin_norm"])]
        ang_vals = [r["ang_norm"] for r in twist_rows if math.isfinite(r["ang_norm"])]
        if lin_vals:
            print(f"- mean complementary-odom lin_norm (from [COMPLEMENTARY_ODOM_TWIST]): {sum(lin_vals) / len(lin_vals):.4f}")
        if ang_vals:
            print(f"- mean complementary-odom ang_norm (from [COMPLEMENTARY_ODOM_TWIST]): {sum(ang_vals) / len(ang_vals):.4f}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Visualize complementary-odometry scale diagnostics from LIORF event.txt"
    )
    parser.add_argument(
        "--input",
        default="",
        help="Path to event.txt or run directory containing event.txt",
    )
    parser.add_argument(
        "--latest",
        action="store_true",
        help="Use latest run under --base-dir",
    )
    parser.add_argument(
        "--base-dir",
        default="~/.ros/liorf_logs",
        help="Base diagnostics log directory (default: ~/.ros/liorf_logs)",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output image path (default: <run_dir>/complementary_odom_scale_diagnostics.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show interactive plot window",
    )
    args = parser.parse_args()

    try:
        event_path = _resolve_event_path(args.input, args.latest, args.base_dir)
        run_dir = os.path.dirname(event_path)

        # Delayed import so users can still inspect parser errors without matplotlib.
        import matplotlib.pyplot as plt

        scale_rows, twist_rows = _load_event_metrics(event_path)
        _print_summary(scale_rows, twist_rows)

        t_scale = _rel_time([r["stamp"] for r in scale_rows])
        scale_applied = [r["scale_applied"] for r in scale_rows]
        scale_ratio_raw = [r["scale_ratio_raw"] for r in scale_rows]
        scale_ls_raw = [r["scale_ls_raw"] for r in scale_rows]
        theta_deg = [r["theta_deg"] for r in scale_rows]
        gate_passed = [r["gate_passed"] for r in scale_rows]
        enabled = [r["enabled"] for r in scale_rows]
        lidar_nondeg_m = [r["lidar_nondeg_m"] for r in scale_rows]
        add_nondeg_m = [r["add_nondeg_m"] for r in scale_rows]
        min_nondeg_m = [r["min_nondeg_m"] for r in scale_rows]
        dt_scan_s = [r["dt_scan_s"] for r in scale_rows]

        # Prefer logged speed fields; fall back to reconstructed values for
        # older logs where these fields may be absent.
        lidar_nondeg_speed = []
        add_nondeg_speed = []
        complementary_odom_speed_orig = []
        lidar_speed_proj_scale1 = []
        lidar_speed_after_scale = []
        for r in scale_rows:
            lidar_logged = r["lidar_lin_speed_nondeg_mps"]
            if not math.isfinite(lidar_logged):
                lidar_logged = r["lidar_lin_speed_reproject_mps"]
            if not math.isfinite(lidar_logged):
                lidar_logged = _nan_safe_div(r["lidar_nondeg_m"], r["dt_scan_s"])
            lidar_nondeg_speed.append(lidar_logged)

            complementary_logged = r["complementary_odom_lin_speed_nondeg_mps"]
            if not math.isfinite(complementary_logged):
                complementary_logged = _nan_safe_div(r["complementary_nondeg_m"], r["dt_scan_s"])
            add_nondeg_speed.append(complementary_logged)

            complementary_odom_speed_orig.append(r["complementary_odom_lin_speed_orig_mps"])
            lidar_speed_proj_scale1.append(r["lidar_lin_speed_proj_scale1_mps"])
            lidar_speed_after_scale.append(r["lidar_lin_speed_after_scale_mps"])

        min_nondeg_speed = [_nan_safe_div(m, dt) for m, dt in zip(min_nondeg_m, dt_scan_s)]

        t_twist = _rel_time([r["stamp"] for r in twist_rows]) if twist_rows else []
        lin_norm = [r["lin_norm"] for r in twist_rows] if twist_rows else []
        ang_norm = [r["ang_norm"] for r in twist_rows] if twist_rows else []

        fig, axes = plt.subplots(4, 1, figsize=(16, 13), sharex=False)

        # 1) Scale traces
        ax = axes[0]
        ax.plot(t_scale, scale_applied, label="scale_applied", linewidth=2.0)
        ax.plot(t_scale, scale_ratio_raw, label="scale_ratio_raw", alpha=0.8)
        ax.plot(t_scale, scale_ls_raw, label="scale_ls_raw", alpha=0.8)
        ax.set_ylim(0.5, 1.5)
        ax.set_ylabel("scale")
        ax.set_title("Add-odom scale estimates")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

        # 2) Theta and gating/enabled state
        ax = axes[1]
        ax.plot(t_scale, theta_deg, label="theta_deg", color="tab:purple", linewidth=1.5)
        # Put gate/enabled as 0/1 lines on same axis for quick alignment checks.
        ax.plot(t_scale, [g * 180.0 for g in gate_passed], label="gate_passed * 180", color="tab:green", alpha=0.4)
        ax.plot(t_scale, [e * 170.0 for e in enabled], label="enabled * 170", color="tab:gray", alpha=0.35)
        ax.set_ylabel("deg")
        ax.set_title("Direction mismatch and gate/enable states")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

        # 3) Observable non-degenerate translational twist magnitudes (speed)
        ax = axes[2]
        ax.plot(t_scale, lidar_nondeg_speed, label="lidar_nondeg_speed [m/s]", color="tab:blue")
        ax.plot(t_scale, add_nondeg_speed, label="add_nondeg_speed [m/s]", color="tab:orange")
        ax.plot(t_scale, lidar_speed_proj_scale1, label="lidar_speed_proj_scale1 [m/s]", color="tab:cyan", alpha=0.8)
        ax.plot(t_scale, lidar_speed_after_scale, label="lidar_speed_after_scale [m/s]", color="tab:olive", alpha=0.8)
        ax.plot(t_scale, complementary_odom_speed_orig, label="complementary_odom_speed_orig [m/s]", color="tab:green", alpha=0.7)
        ax.plot(t_scale, min_nondeg_speed, label="min_nondeg_speed [m/s]", color="tab:red", linestyle="--")
        ax.set_ylim(0.0, 2.0)
        ax.set_ylabel("m/s")
        ax.set_title("Observable non-degenerate translational speed")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

        # 4) Raw complementary-odom twist telemetry
        ax = axes[3]
        if t_twist:
            ax.plot(t_twist, lin_norm, label="COMPLEMENTARY_ODOM_TWIST lin_norm", color="tab:green")
            ax.plot(t_twist, ang_norm, label="COMPLEMENTARY_ODOM_TWIST ang_norm", color="tab:brown")
            ax.legend(loc="best")
        else:
            ax.text(0.01, 0.5, "No [COMPLEMENTARY_ODOM_TWIST] events found", transform=ax.transAxes, va="center")
        ax.set_xlabel("time since first sample [s]")
        ax.set_ylabel("twist norm")
        ax.set_title("Complementary odometry twist norms")
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        output_path = args.output if args.output else os.path.join(run_dir, "complementary_odom_scale_diagnostics.png")
        output_path = _expand(output_path)
        plt.savefig(output_path, dpi=160)
        print(f"Saved plot: {output_path}")

        if args.show:
            plt.show()

    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
