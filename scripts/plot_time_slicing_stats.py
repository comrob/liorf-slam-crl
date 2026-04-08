#!/usr/bin/env python3

import argparse
import csv
import glob
import os
import sys
from collections import defaultdict


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


def _resolve_csv_path(input_path: str, latest: bool, base_dir: str) -> str:
    if latest or not input_path:
        run_dir = _latest_run_dir(base_dir)
        csv_path = os.path.join(run_dir, "timing_stats.csv")
        if not os.path.isfile(csv_path):
            raise FileNotFoundError(f"timing_stats.csv not found in latest run directory: {run_dir}")
        return csv_path

    expanded = _expand(input_path)
    if os.path.isdir(expanded):
        csv_path = os.path.join(expanded, "timing_stats.csv")
    else:
        csv_path = expanded

    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")
    return csv_path


def _moving_average(values, window):
    if window <= 1:
        return values
    out = []
    running = 0.0
    q = []
    for v in values:
        q.append(v)
        running += v
        if len(q) > window:
            running -= q.pop(0)
        out.append(running / len(q))
    return out


def _load_timing_csv(csv_path: str):
    per_stage_t = defaultdict(list)
    per_stage_ms = defaultdict(list)

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        expected = {"stamp_sec", "stage", "elapsed_ms"}
        if not reader.fieldnames or not expected.issubset(set(reader.fieldnames)):
            raise ValueError(
                f"Unexpected CSV format in {csv_path}. "
                f"Expected columns: {sorted(expected)}"
            )

        for row in reader:
            try:
                ts = float(row["stamp_sec"])
                stage = str(row["stage"])
                ms = float(row["elapsed_ms"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid row in {csv_path}: {row}") from exc

            per_stage_t[stage].append(ts)
            per_stage_ms[stage].append(ms)

    if not per_stage_t:
        raise ValueError(f"No timing samples found in: {csv_path}")

    return per_stage_t, per_stage_ms


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot LIORF time-slicing diagnostics from timing_stats.csv"
    )
    parser.add_argument(
        "--input",
        default="",
        help="Path to timing_stats.csv or run directory containing timing_stats.csv",
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
        help="Output image path (default: <run_dir>/timing_stats_plot.png)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=1,
        help="Moving-average window (samples), default: 1 (no smoothing)",
    )
    args = parser.parse_args()

    try:
        csv_path = _resolve_csv_path(args.input, args.latest, args.base_dir)
        per_stage_t, per_stage_ms = _load_timing_csv(csv_path)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(
            "[ERROR] matplotlib is required to plot timing stats. "
            "Install it with: pip install matplotlib\n"
            f"Details: {exc}",
            file=sys.stderr,
        )
        return 3

    fig, ax = plt.subplots(figsize=(12, 6))
    for stage in sorted(per_stage_t.keys()):
        t = per_stage_t[stage]
        y = _moving_average(per_stage_ms[stage], max(1, args.window))
        t0 = t[0]
        t_rel = [x - t0 for x in t]
        ax.plot(t_rel, y, label=stage, linewidth=1.2)

    ax.set_title("LIORF Time Slicing Statistics")
    ax.set_xlabel("Time since first sample [s]")
    ax.set_ylabel("Elapsed time [ms]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()

    output_path = args.output.strip()
    if not output_path:
        output_path = os.path.join(os.path.dirname(csv_path), "timing_stats_plot.png")
    output_path = _expand(output_path)

    fig.savefig(output_path, dpi=150)
    print(f"Saved plot: {output_path}")

    plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
