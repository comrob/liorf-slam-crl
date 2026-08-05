#!/usr/bin/env python3

import argparse
import csv
import glob
import math
import os
import sys
from collections import defaultdict
import statistics


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


def _percentile(sorted_values, q):
    if not sorted_values:
        return float("nan")
    if len(sorted_values) == 1:
        return sorted_values[0]
    q = min(100.0, max(0.0, float(q)))
    pos = (len(sorted_values) - 1) * (q / 100.0)
    lo = int(pos)
    hi = min(len(sorted_values) - 1, lo + 1)
    frac = pos - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def _build_stage_summary(per_stage_ms):
    rows = []
    for stage, values in per_stage_ms.items():
        if not values:
            continue
        sorted_vals = sorted(values)
        n = len(sorted_vals)
        mean_v = statistics.fmean(sorted_vals)
        median_v = statistics.median(sorted_vals)
        p90_v = _percentile(sorted_vals, 90)
        p95_v = _percentile(sorted_vals, 95)
        min_v = sorted_vals[0]
        max_v = sorted_vals[-1]
        std_v = statistics.pstdev(sorted_vals) if n > 1 else 0.0
        rows.append({
            "stage": stage,
            "n": n,
            "mean_ms": mean_v,
            "median_ms": median_v,
            "p90_ms": p90_v,
            "p95_ms": p95_v,
            "min_ms": min_v,
            "max_ms": max_v,
            "std_ms": std_v,
        })
    return rows


def _print_stage_summary(rows, sort_by="mean"):
    if not rows:
        return

    sort_key_map = {
        "name": lambda r: r["stage"],
        "mean": lambda r: r["mean_ms"],
        "p95": lambda r: r["p95_ms"],
        "max": lambda r: r["max_ms"],
    }
    key_fn = sort_key_map.get(sort_by, sort_key_map["mean"])
    reverse = sort_by in {"mean", "p95", "max"}
    rows_sorted = sorted(rows, key=key_fn, reverse=reverse)

    print("\nDetailed timing statistics per stage [ms]:")
    print(
        f"{'stage':55s} {'n':>7s} {'mean':>10s} {'median':>10s} "
        f"{'p90':>10s} {'p95':>10s} {'min':>10s} {'max':>10s} {'std':>10s}"
    )
    print("-" * 145)
    for r in rows_sorted:
        print(
            f"{r['stage'][:55]:55s} "
            f"{r['n']:7d} "
            f"{r['mean_ms']:10.3f} "
            f"{r['median_ms']:10.3f} "
            f"{r['p90_ms']:10.3f} "
            f"{r['p95_ms']:10.3f} "
            f"{r['min_ms']:10.3f} "
            f"{r['max_ms']:10.3f} "
            f"{r['std_ms']:10.3f}"
        )


def _save_stage_summary_csv(rows, output_path):
    fieldnames = [
        "stage",
        "n",
        "mean_ms",
        "median_ms",
        "p90_ms",
        "p95_ms",
        "min_ms",
        "max_ms",
        "std_ms",
    ]
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in sorted(rows, key=lambda r: r["stage"]):
            writer.writerow(row)


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
        description="Plot LILI time-slicing diagnostics from timing_stats.csv"
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
        default="~/.ros/lili_logs",
        help="Base diagnostics log directory (default: ~/.ros/lili_logs)",
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
    parser.add_argument(
        "--show-summary",
        action="store_true",
        help="Print detailed per-stage stats to console (disabled by default)",
    )
    parser.add_argument(
        "--sort-summary-by",
        choices=["name", "mean", "p95", "max"],
        default="mean",
        help="Sort order for printed/exported detailed stats (default: mean)",
    )
    parser.add_argument(
        "--summary-output",
        default="",
        help="Optional path to write detailed stats CSV",
    )
    parser.add_argument(
        "--plot-mode",
        choices=["overlay", "facets", "both"],
        default="both",
        help="Plot style: overlay(single axes), facets(per-stage subplots), or both side-by-side (default: both)",
    )
    parser.add_argument(
        "--max-facet-stages",
        type=int,
        default=24,
        help="Maximum number of stages to draw in faceted mode (sorted by mean time), default: 24",
    )
    parser.add_argument(
        "--facet-cols",
        type=int,
        default=4,
        help="Number of columns for faceted plot grid, default: 4",
    )
    args = parser.parse_args()

    try:
        csv_path = _resolve_csv_path(args.input, args.latest, args.base_dir)
        per_stage_t, per_stage_ms = _load_timing_csv(csv_path)
        summary_rows = _build_stage_summary(per_stage_ms)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    if args.show_summary:
        _print_stage_summary(summary_rows, sort_by=args.sort_summary_by)

    summary_output = args.summary_output.strip()
    if summary_output:
        summary_output = _expand(summary_output)
        summary_dir = os.path.dirname(summary_output)
        if summary_dir:
            os.makedirs(summary_dir, exist_ok=True)
        _save_stage_summary_csv(summary_rows, summary_output)
        print(f"Saved summary CSV: {summary_output}")

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

    stages = sorted(per_stage_t.keys())
    t_ref = min(per_stage_t[s][0] for s in stages if per_stage_t[s])

    def _stage_rel_xy(stage_name):
        t = per_stage_t[stage_name]
        y = _moving_average(per_stage_ms[stage_name], max(1, args.window))
        t_rel = [x - t_ref for x in t]
        return t_rel, y

    fig = None

    if args.plot_mode == "overlay":
        fig, ax = plt.subplots(figsize=(12, 6))
        for stage in stages:
            t_rel, y = _stage_rel_xy(stage)
            ax.plot(t_rel, y, label=stage, linewidth=1.2)

        ax.set_title("LILI Time Slicing Statistics (overlay)")
        ax.set_xlabel("Time since first sample [s]")
        ax.set_ylabel("Elapsed time [ms]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()

    elif args.plot_mode == "facets":
        rows_sorted = sorted(summary_rows, key=lambda r: r["mean_ms"], reverse=True)
        max_stages = max(1, args.max_facet_stages)
        selected = [r["stage"] for r in rows_sorted[:max_stages]]

        cols = max(1, args.facet_cols)
        n = len(selected)
        nrows = max(1, math.ceil(n / cols))

        fig, axes = plt.subplots(nrows=nrows, ncols=cols, figsize=(4.2 * cols, 2.6 * nrows), sharex=True)
        axes_list = axes.flat if hasattr(axes, "flat") else [axes]

        for i, stage in enumerate(selected):
            ax = axes_list[i]
            t_rel, y = _stage_rel_xy(stage)
            ax.plot(t_rel, y, linewidth=1.0)
            ax.set_title(stage, fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=7)

        for j in range(len(selected), nrows * cols):
            axes_list[j].axis("off")

        fig.suptitle("LILI Time Slicing Statistics (detailed facets)", fontsize=12)
        fig.supxlabel("Time since first sample [s]")
        fig.supylabel("Elapsed time [ms]")
        fig.tight_layout(rect=[0, 0, 1, 0.97])

    else:  # both
        fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(16, 6), sharex=True)

        # Left: high-level pipeline view
        high_level_stages = [
            "updateInitialGuess",
            "manageLocalMap",
            "downsampleCurrentScan",
            "scan2MapOptimization",
            "saveKeyFramesAndFactor",
            "correctPoses",
            "updateRollingMap",
        ]
        present_high_level = [s for s in high_level_stages if s in per_stage_t]

        if present_high_level:
            for stage in present_high_level:
                t_rel, y = _stage_rel_xy(stage)
                ax_left.plot(t_rel, y, label=stage, linewidth=1.3)
        else:
            # Fallback for older logs without high-level stage names
            for stage in stages:
                t_rel, y = _stage_rel_xy(stage)
                ax_left.plot(t_rel, y, label=stage, linewidth=1.0)

        ax_left.set_title("High-level pipeline stages")
        ax_left.set_xlabel("Time since first sample [s]")
        ax_left.set_ylabel("Elapsed time [ms]")
        ax_left.grid(True, alpha=0.3)
        ax_left.legend(loc="upper right", fontsize=7)

        # Right: map-handling and extraction components
        map_stage_prefixes = (
            "manageLocalMap",
            "updateRollingMap",
            "extractSurroundingKeyFrames",
            "extractNearby",
            "extractCloud",
        )
        map_component_stages = [
            s for s in stages
            if s.startswith(map_stage_prefixes) or "mapHandling" in s
        ]

        if map_component_stages:
            for stage in map_component_stages:
                t_rel, y = _stage_rel_xy(stage)
                ax_right.plot(t_rel, y, label=stage, linewidth=1.2)
            ax_right.legend(loc="upper right", fontsize=8)
        else:
            ax_right.text(
                0.5,
                0.5,
                "No map-handling component slices found",
                transform=ax_right.transAxes,
                ha="center",
                va="center",
                fontsize=10,
            )

        ax_right.set_title("Map handling + extraction components")
        ax_right.set_xlabel("Time since first sample [s]")
        ax_right.set_ylabel("Elapsed time [ms]")
        ax_right.grid(True, alpha=0.3)

        fig.suptitle("LILI Time Slicing Statistics", fontsize=12)
        fig.tight_layout(rect=[0, 0, 1, 0.95])

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
