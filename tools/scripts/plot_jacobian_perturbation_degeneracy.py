#!/usr/bin/env python3

import argparse
import bisect
import csv
import glob
import os
import sys
from typing import List, Optional, Tuple


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


def _resolve_run_dir(args) -> str:
    if args.latest or (not args.run_dir):
        return _latest_run_dir(args.base_dir)

    run_dir = _expand(args.run_dir)
    if not os.path.isdir(run_dir):
        raise FileNotFoundError(f"Run directory not found: {run_dir}")
    return run_dir


def _parse_pipe_float_array(raw: str, expected_len: int) -> List[float]:
    parts = [p for p in str(raw).split("|") if p != ""]
    vals = [float(p) for p in parts]
    if len(vals) != expected_len:
        raise ValueError(f"Expected {expected_len} values, got {len(vals)} in: {raw}")
    return vals


def _load_jacobian(csv_path: str, backend: Optional[str]) -> Tuple[List[float], List[List[float]], List[int]]:
    times: List[float] = []
    eigenvalues: List[List[float]] = []
    is_degenerate: List[int] = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        expected = {
            "stamp_sec",
            "module_name",
            "computed",
            "is_degenerate",
            "eigenvalues",
        }
        if not reader.fieldnames or not expected.issubset(set(reader.fieldnames)):
            raise ValueError(
                f"Unexpected format in {csv_path}. Expected columns at least: {sorted(expected)}"
            )

        for row in reader:
            if backend and row.get("module_name", "") != backend:
                continue

            ts = float(row["stamp_sec"])
            eig = _parse_pipe_float_array(row["eigenvalues"], 6)
            d = int(row["is_degenerate"])

            times.append(ts)
            eigenvalues.append(eig)
            is_degenerate.append(d)

    if not times:
        raise ValueError(f"No Jacobian rows loaded from: {csv_path}")

    return times, eigenvalues, is_degenerate


def _load_perturbation(csv_path: str) -> Tuple[List[float], List[int]]:
    times: List[float] = []
    detected: List[int] = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        expected = {"stamp_sec", "detected"}
        if not reader.fieldnames or not expected.issubset(set(reader.fieldnames)):
            raise ValueError(
                f"Unexpected format in {csv_path}. Expected columns at least: {sorted(expected)}"
            )

        for row in reader:
            times.append(float(row["stamp_sec"]))
            detected.append(int(row["detected"]))

    if not times:
        raise ValueError(f"No perturbation rows loaded from: {csv_path}")

    return times, detected


def _nearest_signal(
    query_times: List[float],
    ref_times: List[float],
    ref_values: List[int],
    tolerance_sec: float,
) -> List[int]:
    out: List[int] = []
    for t in query_times:
        idx = bisect.bisect_left(ref_times, t)
        best_idx = None
        best_dt = None

        for cand in (idx - 1, idx):
            if 0 <= cand < len(ref_times):
                dt = abs(ref_times[cand] - t)
                if best_dt is None or dt < best_dt:
                    best_dt = dt
                    best_idx = cand

        if best_idx is not None and best_dt is not None and best_dt <= tolerance_sec:
            out.append(int(ref_values[best_idx]))
        else:
            out.append(0)

    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot Jacobian eigenvalues and perturbation degeneracy detection over time"
    )
    parser.add_argument("--run-dir", default="", help="Path to diagnostics run directory")
    parser.add_argument("--latest", action="store_true", help="Use latest run under --base-dir")
    parser.add_argument(
        "--base-dir", default="~/.ros/lili_logs", help="Diagnostics base directory"
    )
    parser.add_argument(
        "--backend",
        default="",
        help="Optional backend filter for jacobian CSV (e.g., kdtree_lm or voxel_pko)",
    )
    parser.add_argument(
        "--tolerance-sec",
        type=float,
        default=0.05,
        help="Nearest timestamp matching tolerance from perturbation to Jacobian samples",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional output image path (default: <run_dir>/jacobian_perturbation_plot.png)",
    )
    parser.add_argument(
        "--min-eigen-output",
        default="",
        help="Optional output image path for standalone smallest-eigenvalue plot "
             "(default: <run_dir>/jacobian_min_eigen_plot.png)",
    )
    parser.add_argument(
        "--eigen-scale",
        choices=["linear", "log"],
        default="linear",
        help="Scale for eigenvalue plots (default: linear)",
    )
    parser.add_argument(
        "--log-eps",
        type=float,
        default=1e-12,
        help="Minimum positive clamp for log-scale eigenvalues (default: 1e-12)",
    )
    parser.add_argument(
        "--separate-min-eigen",
        action="store_true",
        help="Add a dedicated subplot for the smallest eigenvalue over time",
    )
    parser.add_argument("--no-show", action="store_true", help="Do not open interactive window")
    args = parser.parse_args()

    try:
        run_dir = _resolve_run_dir(args)
        jac_path = os.path.join(run_dir, "jacobian_degeneracy_metrics.csv")
        pert_path = os.path.join(run_dir, "perturbation_degeneracy_metrics.csv")

        if not os.path.isfile(jac_path):
            raise FileNotFoundError(f"Missing file: {jac_path}")
        if not os.path.isfile(pert_path):
            raise FileNotFoundError(
                f"Missing file: {pert_path}. Run with updated telemetry logging enabled."
            )

        backend = args.backend.strip() or None
        jac_t, jac_eigs, jac_deg = _load_jacobian(jac_path, backend)
        pert_t, pert_detected = _load_perturbation(pert_path)
        pert_on_jac = _nearest_signal(jac_t, pert_t, pert_detected, args.tolerance_sec)

        t0 = jac_t[0]
        x = [t - t0 for t in jac_t]

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            raise RuntimeError(
                "matplotlib is required. Install it in scripts env (poetry add matplotlib)."
            ) from exc

        min_eigs = [min(row) for row in jac_eigs]
        max_eigs = [max(row) for row in jac_eigs]
        ratio_eps = max(args.log_eps, 1e-20)
        max_min_ratio = []
        ratio_clamped_count = 0
        for max_v, min_v in zip(max_eigs, min_eigs):
            den = min_v
            if abs(den) <= ratio_eps:
                den = ratio_eps
                ratio_clamped_count += 1
            max_min_ratio.append(max_v / den)

        if ratio_clamped_count > 0:
            print(
                f"[INFO] Clamped {ratio_clamped_count} min-eigen denominators to {ratio_eps:.2e} for ratio plot"
            )

        eig_rows = jac_eigs
        min_rows = min_eigs
        if args.eigen_scale == "log":
            eig_eps = max(args.log_eps, 1e-20)
            clamped_count = 0
            eig_rows = []
            for row in jac_eigs:
                clamped_row = []
                for v in row:
                    if v <= eig_eps:
                        clamped_row.append(eig_eps)
                        clamped_count += 1
                    else:
                        clamped_row.append(v)
                eig_rows.append(clamped_row)

            min_rows = []
            for v in min_eigs:
                if v <= eig_eps:
                    min_rows.append(eig_eps)
                    clamped_count += 1
                else:
                    min_rows.append(v)

            if clamped_count > 0:
                print(
                    f"[INFO] Clamped {clamped_count} eigenvalue points to {eig_eps:.2e} for log scale"
                )

        if args.separate_min_eigen:
            fig, (ax0, ax1, ax2, ax3) = plt.subplots(
                4,
                1,
                figsize=(14, 12),
                sharex=True,
                gridspec_kw={"height_ratios": [3, 2, 2, 1]},
            )
        else:
            fig, (ax0, ax2, ax3) = plt.subplots(
                3,
                1,
                figsize=(14, 10),
                sharex=True,
                gridspec_kw={"height_ratios": [3, 2, 1]},
            )
            ax1 = None

        for i in range(6):
            ax0.plot(x, [row[i] for row in eig_rows], label=f"eig_{i}", linewidth=1.2)

        if args.eigen_scale == "log":
            ax0.set_yscale("log")

        ax0.set_ylabel("Eigenvalue")
        ax0.set_title("Jacobian Eigenvalues and Perturbation Degeneracy Detection")
        ax0.grid(True, alpha=0.25)
        ax0.legend(loc="upper right", ncol=3, fontsize=9)

        if ax1 is not None:
            ax1.plot(x, min_rows, label="min_eigenvalue", linewidth=1.3, color="black")
            if args.eigen_scale == "log":
                ax1.set_yscale("log")
            ax1.set_ylabel("Min Eig")
            ax1.grid(True, alpha=0.25)
            ax1.legend(loc="upper right")

        ax3.plot(x, max_min_ratio, label="max/min eigenvalue", linewidth=1.3, color="tab:orange")
        ax3.set_ylabel("Eig Ratio")
        ax3.grid(True, alpha=0.25)
        ax3.legend(loc="upper right")

        ax2.step(x, jac_deg, where="post", label="jacobian_is_degenerate", linewidth=1.4)
        ax2.step(x, pert_on_jac, where="post", label="perturbation_detected", linewidth=1.4)
        ax2.set_ylim(-0.1, 1.1)
        ax2.set_yticks([0, 1])
        ax2.set_ylabel("Flag")
        ax2.grid(True, alpha=0.25)
        ax2.legend(loc="upper right")
        ax2.set_xlabel("Time from run start [s]")

        fig.tight_layout()

        output_path = args.output.strip() or os.path.join(run_dir, "jacobian_perturbation_plot.png")
        fig.savefig(output_path, dpi=160)
        print(f"[INFO] Saved plot: {output_path}")

        # Always create a standalone smallest-eigenvalue plot for easier inspection.
        fig_min, ax_min = plt.subplots(1, 1, figsize=(14, 4))
        ax_min.plot(x, min_rows, label="min_eigenvalue", linewidth=1.4, color="black")
        if args.eigen_scale == "log":
            ax_min.set_yscale("log")
        ax_min.set_title("Smallest Jacobian Eigenvalue")
        ax_min.set_xlabel("Time from run start [s]")
        ax_min.set_ylabel("Min Eigenvalue")
        ax_min.grid(True, alpha=0.25)
        ax_min.legend(loc="upper right")
        fig_min.tight_layout()

        min_output_path = args.min_eigen_output.strip() or os.path.join(
            run_dir, "jacobian_min_eigen_plot.png"
        )
        fig_min.savefig(min_output_path, dpi=160)
        print(f"[INFO] Saved smallest-eigen plot: {min_output_path}")

        if not args.no_show:
            plt.show()

        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
