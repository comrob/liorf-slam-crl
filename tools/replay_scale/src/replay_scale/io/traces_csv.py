"""Writers for the estimator debug traces emitted in scale_mode "estimated"."""

import csv

import numpy as np


def write_scale_trace_csv(path, scale_trace):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "frame_idx",
            "time",
            "gate_observable",
            "scale_instant_raw",
            "scale_filtered",
            "scale_smooth",
            "scale_applied",
        ])
        for s in scale_trace:
            writer.writerow([
                s.frame_idx,
                f"{s.time:.9f}",
                1 if s.gate_observable else 0,
                f"{s.scale_instant_raw:.9f}" if np.isfinite(s.scale_instant_raw) else "nan",
                f"{s.scale_filtered:.9f}" if np.isfinite(s.scale_filtered) else "nan",
                f"{s.scale_smooth:.9f}" if np.isfinite(s.scale_smooth) else "nan",
                f"{s.scale_applied:.9f}" if np.isfinite(s.scale_applied) else "nan",
            ])


def write_scale_vector_csv(path, vector_trace):
    if not vector_trace:
        return

    _vec_fields = (
        ("anchor_pos",              "anchor"),
        ("latest_pos",              "latest"),
        ("t_lidar_map",             "t_lidar_map"),
        ("t_comp_map",              "t_comp_map"),
        ("t_lidar_nondeg_map",      "t_lidar_nondeg_map"),
        ("t_comp_nondeg_map",       "t_comp_nondeg_map"),
        ("t_lidar_nondeg_proj_map", "t_lidar_nondeg_proj_map"),
        ("nondeg_axis_map",         "nondeg_axis_map"),
    )
    header = ["frame_idx", "time", "degeneracy_detected", "gate_observable"]
    for _, col in _vec_fields:
        for ax in ("x", "y", "z"):
            header.append(f"{col}/{ax}")
    header += ["scale_instant_raw", "scale_smooth", "scale_applied"]

    def _fs(x):
        return f"{float(x):.9f}" if np.isfinite(x) else "nan"

    def _fv(v):
        return [_fs(c) for c in v]

    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for vf in vector_trace:
            row = [
                vf.frame_idx,
                f"{vf.time:.9f}",
                1 if vf.degeneracy_detected else 0,
                1 if vf.gate_observable else 0,
            ]
            for attr, _ in _vec_fields:
                row.extend(_fv(getattr(vf, attr)))
            row += [_fs(vf.scale_instant_raw), _fs(vf.scale_smooth), _fs(vf.scale_applied)]
            writer.writerow(row)
