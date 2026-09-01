"""Writers for the estimator debug traces emitted in scale_mode "estimated"."""

import csv

import numpy as np


def _num(x):
    return f"{float(x):.9f}" if np.isfinite(x) else "nan"


def write_scale_trace_csv(path, scale_trace):
    # The lateral_* columns are the cross-track half of the same sample, in
    # units of |comp|; they are all nan unless the correction applies one.
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
            "lateral_instant_raw",
            "lateral_filtered",
            "lateral_smooth",
            "lateral_applied",
        ])
        for s in scale_trace:
            writer.writerow([
                s.frame_idx,
                f"{s.time:.9f}",
                1 if s.gate_observable else 0,
                _num(s.scale_instant_raw),
                _num(s.scale_filtered),
                _num(s.scale_smooth),
                _num(s.scale_applied),
                _num(s.lateral_instant_raw),
                _num(s.lateral_filtered),
                _num(s.lateral_smooth),
                _num(s.lateral_applied),
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
    # speed_gate_observable is the observability gate on its own; under a
    # lines-meet correction gate_observable is additionally narrowed by whether
    # the fit was accepted, so only the former says which frames' lines the fit
    # was made of.
    header = ["frame_idx", "time", "degeneracy_detected", "gate_observable",
              "speed_gate_observable"]
    for _, col in _vec_fields:
        for ax in ("x", "y", "z"):
            header.append(f"{col}/{ax}")
    # meet_point is 2D and in units of |comp|, not metres in the map frame, so
    # it is not one of the vector fields above.
    # window_rotation_rad is how far the lag window turned. It is the axis to
    # plot the samples against: in the LiDAR frame the two are correlated
    # through the lever arm, and that correlation is the artefact the
    # complementary estimation frame exists to remove.
    header += ["meet_x", "meet_y", "window_rotation_rad",
               "scale_instant_raw", "scale_smooth", "scale_applied",
               "lateral_instant_raw", "lateral_smooth", "lateral_applied"]

    _fs = _num

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
                1 if getattr(vf, "speed_gate_observable", vf.gate_observable) else 0,
            ]
            for attr, _ in _vec_fields:
                row.extend(_fv(getattr(vf, attr)))
            meet = getattr(vf, "meet_point", None)
            row += _fv(meet if meet is not None else (np.nan, np.nan))
            row += [_fs(getattr(vf, "window_rotation_rad", np.nan))]
            row += [_fs(vf.scale_instant_raw), _fs(vf.scale_smooth), _fs(vf.scale_applied),
                    _fs(getattr(vf, "lateral_instant_raw", np.nan)),
                    _fs(getattr(vf, "lateral_smooth", np.nan)),
                    _fs(getattr(vf, "lateral_applied", np.nan))]
            writer.writerow(row)
