"""Data model and trajectory reconstruction / scale estimation logic.

Port of:
- src/mapOptimization/mapOptimization_degeneracy.cpp
"""

from collections import deque
from dataclasses import dataclass

import numpy as np

from .se3_math import (
    exp_map,
    matrix_to_twist,
    project_degenerate_correction,
    project_onto_basis_translation,
)


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

@dataclass
class ReplayParams:
    translation_scale: float = 1.0
    scale_estimation_apply: bool = True
    scale_min_nondegenerate_speed: float = 0.2
    scale_baseline_frame_lag: int = 1
    scale_smoothing_window_size: int = 20
    ignore_dz: bool = False


class Frame:
    __slots__ = (
        "time", "dt_scan", "degeneracy_detected", "has_basis",
        "has_complementary", "scale_applied", "basis_size", "basis",
        "lidar_increment", "complementary_twist", "dt_complementary",
        "pose_prev", "pose_optimized", "pose_effective",
    )


@dataclass
class ScaleEstimateFrame:
    frame_idx: int
    time: float
    gate_observable: bool
    scale_instant_raw: float
    scale_filtered: float
    scale_smooth: float
    scale_applied: float


class ScaleVectorFrame:
    __slots__ = (
        "frame_idx", "time", "degeneracy_detected", "gate_observable",
        "anchor_pos", "latest_pos",
        "t_lidar_map", "t_comp_map",
        "t_lidar_nondeg_map", "t_comp_nondeg_map",
        "t_lidar_nondeg_proj_map", "nondeg_axis_map",
        "scale_instant_raw", "scale_smooth", "scale_applied",
    )


# ---------------------------------------------------------------------------
# Trajectory reconstruction
# ---------------------------------------------------------------------------

def reconstruct_fixed(frames, scale_of_frame, apply_correction=True, translation_scale_multiplier=1.0):
    """Chain a trajectory from LiDAR increments using fixed/recorded scales."""
    if not frames:
        return []

    T_prev = frames[0].pose_prev.copy()
    out = []
    for f in frames:
        T_optimized = T_prev @ exp_map(f.lidar_increment, 1.0)

        do_correction = (apply_correction and f.degeneracy_detected
                         and f.has_basis and len(f.basis) > 0)
        if do_correction:
            if f.has_complementary:
                scale = scale_of_frame(f)
                dt = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
                xi_comp = f.complementary_twist.copy()
                xi_comp[:3] *= translation_scale_multiplier
                T_comp_rel = exp_map(xi_comp, dt)
                T_comp_rel[:3, 3] *= scale
                T_comp_abs = T_prev @ T_comp_rel
            else:
                T_comp_abs = T_prev.copy()
            T_corrected = project_degenerate_correction(T_optimized, T_comp_abs, f.basis)
        else:
            T_corrected = T_optimized

        out.append((f.time, T_corrected))
        T_prev = T_corrected
    return out


def reconstruct_complementary_only(frames, translation_scale_multiplier=1.0):
    """Chain a trajectory purely from complementary (additional) odometry twists.

    No LiDAR fusion or estimated-scale correction is applied; only the
    replay-time translationScale multiplier is used, so this shows the raw
    additional-odometry sensor's own drift/shape for comparison.
    """
    if not frames:
        return []

    T_prev = frames[0].pose_prev.copy()
    out = []
    for f in frames:
        if f.has_complementary and np.all(np.isfinite(f.complementary_twist)):
            dt = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
            xi_comp = f.complementary_twist.copy()
            xi_comp[:3] *= translation_scale_multiplier
            T_next = T_prev @ exp_map(xi_comp, dt)
        else:
            T_next = T_prev.copy()
        out.append((f.time, T_next))
        T_prev = T_next
    return out


def build_additional_odom_scale_sample(
    T_anchor,
    T_latest,
    T_lidar_rel,
    T_comp_rel,
    basis,
    ignore_dz,
    dt_complementary_s,
    min_nondegenerate_speed,
    debug=False,
):
    """Port of mapOptimization::buildAdditionalOdomCorrectionResult for scaling."""
    t_lidar_rel_anchor = T_lidar_rel[:3, 3].copy()
    t_comp_rel_uncorrected_anchor = T_comp_rel[:3, 3].copy()

    R_lidar_rel = T_lidar_rel[:3, :3]
    R_comp_rel = T_comp_rel[:3, :3]
    R_orientation_drift = R_comp_rel @ R_lidar_rel.T
    t_comp_rel_corrected_anchor = R_orientation_drift.T @ t_comp_rel_uncorrected_anchor

    if ignore_dz:
        t_lidar_rel_anchor[2] = 0.0
        t_comp_rel_uncorrected_anchor[2] = 0.0
        t_comp_rel_corrected_anchor[2] = 0.0

    R_anchor = T_anchor[:3, :3]
    R_latest = T_latest[:3, :3]

    t_comp_corrected_map = R_anchor @ t_comp_rel_corrected_anchor

    t_lidar_map = T_latest[:3, 3] - T_anchor[:3, 3]
    t_lidar_latest_local = R_latest.T @ t_lidar_map
    t_comp_latest_local = R_latest.T @ t_comp_corrected_map

    if ignore_dz:
        t_lidar_latest_local[2] = 0.0
        t_comp_latest_local[2] = 0.0

    t_lidar_nondeg = t_lidar_latest_local - project_onto_basis_translation(t_lidar_latest_local, basis)
    t_comp_nondeg = t_comp_latest_local - project_onto_basis_translation(t_comp_latest_local, basis)

    comp_nondeg_norm = float(np.linalg.norm(t_comp_nondeg))
    comp_unit = np.zeros(3, dtype=float)
    if comp_nondeg_norm > 1e-6:
        comp_unit = t_comp_nondeg / comp_nondeg_norm

    t_lidar_nondeg_proj = float(t_lidar_nondeg @ comp_unit) * comp_unit
    lidar_proj_norm = float(np.linalg.norm(t_lidar_nondeg_proj))

    scale_instant_raw = np.nan
    if comp_nondeg_norm > 1e-6:
        scale_instant_raw = lidar_proj_norm / comp_nondeg_norm

    projected_nondeg_speed = lidar_proj_norm / max(1e-5, dt_complementary_s)
    gate_observable = (
        np.isfinite(scale_instant_raw)
        and projected_nondeg_speed >= max(0.0, min_nondegenerate_speed)
    )

    result = {
        "valid": True,
        "gate_observable": bool(gate_observable),
        "scale_instant_raw": float(scale_instant_raw),
        "scale_filtered": float(scale_instant_raw) if gate_observable else np.nan,
    }
    if debug:
        result["debug"] = {
            "anchor_pos": T_anchor[:3, 3].copy(),
            "latest_pos": T_latest[:3, 3].copy(),
            "t_lidar_map": t_lidar_map.copy(),
            "t_comp_map": t_comp_corrected_map.copy(),
            "t_lidar_nondeg_map": R_latest @ t_lidar_nondeg,
            "t_comp_nondeg_map": R_latest @ t_comp_nondeg,
            "t_lidar_nondeg_proj_map": R_latest @ t_lidar_nondeg_proj,
            "nondeg_axis_map": R_latest @ comp_unit,
        }
    return result


def reconstruct_with_estimator(frames, params, collect_vectors=False):
    """Replay with online-style lagged scale estimation and application.

    Notes about parity with C++:
    - Uses same observability gate and smoothing history update rule.
    - Uses same causal ordering: applied scale at frame k comes from history
      accumulated up to frame k-1.
    - In replay, estimation is always on; only application is gated by
      ``scaleEstimationApply``.
    """
    if not frames:
        return [], [], []

    T_prev = frames[0].pose_prev.copy()
    out = []
    scale_trace = []
    vector_trace = []

    baseline_lag = max(1, int(params.scale_baseline_frame_lag))
    lidar_pose_buffer = deque()  # tuples: (frame_idx, stamp, T_effective)
    lagged_scale_filtered_history = deque()

    n = len(frames)
    comp_step_rel = [None] * n  # index k stores rel transform from k-1 -> k.
    comp_step_dt = [np.nan] * n

    for k, f in enumerate(frames):
        basis = f.basis if (f.has_basis and len(f.basis) > 0) else []
        estimator_mode_active = bool(f.degeneracy_detected)
        scale_apply_enabled = bool(params.scale_estimation_apply)

        smoothed_scale_for_apply = np.nan
        if scale_apply_enabled and len(lagged_scale_filtered_history) > 0:
            smoothed_scale_for_apply = float(np.mean(lagged_scale_filtered_history))

        has_applied_scale = scale_apply_enabled and np.isfinite(smoothed_scale_for_apply)
        scale_applied = float(smoothed_scale_for_apply) if has_applied_scale else 1.0

        T_optimized = T_prev @ exp_map(f.lidar_increment, 1.0)
        T_comp_scaled_abs = T_prev.copy()

        has_comp = bool(f.has_complementary and np.all(np.isfinite(f.complementary_twist)))
        if has_comp:
            dt_pred = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
            dt_pred = max(1e-5, float(dt_pred))
            xi_comp = f.complementary_twist.copy()
            # Recorded twists already include the original run's translationScale.
            # This multiplier is interpreted as an additional replay-time factor.
            xi_comp[:3] *= float(params.translation_scale)

            T_comp_rel_unscaled = exp_map(xi_comp, dt_pred)
            T_comp_rel_scaled = T_comp_rel_unscaled.copy()
            T_comp_rel_scaled[:3, 3] *= scale_applied

            T_comp_scaled_abs = T_prev @ T_comp_rel_scaled

            comp_step_rel[k] = T_comp_rel_unscaled
            comp_step_dt[k] = dt_pred

        if estimator_mode_active and len(basis) > 0:
            T_corrected = project_degenerate_correction(T_optimized, T_comp_scaled_abs, basis)
        else:
            T_corrected = T_optimized

        T_effective = T_corrected if (estimator_mode_active and len(basis) > 0) else T_optimized
        out.append((f.time, T_effective))

        lidar_pose_buffer.append((k, f.time, T_effective.copy()))
        while len(lidar_pose_buffer) > baseline_lag + 1:
            lidar_pose_buffer.popleft()

        gate_observable = False
        scale_instant_raw = np.nan
        scale_filtered = np.nan
        debug_vecs = None

        # Port of lagged scale update path: if buffer has anchor/current pair,
        # estimate lagged scale sample and append to filtered history when observable.
        if len(lidar_pose_buffer) >= 2:
            anchor_idx, _, T_anchor = lidar_pose_buffer[0]
            latest_idx, _, T_latest = lidar_pose_buffer[-1]

            T_comp_rel_window = np.eye(4, dtype=float)
            dt_comp_window = 0.0
            have_full_window = True
            for j in range(anchor_idx + 1, latest_idx + 1):
                if comp_step_rel[j] is None or not np.isfinite(comp_step_dt[j]):
                    have_full_window = False
                    break
                T_comp_rel_window = T_comp_rel_window @ comp_step_rel[j]
                dt_comp_window += float(comp_step_dt[j])

            if have_full_window and dt_comp_window > 1e-5:
                T_lidar_rel = np.linalg.inv(T_anchor) @ T_latest
                sample = build_additional_odom_scale_sample(
                    T_anchor,
                    T_latest,
                    T_lidar_rel,
                    T_comp_rel_window,
                    basis,
                    bool(params.ignore_dz),
                    dt_comp_window,
                    float(params.scale_min_nondegenerate_speed),
                    debug=collect_vectors,
                )
                gate_observable = sample["gate_observable"]
                scale_instant_raw = sample["scale_instant_raw"]
                scale_filtered = sample["scale_filtered"]
                if collect_vectors:
                    debug_vecs = sample.get("debug")

                if np.isfinite(scale_filtered):
                    lagged_scale_filtered_history.append(float(scale_filtered))
                while len(lagged_scale_filtered_history) > max(1, int(params.scale_smoothing_window_size)):
                    lagged_scale_filtered_history.popleft()

        scale_smooth = float(np.mean(lagged_scale_filtered_history)) if lagged_scale_filtered_history else np.nan
        scale_trace.append(ScaleEstimateFrame(
            frame_idx=k,
            time=float(f.time),
            gate_observable=bool(gate_observable),
            scale_instant_raw=float(scale_instant_raw),
            scale_filtered=float(scale_filtered),
            scale_smooth=float(scale_smooth),
            scale_applied=float(scale_applied),
        ))
        if collect_vectors:
            _nan3 = np.full(3, np.nan)
            vf = ScaleVectorFrame()
            vf.frame_idx = k
            vf.time = float(f.time)
            vf.degeneracy_detected = bool(estimator_mode_active)
            vf.gate_observable = bool(gate_observable)
            if debug_vecs is not None:
                vf.anchor_pos = debug_vecs["anchor_pos"]
                vf.latest_pos = debug_vecs["latest_pos"]
                vf.t_lidar_map = debug_vecs["t_lidar_map"]
                vf.t_comp_map = debug_vecs["t_comp_map"]
                vf.t_lidar_nondeg_map = debug_vecs["t_lidar_nondeg_map"]
                vf.t_comp_nondeg_map = debug_vecs["t_comp_nondeg_map"]
                vf.t_lidar_nondeg_proj_map = debug_vecs["t_lidar_nondeg_proj_map"]
                vf.nondeg_axis_map = debug_vecs["nondeg_axis_map"]
            else:
                for _attr in ("anchor_pos", "latest_pos", "t_lidar_map", "t_comp_map",
                              "t_lidar_nondeg_map", "t_comp_nondeg_map",
                              "t_lidar_nondeg_proj_map", "nondeg_axis_map"):
                    setattr(vf, _attr, _nan3.copy())
            vf.scale_instant_raw = float(scale_instant_raw)
            vf.scale_smooth = float(scale_smooth)
            vf.scale_applied = float(scale_applied)
            vector_trace.append(vf)

        T_prev = T_effective

    return out, scale_trace, vector_trace
