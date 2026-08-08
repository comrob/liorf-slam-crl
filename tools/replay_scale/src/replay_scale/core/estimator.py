"""Trajectory reconstruction / scale estimation logic.

Port of:
- src/mapOptimization/mapOptimization_degeneracy.cpp

Pure computation over :mod:`replay_scale.core.model` objects: reads no files,
writes no files and prints nothing.
"""

from collections import deque

import numpy as np

from .lines import fit_lines
from .model import (
    LINES_MEET_CORRECTIONS,
    SMOOTHING_MODES,
    ScaleEstimateFrame,
    ScaleVectorFrame,
)
from .se3 import (
    exp_map,
    orthonormal_translation_basis,
    project_degenerate_correction,
    project_degenerate_correction_translation,
    project_onto_basis_translation,
)

#: Below this an in-plane degenerate direction is numerically meaningless.
_MIN_INPLANE_DIR = 1e-6


#: Tukey fence width for the "trimmed" mode, in interquartile ranges. The
#: textbook 1.5 -- wide enough to keep a symmetric window intact, narrow enough
#: to cut the ratio's tail.
IQR_FENCE_K = 1.5

#: Below this many samples the quartiles are not meaningful and "trimmed"
#: cannot distinguish a tail from the data.
_MIN_TRIM_SAMPLES = 4


def own_normalized_line(t_lidar_local, t_comp_local, basis):
    """One frame's degenerate line, expressed so frames can be compared.

    Rotated so the complementary displacement points along +x and divided by its
    length, which puts every frame in the same units and the same orientation --
    the geometry the viewer draws with ``|comp| = 1`` in the complementary frame.
    The line runs through the LiDAR's latest position along the degenerate
    direction. Returns ``(origin, direction)`` as 2-vectors, or None when the
    frame has no complementary vector to align to, or no in-plane degenerate
    direction to draw.
    """
    axes = orthonormal_translation_basis(basis)
    if not axes:
        return None

    c = np.asarray(t_comp_local, dtype=float)[:2]
    comp_norm = float(np.linalg.norm(c))
    if comp_norm < 1e-6:
        return None

    # Rotation taking the complementary displacement onto +x.
    cos_a, sin_a = c / comp_norm
    R = np.array([[cos_a, sin_a], [-sin_a, cos_a]])

    direction = R @ np.asarray(axes[0], dtype=float)[:2]
    dir_norm = float(np.linalg.norm(direction))
    if dir_norm < _MIN_INPLANE_DIR:
        return None

    origin = (R @ np.asarray(t_lidar_local, dtype=float)[:2]) / comp_norm
    return origin, direction / dir_norm


def strided_lines(history, size, step):
    """The most recent ``size`` lines from ``history``, every ``step``-th.

    Newest first, which is the order the fit is least surprised by and the same
    rule the viewer's history overlay uses. Consecutive frames' lines are nearly
    identical, so the stride is what turns a fixed number of lines into a long
    span -- and span, not count, is what makes lines meet.
    """
    return list(history)[::-1][::max(1, int(step))][:max(0, int(size))]


def line_x_axis_scale(line):
    """Where one frame's own degenerate line crosses the complementary axis.

    The same normalized geometry as everything else here: the complementary
    displacement is (1, 0), and the line runs through the LiDAR's position along
    the degenerate direction. Where it crosses y = 0 it is saying "if the robot
    went straight along the odometry's own direction, it went this far per unit
    the odometry claimed" -- a scale, from one frame, with no history at all.

    This is the ratio measured on the picture instead of in the node's algebra.
    In the plane with a single degenerate direction the two agree exactly, up
    to one thing: with normal ``n`` perpendicular to the direction ``u``, the
    node computes ``|p.n| / |comp.n|`` while this is ``(p.n) / (comp.n)``. Both
    quotients are of the same two numbers; taking the norms first throws away
    the sign. So where the geometry says the robot went *backwards* relative to
    the odometry, the node reports the distance as a positive scale and this
    reports nan -- a negative scale is not a scale.

    nan also when the line is parallel to the axis and never crosses it, which
    is the frame saying nothing about how far along the odometry's direction
    the robot went.
    """
    if line is None:
        return np.nan
    origin, direction = line
    if abs(float(direction[1])) < _MIN_INPLANE_DIR:
        return np.nan
    crossing = float(origin[0]) - float(origin[1]) * float(direction[0]) / float(direction[1])
    return crossing if crossing > 0.0 else np.nan


def line_meet_point(lines, *, norm="l2"):
    """Where the recent degenerate lines put the robot, or None.

    Each line is one frame saying "the truth lies somewhere along here", drawn
    in units of that frame's own complementary displacement. Where they meet is
    therefore the robot's position from the anchor *in units of |comp|*, in the
    frame where the complementary displacement is (1, 0):

    ``x``
        how far the robot really went along the odometry's own direction, per
        unit it claimed -- which is exactly what a scale means.
    ``y``
        how far it went *sideways* of that direction. No scale can express
        this: a scale can only make the odometry's own vector longer or
        shorter. It is what a lateral odometry error looks like.

    Unlike the ratio, this does not need the LiDAR displacement to be observable
    in the direction being scaled. It needs the lines to have turned relative to
    each other, which is a property of the trajectory rather than of one frame.

    None when the lines are too parallel to meet anywhere, or when they meet
    behind the anchor: a negative along-track coordinate is not a scale, it is
    the fit telling you the lines disagree with the direction of travel.
    """
    fit = fit_lines([o for o, _ in lines], [d for _, d in lines], norm=norm)
    if fit is None or fit.point[0] <= 0.0:
        return None
    return fit.point


def line_meet_scale(lines, *, norm="l2"):
    """The along-complementary coordinate of :func:`line_meet_point`, or nan."""
    point = line_meet_point(lines, norm=norm)
    return np.nan if point is None else float(point[0])


def apply_scale_correction(t_comp, scale):
    """The node's own correction: the whole displacement, scaled.

    Direction untouched, including the vertical component -- the scale is a
    single number and cannot say anything about direction.
    """
    return np.asarray(t_comp, dtype=float) * float(scale)


def apply_similarity_correction(t_comp, scale, lateral):
    """Correct a complementary displacement with a whole meeting point.

    ``scale`` and ``lateral`` are the meeting point ``(cx, cy)``: where the
    lines put the robot, in units of |comp|, in the frame where this
    displacement is ``(d, 0)``. Carrying that back into metres gives
    ``(d*cx, d*cy)`` in the same frame, so with the displacement's own in-plane
    direction ``e_x`` and its left normal ``e_y``:

        t_corrected = d * (cx * e_x + cy * e_y)

    which is the vector to the meeting point -- the odometry's arrow moved onto
    where the lines say the robot ended up. It both stretches and *turns* the
    displacement, and ``lateral = 0`` recovers the scale-only correction in the
    plane exactly.

    z is left as it was: the fit is a 2D one and has nothing to say about it.
    A displacement with no in-plane direction to align to is returned unchanged
    for the same reason -- there is no frame to read (cx, cy) in.
    """
    t = np.array(t_comp, dtype=float)
    inplane = t[:2]
    d = float(np.linalg.norm(inplane))
    if d < _MIN_INPLANE_DIR:
        return t
    e_x = inplane / d
    e_y = np.array([-e_x[1], e_x[0]])       # +90 degrees, i.e. to the left
    t[:2] = d * (float(scale) * e_x + float(lateral) * e_y)
    return t


def _clamped(value, low, high):
    return min(max(value, low), high)


def smoothed_scale(history, mode="mean"):
    """The scale to apply, from the window of accepted samples.

    All three modes read the same trailing window, so all three are causal: the
    window holds past samples only, and the value applied at frame *k* is built
    from samples up to *k-1*.

    ``"mean"``
        The node's arithmetic mean. The samples are a ratio of two short
        displacements and their distribution is heavy-tailed -- on real runs the
        raw ratio reaches 60x -- so the mean is set by the excursions rather
        than by the bulk: one 20x sample moves a 50-wide window by 0.4 and keeps
        it moved for the next 50 frames.
    ``"median"``
        The middle sample. It cannot be dragged: a new sample moves it by at
        most one order statistic and always *towards* the side it fell on. The
        cost is that it throws away most of the window -- half the samples only
        vote on which side the middle lies -- so it is jumpier than it needs to
        be when the samples are in fact clean.
    ``"trimmed"``
        Both: locate the bulk with the quartiles, drop what falls outside a
        Tukey fence of :data:`IQR_FENCE_K` interquartile ranges, then take the
        mean of the rest. On a clean window the fence catches nothing and this
        *is* the mean; on a window with a tail it is the mean of the inliers,
        which uses far more of the data than the median while still ignoring
        the excursions. Falls back to the median when the window is too short
        for quartiles to mean anything.
    """
    if mode == "median":
        return float(np.median(history))
    if mode == "mean":
        return float(np.mean(history))
    if mode == "trimmed":
        samples = np.asarray(history, dtype=float)
        if samples.size < _MIN_TRIM_SAMPLES:
            return float(np.median(samples))
        q1, q3 = np.percentile(samples, (25.0, 75.0))
        iqr = q3 - q1
        inliers = samples[(samples >= q1 - IQR_FENCE_K * iqr)
                          & (samples <= q3 + IQR_FENCE_K * iqr)]
        # The fence always contains the quartiles themselves, so this is never
        # empty; the guard is for a window of identical values, where iqr == 0.
        return float(np.mean(inliers)) if inliers.size else float(np.median(samples))
    raise ValueError(f"Unknown scale_smoothing_mode: {mode!r}; expected one of {SMOOTHING_MODES}")


def _correction_fn(correction_mode):
    if correction_mode == "translation":
        return project_degenerate_correction_translation
    if correction_mode == "twist6":
        return project_degenerate_correction
    raise ValueError(f"Unknown correction_mode: {correction_mode!r}")


# ---------------------------------------------------------------------------
# Trajectory reconstruction
# ---------------------------------------------------------------------------

def _pin_logged_orientation(T_optimized, frame):
    """Take orientation from the logged absolute pose instead of the chain.

    ``lidar_increment`` is ``log(pose_effective[k-1]^-1 * pose_optimized[k])`` --
    referenced to the *online* corrected pose, not to whatever pose this replay
    has reached. Chaining it therefore accumulates orientation error as soon as
    the replayed trajectory departs from the online one, which is the whole point
    of replaying a different correction. Orientation is LiDAR-observable and
    logged absolutely per frame, so read it rather than integrate it; only
    position needs to accumulate for a changed scale to propagate.
    """
    T_optimized[:3, :3] = frame.pose_optimized[:3, :3]


def reconstruct_fixed(frames, scale_of_frame, apply_correction=True, translation_scale_multiplier=1.0,
                      correction_mode="twist6"):
    """Chain a trajectory from LiDAR increments using fixed/recorded scales."""
    if not frames:
        return []

    correct = _correction_fn(correction_mode)
    T_prev = frames[0].pose_prev.copy()
    out = []
    for f in frames:
        T_optimized = T_prev @ exp_map(f.lidar_increment, 1.0)
        _pin_logged_orientation(T_optimized, f)

        # Requires a complementary prediction: with none there is nothing to
        # substitute into the degenerate directions. See the note in
        # reconstruct_with_estimator on why this diverges from the node.
        do_correction = (apply_correction and f.degeneracy_detected
                         and f.has_basis and len(f.basis) > 0
                         and f.has_complementary)
        if do_correction:
            scale = scale_of_frame(f)
            dt = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
            xi_comp = f.complementary_twist.copy()
            xi_comp[:3] *= translation_scale_multiplier
            T_comp_rel = exp_map(xi_comp, dt)
            T_comp_rel[:3, 3] *= scale
            T_comp_abs = T_prev @ T_comp_rel
            T_corrected = correct(T_optimized, T_comp_abs, f.basis)
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
    scale_min=0.0,
    scale_max=float("inf"),
    dt_lidar_s=None,
    debug=False,
):
    """Port of mapOptimization::buildAdditionalOdomCorrectionResult for scaling.

    ``scale_min``/``scale_max`` bound what may enter the smoothing filter and
    are a replay-time addition -- the node has no such limit, so the defaults
    change nothing. A sample outside the range is clamped to the bound, not
    dropped; ``scale_instant_raw`` still reports it as measured.

    The observability gate is measured on the LiDAR displacement alone, which
    is an intentional divergence from the node -- see the comment at the gate.
    ``dt_lidar_s`` is the LiDAR window's own duration, defaulting to
    ``dt_complementary_s`` when a caller has only that.
    """
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

    # This frame's degenerate line in its own complementary-aligned, |comp| = 1
    # geometry -- the same one the viewer superimposes when normalized, which is
    # what makes lines from different frames comparable at all. Returned whether
    # or not anything wants it; see line_meet_scale for what it is for.
    own_line = own_normalized_line(t_lidar_latest_local, t_comp_latest_local, basis)

    # Intentional divergence from the node. It gates on
    # |t_lidar_nondeg_proj| / dt_complementary: the LiDAR displacement projected
    # onto the *complementary* non-degenerate direction, over the complementary
    # window. Both halves of that carry the complementary odometry -- the
    # projection shortens by cos(theta) between the two vectors, and the divisor
    # is the odometry's own interval -- so a frame where the LiDAR moved plenty
    # is called unobservable whenever the complementary vector is short, noisy or
    # misaligned. That inverts the gate's purpose: it asks whether enough motion
    # was *observed* to measure a ratio against, and it ends up rejecting exactly
    # the frames where the complementary odometry is what wants inspecting.
    # Replay therefore gates on the LiDAR displacement over the LiDAR window,
    # which contains no complementary quantity. The scale ratio itself is
    # unchanged, and still uses the projection.
    lidar_nondeg_norm = float(np.linalg.norm(t_lidar_nondeg))
    dt_gate = dt_complementary_s if dt_lidar_s is None else dt_lidar_s
    nondeg_speed = lidar_nondeg_norm / max(1e-5, dt_gate)
    gate_observable = (
        np.isfinite(scale_instant_raw)
        and nondeg_speed >= max(0.0, min_nondegenerate_speed)
    )

    # The bounds clamp rather than reject: an out-of-range sample still says the
    # LiDAR moved further than the odometry claimed, and dropping it would let
    # the filter keep averaging as though the frame had never happened. Clamping
    # admits the direction of the evidence while capping how far one sample can
    # pull the mean. scale_instant_raw stays as measured, so the trace and the
    # scale tab still show what the frame actually produced.
    scale_filtered = np.nan
    if gate_observable:
        scale_filtered = min(max(scale_instant_raw, scale_min), scale_max)

    result = {
        "valid": True,
        "gate_observable": bool(gate_observable),
        "scale_instant_raw": float(scale_instant_raw),
        "scale_filtered": float(scale_filtered),
        # (origin, direction) in this frame's own |comp| = 1 geometry, or None
        # when it has no drawable degenerate line to contribute.
        "own_line": own_line,
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


def reconstruct_with_estimator(frames, params, collect_vectors=False, correction_mode="twist6"):
    """Replay with online-style lagged scale estimation and application.

    Notes about parity with C++:
    - Uses same observability gate and smoothing history update rule.
    - Uses same causal ordering: applied scale at frame k comes from history
      accumulated up to frame k-1.
    - In replay, estimation is always on; only application is gated by
      ``scaleEstimationApply``.

    ``params.complementary_correction`` decides what is applied to the
    complementary displacement: the node's ratio, the along-track coordinate of
    the lines' meeting point, or that whole point -- see
    :data:`~replay_scale.core.model.COMPLEMENTARY_CORRECTIONS`. Only the last
    can turn the displacement as well as stretch it; all three go through the
    same gate, the same clamp and the same smoothing window, and all three fall
    back to the last value that window agreed on when a frame produces no
    sample.
    """
    if not frames:
        return [], [], []

    correct = _correction_fn(correction_mode)
    T_prev = frames[0].pose_prev.copy()
    out = []
    scale_trace = []
    vector_trace = []

    baseline_lag = max(1, int(params.scale_baseline_frame_lag))
    lidar_pose_buffer = deque()  # tuples: (frame_idx, stamp, T_effective)
    lagged_scale_filtered_history = deque()
    # The cross-track half of the same samples, kept in lockstep with it: the
    # pair is one observation of where the lines put the robot, and smoothing
    # the two over different windows would apply a mixture of two answers.
    lagged_lateral_filtered_history = deque()
    # Earlier frames' degenerate lines, each in its own |comp| = 1 geometry.
    # Past frames only, like every other window here: the line for frame k is
    # appended after frame k's sample is taken.
    line_history = deque()
    line_history_size = max(0, int(params.scale_line_history))
    line_history_step = max(1, int(params.scale_line_history_step))

    correction = params.complementary_correction
    uses_lines = correction in LINES_MEET_CORRECTIONS
    uses_own_line = correction == "line_x_axis"
    corrects_laterally = correction == "lines_meet_xy"
    scale_min, scale_max = float(params.scale_min), float(params.scale_max)
    lateral_max = abs(float(params.scale_lateral_max))

    n = len(frames)
    comp_step_rel = [None] * n  # index k stores rel transform from k-1 -> k.
    comp_step_dt = [np.nan] * n

    for k, f in enumerate(frames):
        basis = f.basis if (f.has_basis and len(f.basis) > 0) else []
        estimator_mode_active = bool(f.degeneracy_detected)
        scale_apply_enabled = bool(params.scale_estimation_apply)

        smoothed_scale_for_apply = np.nan
        if scale_apply_enabled and len(lagged_scale_filtered_history) > 0:
            smoothed_scale_for_apply = smoothed_scale(
                lagged_scale_filtered_history, params.scale_smoothing_mode)

        has_applied_scale = scale_apply_enabled and np.isfinite(smoothed_scale_for_apply)
        scale_applied = float(smoothed_scale_for_apply) if has_applied_scale else 1.0

        # Zero is this one's identity, as 1 is the scale's: no sideways
        # correction at all. Held from the same window, so a stretch where the
        # lines say nothing keeps applying the last pair they did agree on.
        smoothed_lateral_for_apply = np.nan
        if corrects_laterally and scale_apply_enabled and len(lagged_lateral_filtered_history) > 0:
            smoothed_lateral_for_apply = smoothed_scale(
                lagged_lateral_filtered_history, params.scale_smoothing_mode)
        lateral_applied = (float(smoothed_lateral_for_apply)
                           if np.isfinite(smoothed_lateral_for_apply) else 0.0)

        T_optimized = T_prev @ exp_map(f.lidar_increment, 1.0)
        _pin_logged_orientation(T_optimized, f)
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
            # Applied to this one frame's step in its own frame, not to the
            # window the pair was measured over. That is what the overlay the
            # pair comes from already assumes: every frame is normalized by its
            # own |comp| and turned so its own odometry is +x before the lines
            # are superimposed, which only means anything if the error is a
            # fixed multiple of |comp| in a fixed direction relative to the
            # odometry -- i.e. a per-step, body-frame quantity. It is also the
            # form apply_complementary_drift injects an error in.
            if corrects_laterally:
                T_comp_rel_scaled[:3, 3] = apply_similarity_correction(
                    T_comp_rel_unscaled[:3, 3], scale_applied, lateral_applied)
            else:
                T_comp_rel_scaled[:3, 3] = apply_scale_correction(
                    T_comp_rel_unscaled[:3, 3], scale_applied)

            T_comp_scaled_abs = T_prev @ T_comp_rel_scaled

            comp_step_rel[k] = T_comp_rel_unscaled
            comp_step_dt[k] = dt_pred

        # The override also requires a complementary prediction. The node leaves
        # its prediction at T_prev when the match fails and still projects it,
        # which substitutes *zero motion* along the degenerate axis and makes the
        # trajectory slide sideways. That is unreachable online (its own odometry
        # matches every frame) but routine when replaying a source with gaps.
        apply_override = estimator_mode_active and len(basis) > 0 and has_comp
        T_corrected = (correct(T_optimized, T_comp_scaled_abs, basis)
                       if apply_override else T_optimized)

        T_effective = T_corrected
        out.append((f.time, T_effective))

        lidar_pose_buffer.append((k, f.time, T_effective.copy()))
        while len(lidar_pose_buffer) > baseline_lag + 1:
            lidar_pose_buffer.popleft()

        gate_observable = False
        scale_instant_raw = np.nan
        scale_filtered = np.nan
        lateral_instant_raw = np.nan
        lateral_filtered = np.nan
        meet_point = None
        debug_vecs = None

        # Port of lagged scale update path: if buffer has anchor/current pair,
        # estimate lagged scale sample and append to filtered history when observable.
        if len(lidar_pose_buffer) >= 2:
            anchor_idx, anchor_stamp, T_anchor = lidar_pose_buffer[0]
            latest_idx, latest_stamp, T_latest = lidar_pose_buffer[-1]
            # The window the LiDAR displacement actually spans, which is what the
            # observability gate divides by. Distinct from dt_comp_window, whose
            # length depends on how the odometry matched.
            dt_lidar_window = float(latest_stamp - anchor_stamp)

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
                    scale_min=scale_min,
                    scale_max=scale_max,
                    dt_lidar_s=dt_lidar_window,
                    debug=collect_vectors,
                )
                gate_observable = sample["gate_observable"]
                scale_instant_raw = sample["scale_instant_raw"]
                scale_filtered = sample["scale_filtered"]

                own_line = sample["own_line"]
                # Where this frame's line and the recent ones agree. Fitted
                # whenever there is a history to fit through, whatever the
                # correction is: the viewer draws its trail as a diagnostic
                # even while the ratio is what gets applied. scaleLineHistory: 0
                # is what turns it off.
                if line_history_size:
                    past = strided_lines(line_history, line_history_size,
                                         line_history_step)
                    lines = ([own_line] if own_line else []) + past
                    meet_point = line_meet_point(
                        lines, norm=params.scale_line_fit_norm)

                if uses_lines or uses_own_line:
                    # The ratio computed above is replaced, not blended: they
                    # are two answers to the same question and mixing them would
                    # hide which one is speaking.
                    if uses_own_line:
                        scale_instant_raw = line_x_axis_scale(own_line)
                    else:
                        scale_instant_raw = (float(meet_point[0])
                                             if meet_point is not None else np.nan)
                    # A line that met nowhere -- or never crossed the axis -- is
                    # no sample at all; saying it was observable would make the
                    # coverage strip lie. Nothing is appended, so the filter
                    # keeps applying what the last frames that did answer said.
                    gate_observable = gate_observable and np.isfinite(scale_instant_raw)
                    scale_filtered = (_clamped(scale_instant_raw, scale_min, scale_max)
                                      if gate_observable else np.nan)
                    if corrects_laterally and meet_point is not None:
                        lateral_instant_raw = float(meet_point[1])
                        if gate_observable:
                            lateral_filtered = _clamped(
                                lateral_instant_raw, -lateral_max, lateral_max)

                if own_line is not None and line_history_size:
                    line_history.append(own_line)
                    # Deep enough that the stride can reach back the full span.
                    while len(line_history) > line_history_size * line_history_step:
                        line_history.popleft()
                if collect_vectors:
                    debug_vecs = sample.get("debug")

                if np.isfinite(scale_filtered):
                    lagged_scale_filtered_history.append(float(scale_filtered))
                    if corrects_laterally:
                        lagged_lateral_filtered_history.append(float(lateral_filtered))
                window = max(1, int(params.scale_smoothing_window_size))
                while len(lagged_scale_filtered_history) > window:
                    lagged_scale_filtered_history.popleft()
                while len(lagged_lateral_filtered_history) > window:
                    lagged_lateral_filtered_history.popleft()

        scale_smooth = (smoothed_scale(lagged_scale_filtered_history,
                                        params.scale_smoothing_mode)
                        if lagged_scale_filtered_history else np.nan)
        lateral_smooth = (smoothed_scale(lagged_lateral_filtered_history,
                                         params.scale_smoothing_mode)
                          if lagged_lateral_filtered_history else np.nan)
        scale_trace.append(ScaleEstimateFrame(
            frame_idx=k,
            time=float(f.time),
            gate_observable=bool(gate_observable),
            scale_instant_raw=float(scale_instant_raw),
            scale_filtered=float(scale_filtered),
            scale_smooth=float(scale_smooth),
            scale_applied=float(scale_applied),
            lateral_instant_raw=float(lateral_instant_raw),
            lateral_filtered=float(lateral_filtered),
            lateral_smooth=float(lateral_smooth),
            # NaN rather than 0 when nothing is being corrected sideways: 0 is a
            # measurement ("the lines say straight ahead"), absence is not.
            lateral_applied=(float(lateral_applied) if corrects_laterally else np.nan),
        ))
        if collect_vectors:
            _nan3 = np.full(3, np.nan)
            vf = ScaleVectorFrame()
            vf.frame_idx = k
            vf.time = float(f.time)
            vf.degeneracy_detected = bool(estimator_mode_active)
            vf.gate_observable = bool(gate_observable)
            # Mirrors the trimming of lidar_pose_buffer above, which holds
            # frames max(0, k - baseline_lag)..k. Recorded unconditionally: the
            # anchor *pose* is well defined even on frames where the
            # complementary window failed to close and the vectors below are nan.
            vf.anchor_frame_idx = max(0, k - baseline_lag)
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
            vf.meet_point = (np.array(meet_point, dtype=float)
                             if meet_point is not None else np.full(2, np.nan))
            vf.scale_instant_raw = float(scale_instant_raw)
            vf.scale_smooth = float(scale_smooth)
            vf.scale_applied = float(scale_applied)
            vf.lateral_instant_raw = float(lateral_instant_raw)
            vf.lateral_smooth = float(lateral_smooth)
            vf.lateral_applied = (float(lateral_applied) if corrects_laterally else np.nan)
            vector_trace.append(vf)

        T_prev = T_effective

    return out, scale_trace, vector_trace
