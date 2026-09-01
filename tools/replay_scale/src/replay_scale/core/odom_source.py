"""Re-synchronization of an alternative complementary odometry source.

Port of ``mapOptimization::inferComplementaryOdomTwist``, so that an odometry
stream other than the one used online can be replayed against the same LiDAR
frames.

The online node bakes its match into ``scale_replay_frames.csv`` as a finished
velocity twist. To swap sources, replay redoes the same steps here:
  1. obtain the source pose at each of the two LiDAR frame stamps,
  2. take the relative motion between the two,
  3. conjugate it into the LiDAR frame through the extrinsic,
  4. divide by the interval between them to get a velocity twist.

Step 1 has two modes, ``MATCH_MODES``:

``"nearest"``
    What the node does: pick the sample closest to each stamp and reject the
    match if it is farther than ``max_match_dt_s``. The twist is then measured
    over the *odom pair's* own interval, not the LiDAR one.

``"interpolate"``
    Evaluate the source pose at the LiDAR stamp itself, between the two samples
    bracketing it. This is for a source slower than the LiDAR: nearest matching
    quantizes both endpoints onto the sparse grid, so the window a twist is
    measured over is off by up to a sample period at each end, and a sample
    period of jitter lands directly in a displacement the scale estimate is a
    ratio of. Interpolating removes that: the interval is exactly the LiDAR one.
    It cannot invent motion the source did not observe -- it assumes constant
    twist across the gap, so a source too sparse to resolve the real motion
    yields a smoothed one, which is why the gap is gated.

Not a port of the node in ``"interpolate"``: it is a replay-time improvement on
a stream the node had to consume live.
"""

import numpy as np

from .se3 import BodyFrame, exp_map, matrix_to_twist

# Gates from mapOptimization_degeneracy.cpp.
_MIN_DT_COMPLEMENTARY_S = 1e-3

#: How a source pose is obtained at a LiDAR stamp. See the module docstring.
MATCH_MODES = ("nearest", "interpolate")


def _nearest_index(stamps, target, exclude_idx=-1):
    """Closest sample to ``target``; returns (index, abs_dt), or (-1, inf).

    The node scans its whole odometry queue linearly. Streams here are far
    larger, so this binary-searches instead: on sorted stamps the nearest (and
    the next-nearest, once one index is excluded) can only be adjacent to the
    insertion point. Ties keep the lowest index, matching the node's strict
    ``<`` comparison over an ascending scan.
    """
    n = len(stamps)
    if n == 0:
        return -1, np.inf

    pos = int(np.searchsorted(stamps, target))
    best_idx = -1
    best_abs_dt = np.inf
    for i in range(max(0, pos - 2), min(n, pos + 2)):
        if i == exclude_idx:
            continue
        abs_dt = abs(stamps[i] - target)
        if abs_dt < best_abs_dt:
            best_abs_dt = abs_dt
            best_idx = i
    return best_idx, best_abs_dt


def _interpolated_pose(stamps, poses, target, max_gap_s):
    """Source pose at ``target``, between the samples bracketing it.

    Returns None when ``target`` falls outside the stream -- extrapolating
    would be inventing motion -- or when the bracketing samples are more than
    ``max_gap_s`` apart, which is the gate on how much of the trajectory the
    constant-twist assumption is allowed to stand in for.

    Interpolation is along the constant twist connecting the two samples (the
    SE(3) geodesic), i.e. the same motion model the reconstruction integrates,
    rather than a straight line for position with the rotation handled apart
    from it.
    """
    if target < stamps[0] or target > stamps[-1]:
        return None
    # searchsorted is 'left', so a target equal to a stamp lands on that index;
    # clamping to >= 1 makes the very first stamp bracket as [0, 1] with u = 0.
    hi = max(1, int(np.searchsorted(stamps, target)))
    lo = hi - 1

    gap = float(stamps[hi] - stamps[lo])
    if gap <= 0.0 or gap > max_gap_s:
        return None

    T_lo, T_hi = poses[lo], poses[hi]
    xi = matrix_to_twist(np.linalg.inv(T_lo) @ T_hi, 1.0)
    return T_lo @ exp_map(xi, (target - stamps[lo]) / gap)


def sync_odom_to_frames(odom_stream, frames, T_comp_to_lidar, max_match_dt_s=0.25,
                        match_mode="nearest"):
    """Match an odometry stream to LiDAR frames and build per-frame twists.

    ``odom_stream`` is a list of (stamp, 4x4 pose) as returned by ``load_tum``.
    Returns a list aligned 1:1 with ``frames`` of (twist6, dt_s) pairs, using
    (None, nan) for frames whose match failed a gate.

    ``match_mode`` is one of :data:`MATCH_MODES`; ``max_match_dt_s`` gates both,
    as the largest tolerated distance from a stamp to its sample in
    ``"nearest"`` and as the widest gap that may be interpolated across in
    ``"interpolate"``.

    The twist is left unscaled: ``translationScale`` stays a replay-time knob
    applied downstream by the reconstruction, exactly as it is for the twist
    baked into the CSV. Applying it here too would square it.
    """
    if match_mode not in MATCH_MODES:
        raise ValueError(f"match_mode must be one of {MATCH_MODES}, got {match_mode!r}")

    results = []
    if len(odom_stream) < 2:
        return [(None, np.nan)] * len(frames)

    # Binary search below requires ascending stamps; a stream read from file is
    # not guaranteed to be ordered.
    odom_stream = sorted(odom_stream, key=lambda item: item[0])
    stamps = np.array([s for s, _ in odom_stream], dtype=float)
    poses = [T for _, T in odom_stream]
    T_ext = np.asarray(T_comp_to_lidar, dtype=float)
    T_ext_inv = np.linalg.inv(T_ext)

    for f in frames:
        # The online run matched against these two targets; both are logged.
        prev_target = f.lidar_prev_stamp
        curr_target = f.time
        if not np.isfinite(prev_target):
            prev_target = curr_target - f.dt_scan

        if match_mode == "interpolate":
            T1 = _interpolated_pose(stamps, poses, prev_target, max_match_dt_s)
            T2 = _interpolated_pose(stamps, poses, curr_target, max_match_dt_s)
            if T1 is None or T2 is None:
                results.append((None, np.nan))
                continue
            # The LiDAR interval itself, because that is what the poses are at.
            dt_complementary = float(curr_target - prev_target)
        else:
            # Mirrors the C++ search: the second lookup excludes the first index,
            # so two LiDAR stamps landing on one sample take the second-closest.
            idx_prev, prev_abs_dt = _nearest_index(stamps, prev_target)
            idx_curr, curr_abs_dt = _nearest_index(stamps, curr_target, exclude_idx=idx_prev)
            if idx_prev < 0 or idx_curr < 0:
                results.append((None, np.nan))
                continue

            if prev_abs_dt > max_match_dt_s or curr_abs_dt > max_match_dt_s:
                results.append((None, np.nan))
                continue

            idx1, idx2 = (idx_prev, idx_curr)
            if stamps[idx1] > stamps[idx2]:
                idx1, idx2 = idx2, idx1

            # The odom pair's own interval, which is what the motion spans.
            dt_complementary = float(stamps[idx2] - stamps[idx1])
            T1, T2 = poses[idx1], poses[idx2]

        if dt_complementary < _MIN_DT_COMPLEMENTARY_S:
            results.append((None, np.nan))
            continue

        T_add_delta = np.linalg.inv(T1) @ T2
        T_lidar_delta = T_ext @ T_add_delta @ T_ext_inv
        results.append((matrix_to_twist(T_lidar_delta, dt_complementary), dt_complementary))

    return results


#: Body-frame axes a simulated drift can be injected along.
DRIFT_AXES = ("x", "y", "z")


def apply_complementary_drift(frames, alpha, axis="y", body_frame=None):
    """Inject a distance-proportional error into the complementary odometry.

    Each frame's complementary displacement gains ``alpha * ||displacement||``
    along the given body axis, which is how odometry error on a legged or
    wheeled platform actually accumulates -- proportional to ground covered,
    not constant per frame and not white noise.

    Applied to the raw twist, i.e. *before* ``complementaryOdom.translationScale``
    and before any estimated scale, so the injected error is a property of the
    simulated sensor rather than of the correction being tested.

    The axis decides what is being tested. Lateral (``y``, the default) is an
    error the scale cannot express, and it lands squarely on the non-degenerate
    component the estimate is *computed from* -- so it does not stay lateral, it
    comes back out as a longitudinal scale error. Along travel (``x``) is a pure
    scale error the estimator should be able to recover.

    ``body_frame`` is which body's axes those are, and which displacement the
    distance is measured on: the same frame the estimator works in, so that
    "we put in alpha along x, did we get it back" stays an exact question. It
    matters as soon as the extrinsic turns -- the ANYmal mount is a 180 degree
    yaw, so a lateral drift injected in the LiDAR frame and one injected at the
    base point opposite ways. Defaults to the LiDAR frame, which is where this
    injected before there was anywhere else to inject.

    Frames mutate in place; returns the count modified.
    """
    if axis not in DRIFT_AXES:
        raise ValueError(f"drift axis must be one of {DRIFT_AXES}, got {axis!r}")
    if not alpha:
        return 0

    body_frame = body_frame if body_frame is not None else BodyFrame()
    component = DRIFT_AXES.index(axis)
    modified = 0
    for f in frames:
        if not f.has_complementary or not np.all(np.isfinite(f.complementary_twist)):
            continue
        # Same dt fallback the reconstruction uses, so the injected displacement
        # is the one that actually gets integrated.
        dt = f.dt_complementary if f.dt_complementary > 1e-5 else f.dt_scan
        if not np.isfinite(dt) or dt <= 0.0:
            continue

        twist = body_frame.twist(f.complementary_twist)
        distance = float(np.linalg.norm(twist[:3] * dt))
        if distance <= 0.0:
            continue

        twist[component] += alpha * distance / dt
        f.complementary_twist = body_frame.untwist(twist)
        modified += 1
    return modified


def apply_odom_source(frames, synced):
    """Overwrite each frame's complementary fields with a re-synced source.

    Everything downstream (reconstruct_fixed / reconstruct_with_estimator /
    reconstruct_complementary_only) reads only these three fields, so no
    estimator changes are needed. Returns the number of matched frames.
    """
    matched = 0
    for f, (twist, dt) in zip(frames, synced):
        if twist is None or not np.all(np.isfinite(twist)):
            f.has_complementary = False
            f.complementary_twist = np.full(6, np.nan)
            f.dt_complementary = np.nan
            continue
        f.has_complementary = True
        f.complementary_twist = twist
        f.dt_complementary = dt
        matched += 1
    return matched
