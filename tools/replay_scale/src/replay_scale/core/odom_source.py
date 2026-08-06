"""Re-synchronization of an alternative complementary odometry source.

Port of ``mapOptimization::inferComplementaryOdomTwist`` (nearest-sample
matching, not interpolation), so that an odometry stream other than the one
used online can be replayed against the same LiDAR frames.

The online node bakes its match into ``scale_replay_frames.csv`` as a finished
velocity twist. To swap sources, replay redoes the same steps here:
  1. pick the odom sample nearest each of the two LiDAR frame stamps,
  2. reject matches farther than ``max_match_dt_s``,
  3. take the relative motion between the two picked samples,
  4. conjugate it into the LiDAR frame through the extrinsic,
  5. divide by the *odom pair* interval to get a velocity twist.
"""

import numpy as np

from .se3 import matrix_to_twist

# Gates from mapOptimization_degeneracy.cpp.
_MIN_DT_COMPLEMENTARY_S = 1e-3


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


def sync_odom_to_frames(odom_stream, frames, T_comp_to_lidar, max_match_dt_s=0.25):
    """Match an odometry stream to LiDAR frames and build per-frame twists.

    ``odom_stream`` is a list of (stamp, 4x4 pose) as returned by ``load_tum``.
    Returns a list aligned 1:1 with ``frames`` of (twist6, dt_s) pairs, using
    (None, nan) for frames whose match failed a gate.

    The twist is left unscaled: ``translationScale`` stays a replay-time knob
    applied downstream by the reconstruction, exactly as it is for the twist
    baked into the CSV. Applying it here too would square it.
    """
    results = []
    if len(odom_stream) < 2:
        return [(None, np.nan)] * len(frames)

    # Binary search below requires ascending stamps; a stream read from file is
    # not guaranteed to be ordered.
    odom_stream = sorted(odom_stream, key=lambda item: item[0])
    stamps = np.array([s for s, _ in odom_stream], dtype=float)
    T_ext = np.asarray(T_comp_to_lidar, dtype=float)
    T_ext_inv = np.linalg.inv(T_ext)

    for f in frames:
        # The online run matched against these two targets; both are logged.
        prev_target = f.lidar_prev_stamp
        curr_target = f.time
        if not np.isfinite(prev_target):
            prev_target = curr_target - f.dt_scan

        # Mirrors the C++ search: the second lookup excludes the first index, so
        # two LiDAR stamps landing on one sample take the second-closest.
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

        dt_complementary = stamps[idx2] - stamps[idx1]
        if dt_complementary < _MIN_DT_COMPLEMENTARY_S:
            results.append((None, np.nan))
            continue

        T1 = odom_stream[idx1][1]
        T2 = odom_stream[idx2][1]
        T_add_delta = np.linalg.inv(T1) @ T2
        T_lidar_delta = T_ext @ T_add_delta @ T_ext_inv
        results.append((matrix_to_twist(T_lidar_delta, dt_complementary), dt_complementary))

    return results


#: Body-frame axes a simulated drift can be injected along.
DRIFT_AXES = ("x", "y", "z")


def apply_complementary_drift(frames, alpha, axis="y"):
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

    Frames mutate in place; returns the count modified.
    """
    if axis not in DRIFT_AXES:
        raise ValueError(f"drift axis must be one of {DRIFT_AXES}, got {axis!r}")
    if not alpha:
        return 0

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

        distance = float(np.linalg.norm(f.complementary_twist[:3] * dt))
        if distance <= 0.0:
            continue

        twist = f.complementary_twist.copy()
        twist[component] += alpha * distance / dt
        f.complementary_twist = twist
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
