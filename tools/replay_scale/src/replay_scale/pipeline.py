"""The replay itself: CSV in, trajectories out.

This is the seam every frontend goes through. :func:`run_replay` performs a
complete replay and returns everything it computed in memory; it never prints
and, with ``write=False``, never touches the filesystem either. The CLI is a
thin argparse wrapper around it, and a GUI is expected to be the same -- see
``README.md`` for why frontends call this rather than shelling out to the CLI.
"""

import os
from dataclasses import dataclass, field

import numpy as np

from .core.estimator import reconstruct_fixed, reconstruct_with_estimator
from .core.odom_source import apply_complementary_drift, apply_odom_source, sync_odom_to_frames
from .io.paths import (
    complementary_source_tag,
    expand_path,
    resolve_output_dirs,
)
from .io.frames_csv import load_frames
from .io.traces_csv import write_scale_trace_csv, write_scale_vector_csv
from .io.tum import load_tum, write_tum
from .settings import (
    META_NAME,
    estimation_body_frame,
    load_complementary_odom_meta,
)


@dataclass
class ValidationReport:
    """Replay-with-recorded-scale vs. the pose the online run actually used."""

    trajectory: list
    drift: np.ndarray

    @property
    def mean(self):
        return float(self.drift.mean()) if self.drift.size else float("nan")

    @property
    def max(self):
        return float(self.drift.max()) if self.drift.size else float("nan")

    @property
    def rms(self):
        return float((self.drift ** 2).mean()) ** 0.5 if self.drift.size else float("nan")


@dataclass
class ReplayResult:
    """Everything one replay produced, in memory.

    ``written`` lists the files actually created; it stays empty when
    :func:`run_replay` is called with ``write=False``.
    """

    csv_path: str
    settings: object
    params: object
    frames: list
    out_dir: str
    traj_dir: str
    log_dir: str
    #: Which odometry was corrected and the mount that placed it; see
    #: :class:`ActiveOdometry`.
    active_odometry: object = None
    #: The body frame the correction was measured in -- which a frontend needs
    #: to draw the auxiliary curves in the same frame the replay used.
    body_frame: object = None

    @property
    def extrinsic(self):
        """T_complementary_to_lidar of the odometry that was corrected."""
        return None if self.active_odometry is None else self.active_odometry.extrinsic

    @property
    def extrinsic_origin(self):
        return "" if self.active_odometry is None else self.active_odometry.origin
    source_status: str = None
    drift_status: str = None
    recorded_effective: list = field(default_factory=list)
    replays: list = field(default_factory=list)       # [(tag, trajectory)]
    lidar_only: list = None
    validation: ValidationReport = None
    scale_trace: list = field(default_factory=list)
    vector_trace: list = field(default_factory=list)
    written: list = field(default_factory=list)
    #: (label, trajectory) of the external reference, or None when none is set.
    reference: tuple = None
    reference_status: str = None

    @property
    def n_frames(self):
        return len(self.frames)

    @property
    def n_degeneracy_override(self):
        return sum(1 for f in self.frames if f.degeneracy_detected and f.has_basis)

    def curves(self):
        """(label, trajectory, kind) triples for :mod:`replay_scale.plotting`."""
        out = [("recorded effective", self.recorded_effective, "reference")]
        if self.reference is not None:
            # First, so it reads as the thing the rest is being judged against
            # rather than as one more replay.
            out.insert(0, (self.reference[0], self.reference[1], "external"))
        out += [(tag.replace("_", " "), traj, "replay") for tag, traj in self.replays]
        if self.validation is not None:
            out.append(("validate", self.validation.trajectory, "replay"))
        if self.lidar_only is not None:
            out.append(("lidar only", self.lidar_only, "replay"))
        return out


# ---------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------

def recorded_effective_trajectory(frames):
    return [(f.time, f.pose_effective) for f in frames]


def position_drift(traj_a, traj_b):
    """Per-pose Euclidean position difference (assumes aligned frame order)."""
    n = min(len(traj_a), len(traj_b))
    diffs = np.array([np.linalg.norm(traj_a[i][1][:3, 3] - traj_b[i][1][:3, 3])
                      for i in range(n)], dtype=float)
    return diffs


@dataclass
class ActiveOdometry:
    """Which complementary odometry this replay is correcting, and its mount.

    There are two odometries in play and they are different sensors: the one the
    node recorded into the CSV, and an alternative stream that replaces it. Each
    has its own mount, and neither may stand in for the other -- a mount
    inherited from the wrong sensor is a silent wrong answer, not a missing one,
    because it comes back out as a scale.

    So the choice is made once, here, and everything downstream -- the
    re-synchronization, the estimation frame, the injected drift, the status
    lines -- reads these fields rather than repeating the rule. A third source
    would be a branch in :func:`resolve_active_odometry` and nothing else.
    """

    #: "recorded" or "source".
    kind: str
    #: Human-readable name of the odometry in play.
    label: str
    #: T_complementary_to_lidar for *that* odometry.
    extrinsic: object
    #: Where the mount came from, for the status line.
    origin: str
    #: Configuration that was present but does not apply, if any.
    note: str = ""

    @property
    def is_source(self):
        return self.kind == "source"


def resolve_active_odometry(settings, csv_path):
    """Which odometry is being corrected, and the mount that belongs to it.

    Resolved for every replay, not only for one that swaps the source: the
    estimation frame is placed by this mount, so a replay of the recorded twist
    needs it just as much.
    """
    source = settings.complementary_source
    if source.path:
        label = os.path.basename(expand_path(source.path))
        extrinsic = source.extrinsic
        if extrinsic is None:
            raise ValueError(
                f"complementary_source.path is set ({source.path}) but no mount is: "
                "write complementary_source.extrinsicTrans/extrinsicRot for the "
                "stream's own mount, 'extrinsic: run' if it is the sensor the run "
                "recorded (complementary_odom_stream.tum is), or 'extrinsic: identity' "
                "if it is already in the LiDAR frame. It is not inherited from the "
                "recorded odometry: that is a different sensor, and a wrong mount does "
                "not fail, it comes back out as a scale.")
        # isinstance first: a mount is an array, and `array == "run"` is neither
        # True nor False but an array, which no `if` can read.
        if isinstance(extrinsic, str) and extrinsic == "identity":
            return ActiveOdometry("source", label, np.eye(4, dtype=float),
                                  "identity (configured)")
        if isinstance(extrinsic, str) and extrinsic == "run":
            from_run = load_complementary_odom_meta(csv_path)
            if from_run is None:
                raise ValueError(
                    "complementary_source.extrinsic is 'run', but this run recorded no "
                    f"{META_NAME} to take a mount from.")
            return ActiveOdometry("source", label, from_run,
                                  f"run {META_NAME} (shared with the recorded odometry)")
        return ActiveOdometry("source", label, np.asarray(extrinsic, dtype=float),
                              "complementary_source.extrinsic")

    # The recorded twist is what is being corrected, so the recorded sensor's
    # mount is the one that applies. Anything configured for a source that is
    # not being replayed is inert, and says so rather than leaking in.
    note = ""
    if source.extrinsic is not None:
        note = ("complementary_source.extrinsic is set but no source path is: it "
                "describes a stream this replay does not read, and is ignored. Put a "
                "mount for the recorded odometry under recorded_odometry.")
    configured = settings.recorded_odometry.extrinsic
    if configured is not None:
        return ActiveOdometry("recorded", "recorded odometry",
                              np.asarray(configured, dtype=float),
                              "recorded_odometry.extrinsic", note)
    from_run = load_complementary_odom_meta(csv_path)
    if from_run is not None:
        return ActiveOdometry("recorded", "recorded odometry", from_run,
                              f"run {META_NAME}", note)
    return ActiveOdometry("recorded", "recorded odometry", np.eye(4, dtype=float),
                          f"identity (no recorded_odometry.extrinsic and no run "
                          f"{META_NAME})", note)


def describe_body_frame(body_frame, params, active):
    """One line saying which frame the correction was measured and applied in.

    Worth printing on every run: a 0.35 m mount silently becoming the identity
    -- a missing meta file, a mount that dropped its translation -- does not
    fail, it comes back out as a scale.
    """
    if body_frame is None or body_frame.is_identity:
        reason = ("as configured" if params is None
                  or not params.estimates_in_complementary_frame
                  else f"mount is identity, from {active.origin}")
        return f"estimation frame: lidar ({reason})"
    t = body_frame.offset
    axes = ("odometry axes (extrinsic rotation adopted)"
            if params.estimation_frame_use_extrinsic_rot else "LiDAR axes")
    return (f"estimation frame: complementary — {active.label}, "
            f"mount from {active.origin}\n"
            f"    origin offset: [{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}] m; {axes}")


def apply_complementary_source(frames, settings, csv_path, active=None):
    """Re-synchronize an alternative complementary odometry source onto frames.

    No-op returning None when no source path is configured, in which case the
    twist recorded by the online run is kept. Otherwise the frames are mutated
    in place and a short human-readable status is returned.

    ``active`` is the resolved :class:`ActiveOdometry`; a caller that has one
    passes it rather than resolving the mount a second time.
    """
    source = settings.complementary_source
    if not source.path:
        return None

    stream_path = expand_path(source.path)
    if not os.path.isfile(stream_path):
        raise FileNotFoundError(f"Complementary odometry source not found: {stream_path}")

    if active is None:
        active = resolve_active_odometry(settings, csv_path)

    odom_stream = load_tum(stream_path)
    synced = sync_odom_to_frames(odom_stream, frames, active.extrinsic,
                                 max_match_dt_s=source.max_match_dt_s,
                                 match_mode=source.match_mode)
    matched = apply_odom_source(frames, synced)
    gate = ("max interpolated gap" if source.match_mode == "interpolate"
            else "max match dt")
    return (f"complementary source: {stream_path}\n"
            f"    mount: {active.origin}\n"
            f"    match mode: {source.match_mode} ({gate}: {source.max_match_dt_s} s)\n"
            f"    stream samples: {len(odom_stream)}\n"
            f"    matched frames: {matched}/{len(frames)}")


def _nearest_stamp_index(stamps, target):
    pos = int(np.searchsorted(stamps, target))
    candidates = [i for i in (pos - 1, pos) if 0 <= i < len(stamps)]
    return min(candidates, key=lambda i: abs(stamps[i] - target))


def _sampled_positions(trajectory, stamps):
    """Replay positions at the given stamps, linearly interpolated.

    Returns ``(positions, mask)``, the mask marking which stamps fell inside the
    replay's span. Interpolating rather than taking the nearest pose: the replay
    is dense and continuous, and a reference sample landing between two of its
    poses should be compared with where the robot was *then*, not with whichever
    LiDAR frame happened to be closest. Extrapolation is refused -- a reference
    that runs past the end of the run says nothing about it.
    """
    replay_stamps = np.array([t for t, _ in trajectory], dtype=float)
    replay_xyz = np.array([T[:3, 3] for _, T in trajectory], dtype=float)
    stamps = np.asarray(stamps, dtype=float)
    mask = (stamps >= replay_stamps[0]) & (stamps <= replay_stamps[-1])
    positions = np.full((stamps.size, 3), np.nan)
    for axis in range(3):
        positions[mask, axis] = np.interp(stamps[mask], replay_stamps, replay_xyz[:, axis])
    return positions, mask


def _fit_rotation(src, dst, yaw_only):
    """The rotation carrying ``src`` onto ``dst`` in the least-squares sense.

    Both are (n, 3) offsets from their own anchor, so there is no translation
    left to fit and no centroid to remove -- the anchor already fixes where the
    two trajectories are pinned together, and re-centring would unpin it.

    ``yaw_only`` restricts the fit to a turn about the vertical, which has a
    closed form. That is the honest choice for two gravity-levelled frames: the
    heading between them is genuinely unknown, while roll and pitch are not, and
    fitting all three would quietly absorb a real tilt error into the alignment.
    """
    if yaw_only:
        # argmin over theta of |Rz(theta) a - b|^2.
        cross = float(np.sum(src[:, 0] * dst[:, 1] - src[:, 1] * dst[:, 0]))
        dot = float(np.sum(src[:, 0] * dst[:, 0] + src[:, 1] * dst[:, 1]))
        theta = float(np.arctan2(cross, dot))
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]), theta

    # Kabsch. The reflection guard matters: without it a poorly conditioned
    # fit can return a mirrored "rotation", which would draw a reference that
    # is not the shape that was surveyed.
    U, _, Vt = np.linalg.svd(src.T @ dst)
    d = float(np.sign(np.linalg.det(Vt.T @ U.T)))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def load_reference_trajectory(settings, frames, fit_target=None):
    """The external trajectory to draw alongside the replays, or None.

    Returns ``((label, trajectory), status)``, or ``(None, None)`` when no path
    is configured. Nothing the replay computes depends on this: it is drawn, not
    measured against, so a reference in the wrong frame can mislead the eye but
    cannot move an estimate.

    ``fit_target`` is the replayed trajectory the fitted alignments turn the
    reference onto; the caller passes the replay it wants compared. Every
    alignment is rigid and none fits a scale -- a scale is the quantity under
    test, and a fitted one would absorb exactly the error being looked for.
    See :data:`~replay_scale.core.model.REFERENCE_ALIGNMENTS`.
    """
    reference = settings.reference_trajectory
    if not reference.path:
        return None, None

    path = expand_path(reference.path)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Reference trajectory not found: {path}")

    trajectory = load_tum(path)
    if not trajectory:
        raise ValueError(f"Reference trajectory holds no poses: {path}")

    offset = float(reference.time_offset_s)
    if offset:
        trajectory = [(stamp + offset, T) for stamp, T in trajectory]

    label = reference.label or os.path.splitext(os.path.basename(path))[0].replace("_", " ")
    status = [f"reference trajectory: {path}",
              f"    poses: {len(trajectory)}"
              + (f"  (stamps shifted by {offset:+.3f} s)" if offset else "")]

    align = reference.align
    if align == "none" or not frames:
        status.append("    aligned: not moved (align: none)")
        return (label, trajectory), "\n".join(status)

    stamps = np.array([stamp for stamp, _ in trajectory], dtype=float)
    t0 = float(frames[0].time)
    overlaps = stamps[0] <= t0 <= stamps[-1]

    if align == "first_pose":
        # Disjoint clocks are the normal case for a reference logged
        # separately, and "nearest" would then pick whichever end happens to be
        # closer -- the last pose, for a reference whose stamps start lower.
        idx = _nearest_stamp_index(stamps, t0) if overlaps else 0
        how = (f"its pose at the replay's first stamp (index {idx})" if overlaps
               else "its own first pose (stamps do not overlap the run's)")
        T_fix = frames[0].pose_prev @ np.linalg.inv(trajectory[idx][1])
        trajectory = [(stamp, T_fix @ T) for stamp, T in trajectory]
        status.append(f"    aligned: {how} moved onto the replay's first pose")
        return (label, trajectory), "\n".join(status)

    # -- the fitted alignments ---------------------------------------------
    yaw_only = align == "first_position_yaw"
    fitted = "yaw" if yaw_only else "full rotation"

    if not fit_target:
        status.append(f"    aligned: first position only -- no replayed "
                      f"trajectory to fit the {fitted} against")
        fit_target = [(f.time, f.pose_prev) for f in frames[:1]]

    sampled, paired = _sampled_positions(fit_target, stamps)
    reference_xyz = np.array([T[:3, 3] for _, T in trajectory], dtype=float)

    if int(paired.sum()) < 2:
        # Nothing to fit against, and inventing correspondences -- stretching
        # the reference's clock onto the run's -- would be assuming the very
        # thing being measured. Anchor the position and say what would fix it.
        anchor_idx = _nearest_stamp_index(stamps, t0) if overlaps else 0
        R = np.eye(3)
        status.append(f"    aligned: first position only -- {int(paired.sum())} of "
                      f"{len(trajectory)} samples fall inside the run's span, too "
                      f"few to fit a {fitted}")
        status.append(f"    hint: set reference_trajectory.time_offset_s to "
                      f"{t0 - stamps[0] + offset:.3f} to line the two starts up")
    else:
        anchor_idx = int(np.flatnonzero(paired)[0])
        src = reference_xyz[paired] - reference_xyz[anchor_idx]
        dst = sampled[paired] - sampled[anchor_idx]
        R, angle = _fit_rotation(src, dst, yaw_only)
        residual = np.linalg.norm((src @ R.T) - dst, axis=1)
        # Signed for yaw, where the sign says which way round the reference was
        # turned; a magnitude for the full rotation, whose sign means nothing
        # without naming the axis it is about.
        turned = (f"{np.degrees(angle):+.2f} deg" if yaw_only
                  else f"{np.degrees(angle):.2f} deg about the fitted axis")
        status.append(
            f"    aligned: first position + best-fit {fitted} "
            f"({turned}) over {int(paired.sum())} paired samples")
        status.append(
            f"    residual after alignment: rms {float(np.sqrt((residual ** 2).mean())):.3f} m  "
            f"max {float(residual.max()):.3f} m")

    anchor_dst = (sampled[anchor_idx] if paired[anchor_idx]
                  else np.asarray(frames[0].pose_prev[:3, 3], dtype=float))
    T_fix = np.eye(4)
    T_fix[:3, :3] = R
    T_fix[:3, 3] = anchor_dst - R @ reference_xyz[anchor_idx]
    trajectory = [(stamp, T_fix @ T) for stamp, T in trajectory]
    return (label, trajectory), "\n".join(status)


def apply_simulated_drift(frames, settings, body_frame=None):
    """Inject the configured complementary-odometry drift, if any.

    No-op returning None when alpha is zero. Runs *after* any source swap, so
    the drift applies to whichever odometry is actually being replayed, and in
    the frame the estimator will measure it in, so that recovering alpha stays
    an exact question.
    """
    drift = settings.complementary_drift
    if not drift.alpha:
        return None
    modified = apply_complementary_drift(frames, drift.alpha, drift.axis,
                                         body_frame=body_frame)
    where = ("lidar" if body_frame is None or body_frame.is_identity
             else "complementary")
    return (f"simulated drift: {drift.alpha:+g} x |displacement| "
            f"along {where}-frame {drift.axis}\n"
            f"    frames affected: {modified}/{len(frames)}")


def reconstruct_replay_trajectories(frames, settings, params, collect_traces=False,
                                    body_frame=None):
    """Build the trajectory/trajectories selected by settings.scale_mode.

    Returns (results, scale_trace, vector_trace) where results is a list of
    (tag, trajectory) pairs; scale_trace/vector_trace are only populated for
    scale_mode "estimated" (and only when collect_traces is True).
    """
    mode = settings.correction_mode
    if settings.scale_mode == "fixed":
        results = []
        for scale in settings.scales:
            traj = reconstruct_fixed(frames, lambda f, s=scale: s, apply_correction=True,
                                     correction_mode=mode, body_frame=body_frame)
            results.append((f"scale_{scale:g}", traj))
        return results, [], []

    if settings.scale_mode == "recorded":
        traj = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True,
                                 correction_mode=mode, body_frame=body_frame)
        return [("recorded_scale", traj)], [], []

    if settings.scale_mode == "estimated":
        traj, scale_trace, vector_trace = reconstruct_with_estimator(
            frames, params, collect_vectors=collect_traces, correction_mode=mode,
            body_frame=body_frame)
        return [("estimated_scale", traj)], scale_trace, vector_trace

    raise ValueError(f"Unknown replay_scale_tool.scale_mode: {settings.scale_mode!r}")


# ---------------------------------------------------------------------------
# The pipeline
# ---------------------------------------------------------------------------

def _format_params(params):
    lines = ["  estimator params:",
             f"    complementaryCorrection: {params.complementary_correction}",
             f"    estimationFrame: {params.estimation_frame}"
             + ("  (+ extrinsic rotation)"
                if params.estimates_in_complementary_frame
                and params.estimation_frame_use_extrinsic_rot else ""),
             f"    translationScale: {params.translation_scale}",
             f"    scaleEstimationApply: {params.scale_estimation_apply}",
             f"    scaleMinNonDegenerateSpeed: {params.scale_min_nondegenerate_speed}",
             f"    scaleBaselineFrameLag: {params.scale_baseline_frame_lag}"]
    if params.uses_lines:
        lines.append(f"    scaleLineHistory: {params.scale_line_history}"
                     f" x {params.scale_line_history_step}"
                     f"  ({params.scale_line_fit_norm})")
        gates = []
        if np.isfinite(params.scale_line_max_scale_sigma):
            gates.append(f"sigma <= {params.scale_line_max_scale_sigma:g}")
        if params.scale_line_min_leg_lines > 0:
            gates.append(f">= {params.scale_line_min_leg_lines} lines "
                         f"{params.scale_line_min_leg_separation_deg:g} deg off")
        lines.append("    accepts a meeting point when: "
                     + (" and ".join(gates) if gates else "it exists"))
        if params.corrects_laterally:
            lines.append(f"    scaleLateralMax: {params.scale_lateral_max}")
    lines += [f"    scaleSmoothingWindowSize: {params.scale_smoothing_window_size}"
              f"  ({params.scale_smoothing_mode})",
              f"    scaleMin: {params.scale_min}  scaleMax: {params.scale_max}",
              f"    ignore_dz: {params.ignore_dz}"]
    return "\n".join(lines)


def run_replay(csv_path, settings, params=None, *, write=True, on_progress=None):
    """Replay one run and return a :class:`ReplayResult`.

    ``on_progress`` receives human-readable status lines as the replay
    advances; the CLI passes ``print``, a GUI can append to a log pane. Pass
    ``write=False`` to compute without emitting any file -- what a GUI wants
    while the user is dragging a parameter around.

    Raises ValueError if the CSV holds no frames.
    """
    emit = on_progress if on_progress is not None else (lambda _msg: None)

    frames = load_frames(csv_path)
    if not frames:
        raise ValueError(f"No frames found in {csv_path}")

    out_dir, traj_dir, log_dir = resolve_output_dirs(csv_path, settings)
    if write:
        os.makedirs(traj_dir, exist_ok=True)
        os.makedirs(log_dir, exist_ok=True)

    active = resolve_active_odometry(settings, csv_path)
    body_frame = estimation_body_frame(active.extrinsic, params)

    result = ReplayResult(
        csv_path=csv_path, settings=settings, params=params, frames=frames,
        out_dir=out_dir, traj_dir=traj_dir, log_dir=log_dir,
        active_odometry=active, body_frame=body_frame,
    )

    def _write_tum(path, trajectory):
        if not write:
            return
        write_tum(path, trajectory)
        result.written.append(path)
        emit(f"  wrote {path}")

    emit(f"Loaded {len(frames)} frames from {csv_path}")
    emit(f"  frames with degeneracy override: {result.n_degeneracy_override}")
    emit(f"  output directory: {out_dir}")
    emit(f"  scale mode: {settings.scale_mode}")
    if active.note:
        emit(f"  note: {active.note}")
    emit(f"  {describe_body_frame(body_frame, params, active)}")

    result.source_status = apply_complementary_source(frames, settings, csv_path, active)
    if result.source_status:
        emit(f"  {result.source_status}")

    result.drift_status = apply_simulated_drift(frames, settings, body_frame)
    if result.drift_status:
        emit(f"  {result.drift_status}")

    result.recorded_effective = recorded_effective_trajectory(frames)
    _write_tum(os.path.join(traj_dir, "trajectory_recorded_effective.tum"),
               result.recorded_effective)

    if settings.validate:
        # Deliberately twist6 regardless of settings.correction_mode: this
        # reproduces what the online run did, so it must use the node's rule.
        # Deliberately the LiDAR frame as well as twist6: the online run applied
        # its scale there, so reproducing it must too.
        traj_val = reconstruct_fixed(frames, lambda f: f.scale_applied, apply_correction=True)
        result.validation = ValidationReport(
            trajectory=traj_val, drift=position_drift(traj_val, result.recorded_effective))
        _write_tum(os.path.join(traj_dir, "trajectory_replay_validate.tum"), traj_val)
        if result.validation.drift.size:
            emit("  validation drift vs recorded effective: "
                 f"mean={result.validation.mean:.6f} m  max={result.validation.max:.6f} m  "
                 f"rms={result.validation.rms:.6f} m")

    if settings.scale_mode == "estimated":
        if params is None:
            raise ValueError("scale_mode 'estimated' requires params")
        emit(_format_params(params))

    source_tag = complementary_source_tag(settings)
    result.replays, result.scale_trace, result.vector_trace = reconstruct_replay_trajectories(
        frames, settings, params, collect_traces=True, body_frame=body_frame)
    for tag, traj in result.replays:
        _write_tum(os.path.join(traj_dir, f"trajectory_replay_{tag}{source_tag}.tum"), traj)

    # After the replays, because the fitted alignments turn the reference onto
    # one of them -- the first, which is the estimated trajectory in the mode
    # that produces several. It therefore moves when the configuration does,
    # which is the point ("fit it to what I am looking at") and also the reason
    # "none" and "first_pose" stay available for a fixed yardstick.
    result.reference, result.reference_status = load_reference_trajectory(
        settings, frames, result.replays[0][1] if result.replays else None)
    if result.reference_status:
        emit(f"  {result.reference_status}")

    if settings.scale_mode == "estimated" and write:
        trace_path = os.path.join(log_dir, f"scale_replay_estimator_trace{source_tag}.csv")
        vector_path = os.path.join(log_dir, f"scale_replay_vectors{source_tag}.csv")
        write_scale_trace_csv(trace_path, result.scale_trace)
        write_scale_vector_csv(vector_path, result.vector_trace)
        result.written += [trace_path, vector_path]
        emit(f"  wrote {trace_path}")
        emit(f"  wrote {vector_path}")

    if settings.no_correction:
        result.lidar_only = reconstruct_fixed(frames, lambda f: 1.0, apply_correction=False,
                                              body_frame=body_frame)
        _write_tum(os.path.join(traj_dir, "trajectory_replay_lidar_only.tum"), result.lidar_only)

    return result
