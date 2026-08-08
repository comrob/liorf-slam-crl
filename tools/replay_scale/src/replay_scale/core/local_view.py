"""Per-frame geometry for the anchor-frame vector view.

Two stages, deliberately separated:

``FrameGeometry``
    A neutral per-frame record holding everything the view needs, in map frame
    and in matrix form. One adapter per data source produces these; today the
    only source is a completed replay (:func:`geometry_from_replay`). A reader
    for ``scale_replay_vectors.csv`` would be a second adapter, never a second
    builder -- that is what keeps two sources from drifting apart. Rotations
    stay 3x3 here: quaternions are a serialization concern and must not leak
    back into this layer.

``LocalFrameView``
    The 2D drawable. Its origin is always the anchor position; ``frame``
    chooses whether the axes are rotated with it:

    ``"map"`` (default)
        Translate only -- axes stay map-aligned, so what you see lines up
        directly with the trajectory plot and with the tunnel.
    ``"anchor"``
        Also rotate by ``R_anchor^T``, putting the anchor's forward at +x.
        This is the frame the estimator itself works in: ``t_comp_map`` is
        literally ``R_anchor @ (displacement in anchor frame)``, so undoing
        ``R_anchor`` recovers the vector the estimator compared. It makes the
        view rotation-stable while scrubbing, at the cost of an orientation
        that changes meaning every frame.

Pure computation: no file access, no matplotlib, no Qt.
"""

from bisect import bisect_left
from dataclasses import dataclass, field

import numpy as np

from .lines import closest_point_to_lines  # noqa: F401  (re-exported for the view's users)
from .se3 import orthonormal_translation_basis

#: Below this the in-plane part of a degenerate axis is meaningless to draw.
_MIN_INPLANE_NORM = 1e-3

#: Window state, ordered: the further along, the more the estimator could do.
NO_WINDOW = 0        # the complementary lag window never closed
WINDOW = 1           # window closed, but the sample failed the speed gate
GATE_OBSERVABLE = 2  # window closed and the sample fed the scale history

WINDOW_STATES = (NO_WINDOW, WINDOW, GATE_OBSERVABLE)

#: Reference frames the view can be expressed in. All put the anchor at (0,0).
#:   map    -- translate only; axes stay map-aligned
#:   anchor -- rotate by R_anchor^T; the anchor's forward is +x
#:   comp   -- rotate in-plane so the complementary displacement is +x
FRAMES = ("map", "anchor", "comp")

#: How many earlier degenerate frames to overlay behind the current one.
DEFAULT_HISTORY = 50

#: Stride through the earlier degenerate frames, so the overlay spans a useful
#: interval instead of several near-identical consecutive lines.
DEFAULT_HISTORY_STEP = 4

#: Below this the complementary vector is too short to normalize by.
_MIN_COMP_NORM = 1e-6

#: Half-width used when normalizing, in units of |comp| rather than metres.
NORMALIZED_EXTENT = 2.0


@dataclass
class HistoryLine:
    """An earlier frame's degenerate direction, carried in two forms.

    Metric drawing re-references the line onto the *current* anchor, so the
    overlay shows real relative motion. Normalized drawing cannot do that --
    each frame is expressed in units of its own complementary length, so mixing
    a current-anchor offset with another frame's divisor would be meaningless.
    The ``own_*`` fields are therefore that frame's line in its own normalized
    geometry, which is what superimposes comparably.
    """

    origin: np.ndarray          # 2 -- latest position relative to the current anchor
    direction: np.ndarray       # 2 -- unit, in the current view's rotation
    own_origin: np.ndarray      # 2 -- latest position relative to its own anchor
    own_direction: np.ndarray   # 2 -- unit, in its own rotation
    #: |comp| of that frame; None when it had no window to normalize by.
    comp_norm: float = None
    age: int = 0                # frames back from the current one


@dataclass
class FrameGeometry:
    """Everything one frame contributes to the view, in map frame.

    ``has_window`` is explicit rather than inferred from NaN: adapters know
    whether the window closed, and every consumer must agree on the answer.
    """

    frame_idx: int
    time: float
    degeneracy_detected: bool
    gate_observable: bool
    has_window: bool
    anchor_frame_idx: int
    anchor_R: np.ndarray          # 3x3
    anchor_p: np.ndarray          # 3
    latest_R: np.ndarray          # 3x3
    latest_p: np.ndarray          # 3
    # Complementary displacement over the lag window, map frame. Only
    # meaningful when has_window; NaN otherwise.
    t_comp_map: np.ndarray        # 3
    # Degenerate translational directions, map frame, unit length. Empty when
    # the frame carries no usable basis -- independent of has_window.
    degenerate_axes_map: list = field(default_factory=list)
    # (cx, cy) where this frame's degenerate lines met, in its own
    # complementary-aligned |comp| = 1 geometry. None when they did not meet, or
    # when the estimator was not fitting them. Not a map-frame quantity: it only
    # means anything in that normalized frame, which is where the view draws it.
    meet_point: np.ndarray = None      # 2
    scale_instant_raw: float = float("nan")
    scale_smooth: float = float("nan")
    scale_applied: float = float("nan")
    lateral_instant_raw: float = float("nan")
    lateral_smooth: float = float("nan")
    lateral_applied: float = float("nan")

    @property
    def t_lidar_map(self):
        """LiDAR displacement anchor -> latest, map frame.

        Recomputed rather than read from the trace so it is available on every
        frame; identical to the estimator's own definition.
        """
        return self.latest_p - self.anchor_p

    @property
    def window_state(self):
        if not self.has_window:
            return NO_WINDOW
        return GATE_OBSERVABLE if self.gate_observable else WINDOW


@dataclass
class LocalFrameView:
    """One frame relative to its anchor, XY only.

    Geometry only: the degenerate directions are stored as unit vectors rather
    than as line segments, because how far to extend them is a property of the
    view being drawn, not of the frame.
    """

    frame_idx: int
    time: float
    #: Which reference frame the 2D vectors below are expressed in; see FRAMES.
    frame: str
    #: Seconds since the first frame -- absolute ROS stamps are unreadable.
    time_rel: float
    degeneracy_detected: bool
    window_state: int
    anchor_frame_idx: int
    #: Complementary displacement from the anchor origin. None when no window.
    comp_vec: np.ndarray = None            # 2
    #: Latest LiDAR position relative to the anchor. Always present.
    latest_pos: np.ndarray = None          # 2
    #: Unit in-plane degenerate directions through ``latest_pos``.
    degenerate_dirs: list = field(default_factory=list)
    #: Earlier frames' degenerate lines, re-expressed in this view's frame.
    history_lines: list = field(default_factory=list)
    #: Where the estimator's fit put the robot on this frame, as (cx, cy) in the
    #: complementary-aligned |comp| = 1 geometry. None when it did not fit one.
    meet_point: np.ndarray = None          # 2
    #: The same for the frames the overlay looks back at, as (age, point) pairs,
    #: newest first. Only comparable in that normalized frame -- each pair is in
    #: units of its own frame's |comp| -- so it is drawn nowhere else.
    meet_history: list = field(default_factory=list)
    #: Degenerate axes that exist but are too close to vertical to draw.
    n_out_of_plane_axes: int = 0
    #: True when frame == "comp" and there was a complementary vector to align
    #: to. False means the frame fell back to map orientation for this one.
    comp_aligned: bool = False
    scale_applied: float = float("nan")
    scale_instant_raw: float = float("nan")
    #: Cross-track part of what was applied, in units of |comp|. NaN unless the
    #: correction actually applies one.
    lateral_applied: float = float("nan")

    @property
    def has_comp_vec(self):
        return self.comp_vec is not None

    @property
    def reach(self):
        """Distance from the anchor to the farthest thing this frame draws."""
        r = float(np.linalg.norm(self.latest_pos))
        if self.has_comp_vec:
            r = max(r, float(np.linalg.norm(self.comp_vec)))
        return r

    @property
    def comp_norm(self):
        """|comp_vec| in metres, or None when there is nothing to divide by.

        Dividing the view through by this puts the complementary vector at unit
        length, so the LiDAR displacement is read directly as a multiple of it
        -- which is the ratio the scale estimate is made of.
        """
        if not self.has_comp_vec:
            return None
        norm = float(np.linalg.norm(self.comp_vec))
        return norm if norm > _MIN_COMP_NORM else None

    def status(self):
        """One line explaining what is on screen, or why nothing is."""
        bits = []
        if self.has_comp_vec:
            bits.append(f"|comp| = {float(np.linalg.norm(self.comp_vec)):.3f} m")
        else:
            bits.append("no complementary window")
        if self.degenerate_dirs:
            bits.append(f"{len(self.degenerate_dirs)} degenerate direction(s)")
        elif self.degeneracy_detected:
            bits.append("degenerate, but no in-plane axis to draw"
                        if self.n_out_of_plane_axes else "degenerate, no basis")
        else:
            bits.append("not degenerate")
        if np.isfinite(self.scale_applied):
            applied = f"scale applied = {self.scale_applied:.4f}"
            if np.isfinite(self.lateral_applied):
                applied += f"  (lateral {self.lateral_applied:+.4f})"
            bits.append(applied)
        return "  |  ".join(bits)


# ---------------------------------------------------------------------------
# Adapters: data source -> FrameGeometry
# ---------------------------------------------------------------------------

def geometry_from_replay(frames, trajectory, vector_trace):
    """Adapt a completed in-memory replay into per-frame geometry.

    ``trajectory`` is the estimated-scale trajectory as returned by
    ``run_replay`` -- the *corrected* poses, which is what the estimator
    buffers as anchor and latest. Passing the optimized poses instead would
    tilt every degenerate line in ``twist6`` mode.

    All three sequences are indexed by frame and must be the same length.
    """
    n = len(frames)
    if not (len(trajectory) == len(vector_trace) == n):
        raise ValueError(
            f"misaligned inputs: {n} frames, {len(trajectory)} poses, "
            f"{len(vector_trace)} vector records")

    out = []
    for k, (f, (_, T_latest), vf) in enumerate(zip(frames, trajectory, vector_trace)):
        anchor_idx = int(vf.anchor_frame_idx)
        T_anchor = trajectory[anchor_idx][1]

        basis = f.basis if (f.has_basis and len(f.basis) > 0) else []
        axes_map = [T_latest[:3, :3] @ u for u in orthonormal_translation_basis(basis)]

        has_window = bool(np.all(np.isfinite(vf.t_comp_map)))
        meet_point = getattr(vf, "meet_point", None)
        if meet_point is not None and not np.all(np.isfinite(meet_point)):
            meet_point = None
        out.append(FrameGeometry(
            frame_idx=k,
            time=float(f.time),
            degeneracy_detected=bool(vf.degeneracy_detected),
            gate_observable=bool(vf.gate_observable),
            has_window=has_window,
            anchor_frame_idx=anchor_idx,
            anchor_R=T_anchor[:3, :3].copy(),
            anchor_p=T_anchor[:3, 3].copy(),
            latest_R=T_latest[:3, :3].copy(),
            latest_p=T_latest[:3, 3].copy(),
            t_comp_map=np.array(vf.t_comp_map, dtype=float),
            degenerate_axes_map=axes_map,
            meet_point=(np.array(meet_point, dtype=float)
                        if meet_point is not None else None),
            scale_instant_raw=float(vf.scale_instant_raw),
            scale_smooth=float(vf.scale_smooth),
            scale_applied=float(vf.scale_applied),
            lateral_instant_raw=float(getattr(vf, "lateral_instant_raw", np.nan)),
            lateral_smooth=float(getattr(vf, "lateral_smooth", np.nan)),
            lateral_applied=float(getattr(vf, "lateral_applied", np.nan)),
        ))
    return out


# ---------------------------------------------------------------------------
# Builder: FrameGeometry -> LocalFrameView
# ---------------------------------------------------------------------------

#: Zoom steps in metres (half-width). Discrete on purpose -- see snap_extent.
EXTENT_LADDER = (0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 25.0, 50.0)


def snap_extent(reach, *, margin=1.25, ladder=EXTENT_LADDER):
    """Smallest ladder step that contains ``reach``.

    Per-frame autoscaling makes scrubbing unreadable -- the axes rescale
    continuously and nothing holds still. A single extent pinned to the run's
    maximum has the opposite problem when motion is bimodal: the slow frames
    collapse into a dot. Snapping to a coarse ladder gives both, since the view
    changes rarely and in steps you can recognise.
    """
    want = float(reach) * margin
    for step in ladder:
        if want <= step:
            return step
    return ladder[-1]


def _to_local(anchor_R, frame):
    """Rotation applied to map-frame vectors before dropping z.

    ``comp`` shares the map base: its alignment is an in-plane rotation applied
    afterwards, and aligning the complementary vector with +x wipes out
    whatever base it started from anyway.
    """
    if frame == "anchor":
        return anchor_R.T
    if frame in ("map", "comp"):
        return np.eye(3)
    raise ValueError(f"Unknown frame {frame!r}; expected one of {FRAMES}")


def _inplane_alignment(comp_2d):
    """2x2 rotation putting ``comp_2d`` on +x, or None if there is no direction.

    This is what makes the complementary displacement point forward on every
    frame: the whole picture is turned back by that vector's own angle.
    """
    if comp_2d is None:
        return None
    norm = float(np.linalg.norm(comp_2d))
    if norm < _MIN_COMP_NORM:
        return None
    c, s = comp_2d[0] / norm, comp_2d[1] / norm
    return np.array([[c, s], [-s, c]])


def _aligned(rot2, vec2):
    return vec2 if rot2 is None else rot2 @ vec2


def compute_axis_extent(geometries, *, frame="map", margin=1.15, minimum=1.0):
    """Half-width of one square view containing every frame, for a fixed zoom."""
    reach = 0.0
    for g in geometries:
        to_local = _to_local(g.anchor_R, frame)
        reach = max(reach, float(np.linalg.norm((to_local @ g.t_lidar_map)[:2])))
        if g.has_window:
            reach = max(reach, float(np.linalg.norm((to_local @ g.t_comp_map)[:2])))
    return max(minimum, reach * margin)


def build_local_frame_views(geometries, *, frame="map", history=DEFAULT_HISTORY,
                            history_step=DEFAULT_HISTORY_STEP, observable_only=False):
    """Project every frame relative to its anchor, in the requested frame.

    ``history`` overlays that many earlier degenerate frames behind the current
    one, so the evolution of the degenerate direction is visible rather than
    having to be reconstructed by scrubbing.

    ``observable_only`` restricts what the overlay may look back at to frames
    whose sample passed the observability gate. It applies to the history only:
    the current frame's own line is always drawn.

    Rebuilding with different options is cheap, so a frontend can offer them as
    toggles rather than baking one choice in.
    """
    if frame not in FRAMES:
        raise ValueError(f"Unknown frame {frame!r}; expected one of {FRAMES}")
    if history < 0:
        raise ValueError(f"history must be >= 0, got {history}")
    if history_step < 1:
        raise ValueError(f"history_step must be >= 1, got {history_step}")

    t0 = geometries[0].time if geometries else 0.0
    # Only frames that actually carry a drawable basis are worth looking back
    # at, so walk an index of those rather than the last N frames outright.
    degenerate_idx = [
        i for i, g in enumerate(geometries)
        if g.degenerate_axes_map
        and (not observable_only or g.window_state == GATE_OBSERVABLE)]

    views = []
    for k, g in enumerate(geometries):
        cut = bisect_left(degenerate_idx, k)
        if history:
            # Newest first, every history_step-th, then back to ascending.
            past = degenerate_idx[:cut][::-1][::history_step][:history][::-1]
        else:
            past = []
        views.append(_build_view(g, t0, frame, geometries, past))
    return views


def _build_view(g, t0, frame, geometries, past_indices):
    to_local = _to_local(g.anchor_R, frame)

    latest_pos = (to_local @ g.t_lidar_map)[:2]
    comp_vec = (to_local @ g.t_comp_map)[:2] if g.has_window else None

    align = _inplane_alignment(comp_vec) if frame == "comp" else None
    latest_pos = _aligned(align, latest_pos)
    if comp_vec is not None:
        comp_vec = _aligned(align, comp_vec)

    dirs = []
    out_of_plane = 0
    for axis in g.degenerate_axes_map:
        d = (to_local @ axis)[:2]
        norm = float(np.linalg.norm(d))
        if norm < _MIN_INPLANE_NORM:
            out_of_plane += 1
            continue
        dirs.append(_aligned(align, d / norm))

    # Each earlier frame's line was computed against *its own* anchor, so it has
    # to be re-referenced to this frame's anchor before it can share the axes.
    # Their meeting points cannot be: each is already in units of its own
    # frame's |comp|, which is the one frame they are all comparable in.
    history_lines = []
    meet_history = [(g.frame_idx - geometries[j].frame_idx, geometries[j].meet_point)
                    for j in reversed(past_indices)
                    if geometries[j].meet_point is not None]
    for j in past_indices:
        gj = geometries[j]
        to_local_j = _to_local(gj.anchor_R, frame)
        comp_j = (to_local_j @ gj.t_comp_map)[:2] if gj.has_window else None
        # Each earlier frame gets turned back by *its own* complementary angle,
        # so every overlaid frame has its odometry pointing forward too.
        align_j = _inplane_alignment(comp_j) if frame == "comp" else None

        origin = _aligned(align, (to_local @ (gj.latest_p - g.anchor_p))[:2])
        own_origin = _aligned(align_j, (to_local_j @ (gj.latest_p - gj.anchor_p))[:2])

        comp_norm = None
        if comp_j is not None:
            norm = float(np.linalg.norm(comp_j))
            comp_norm = norm if norm > _MIN_COMP_NORM else None

        for axis in gj.degenerate_axes_map:
            d = (to_local @ axis)[:2]
            own_d = (to_local_j @ axis)[:2]
            norm, own_norm = float(np.linalg.norm(d)), float(np.linalg.norm(own_d))
            if norm < _MIN_INPLANE_NORM or own_norm < _MIN_INPLANE_NORM:
                continue
            history_lines.append(HistoryLine(
                origin=origin,
                direction=_aligned(align, d / norm),
                own_origin=own_origin,
                own_direction=_aligned(align_j, own_d / own_norm),
                comp_norm=comp_norm, age=g.frame_idx - gj.frame_idx))

    return LocalFrameView(
        frame_idx=g.frame_idx,
        time=g.time,
        frame=frame,
        time_rel=g.time - t0,
        degeneracy_detected=g.degeneracy_detected,
        window_state=g.window_state,
        anchor_frame_idx=g.anchor_frame_idx,
        comp_vec=comp_vec,
        latest_pos=latest_pos,
        degenerate_dirs=dirs,
        history_lines=history_lines,
        meet_point=g.meet_point,
        meet_history=meet_history,
        comp_aligned=align is not None,
        n_out_of_plane_axes=out_of_plane,
        scale_applied=g.scale_applied,
        scale_instant_raw=g.scale_instant_raw,
        lateral_applied=g.lateral_applied,
    )
