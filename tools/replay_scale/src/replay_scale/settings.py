"""Configuration: the settings objects and the YAML that populates them.

The module is named ``settings`` rather than ``config`` because ``config/`` is
the package's data directory holding ``default.yaml``.

Everything a replay does is decided here. A frontend that wants to drive the
tool programmatically (the GUI) builds a :class:`ReplayToolSettings` plus a
:class:`ReplayParams` and hands them to :func:`replay_scale.pipeline.run_replay`;
the YAML loaders below are just one way of producing them.
"""

import os
from dataclasses import dataclass, field, fields, replace

import numpy as np

from .core.model import (
    COMPLEMENTARY_CORRECTIONS,
    CORRECTION_MODES,
    ESTIMATION_FRAMES,
    REFERENCE_ALIGNMENTS,
    SCALE_MODES,
    SMOOTHING_MODES,
    ReplayParams,
)
from .core.lines import LINE_FIT_NORMS
from .core.odom_source import DRIFT_AXES, MATCH_MODES
from .core.se3 import BodyFrame, quat_to_matrix

try:
    import yaml
except ImportError:  # Optional dependency until used.
    yaml = None

DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config", "default.yaml")
META_NAME = "complementary_odom_meta.yaml"


@dataclass
class RecordedOdometrySettings:
    """The odometry the node recorded into ``scale_replay_frames.csv``.

    Its mount is a property of the run, so it normally comes from the run's own
    ``complementary_odom_meta.yaml`` and this stays empty. Set it for a run whose
    meta file is missing or wrong.
    """

    # 4x4 T_complementary_to_lidar; None falls back to the run's meta file.
    extrinsic: object = None


@dataclass
class ComplementarySourceSettings:
    # Empty path keeps the twist baked into scale_replay_frames.csv by the online run.
    path: str = ""
    max_match_dt_s: float = 0.25
    # How the source pose at a LiDAR stamp is obtained; see core.odom_source.
    # "nearest" is what the online node does, so it stays the default.
    match_mode: str = "nearest"
    # The mount of *this stream*, and of nothing else: a 4x4, or one of
    # EXTRINSIC_SENTINELS. It never falls back to the recorded odometry's mount
    # -- that is a different sensor, and inheriting its mount is a silent wrong
    # answer rather than a missing one. ``"run"`` says in the file that this
    # stream is the recorded sensor's own (complementary_odom_stream.tum is),
    # which is the one case where sharing that mount is correct.
    extrinsic: object = None


@dataclass
class ComplementaryDriftSettings:
    """A simulated, distance-proportional error in the complementary odometry.

    Injecting a *known* error is what turns "does the estimated scale look
    right" into "we put in alpha, did we get it back".
    """

    # Fraction of distance travelled added along `axis`. 0 disables.
    alpha: float = 0.0
    # Body-frame axis the drift is injected along. Lateral by default: that is
    # the component the scale estimate is derived from, so it is where a
    # complementary-odometry error actually corrupts the result.
    axis: str = "y"


@dataclass
class ReferenceTrajectorySettings:
    """An external trajectory drawn alongside the replayed ones.

    Ground truth, a survey, another SLAM's output -- anything in TUM form. The
    replay never reads it: it is drawn, not compared against, so nothing the
    estimator does can depend on it.
    """

    # TUM file ("stamp tx ty tz qx qy qz qw"). Empty draws nothing. A
    # position-only reference -- a total station tracking a prism -- still has
    # to carry a quaternion per line; the alignments that fit a rotation never
    # read it.
    path: str = ""
    # Legend name; empty derives one from the filename.
    label: str = ""
    # How it is placed against the replay; see REFERENCE_ALIGNMENTS.
    align: str = "first_position_yaw"
    # Seconds added to the reference's stamps before it is matched against the
    # replay. A reference logged on its own clock shares no epoch with the run,
    # and the fitted alignments need correspondences: without an offset that
    # brings the two spans together there are none, and the replay says so --
    # printing the offset that would line the starts up.
    time_offset_s: float = 0.0


@dataclass
class ReplayToolSettings:
    scale_mode: str = "estimated"
    scales: list = field(default_factory=lambda: [1.0])
    # Which run to replay when the command line does not say. Empty input_path
    # means the newest run under base_dir; empty base_dir means the built-in
    # io.paths.DEFAULT_BASE_DIR, which is the only place that literal lives --
    # settings may not import io.
    input_path: str = ""
    base_dir: str = ""
    output_dir: str = ""
    output_subdir: str = "replay"
    no_correction: bool = False
    validate: bool = False
    correction_mode: str = "twist6"
    recorded_odometry: RecordedOdometrySettings = field(
        default_factory=RecordedOdometrySettings)
    complementary_source: ComplementarySourceSettings = field(
        default_factory=ComplementarySourceSettings)
    complementary_drift: ComplementaryDriftSettings = field(
        default_factory=ComplementaryDriftSettings)
    reference_trajectory: ReferenceTrajectorySettings = field(
        default_factory=ReferenceTrajectorySettings)

    def validated(self):
        """Return self after checking the enum-ish fields; raises ValueError."""
        if self.scale_mode not in SCALE_MODES:
            raise ValueError(
                f"replay_scale_tool.scale_mode must be one of {SCALE_MODES}, got {self.scale_mode!r}")
        if self.correction_mode not in CORRECTION_MODES:
            raise ValueError(f"replay_scale_tool.correction_mode must be one of "
                             f"{CORRECTION_MODES}, got {self.correction_mode!r}")
        if not self.scales:
            raise ValueError("replay_scale_tool.scales must contain at least one value")
        if self.complementary_source.match_mode not in MATCH_MODES:
            raise ValueError(f"replay_scale_tool.complementary_source.match_mode must be one of "
                             f"{MATCH_MODES}, got {self.complementary_source.match_mode!r}")
        if self.complementary_drift.axis not in DRIFT_AXES:
            raise ValueError(f"replay_scale_tool.complementary_drift.axis must be one of "
                             f"{DRIFT_AXES}, got {self.complementary_drift.axis!r}")
        if self.reference_trajectory.align not in REFERENCE_ALIGNMENTS:
            raise ValueError(f"replay_scale_tool.reference_trajectory.align must be one of "
                             f"{REFERENCE_ALIGNMENTS}, got "
                             f"{self.reference_trajectory.align!r}")
        return self

    def evolve(self, **changes):
        """Copy with fields replaced -- the GUI's edit primitive."""
        return replace(self, **changes)


# ---------------------------------------------------------------------------
# Extrinsics
# ---------------------------------------------------------------------------

#: How far from orthonormal a rotation matrix may be before it is rejected.
#: Loose enough for a hand-written matrix rounded to six decimals.
_ROTATION_TOLERANCE = 1e-3


def _flatten_numbers(values):
    """Flatten one level of nesting, so a 3x3 may be written as rows or flat."""
    flat = []
    for value in values:
        if isinstance(value, (list, tuple)):
            flat.extend(float(v) for v in value)
        else:
            flat.append(float(value))
    return flat


def rotation_from_sequence(values):
    """3x3 from nine numbers in row-major order, as the node's parameters are.

    ``Eigen::Map<..., RowMajor>(extRotV.data(), 3, 3)`` in ``utility.h`` is what
    this mirrors, so a matrix pasted from a node config means the same thing
    here as it does there. Rows may be nested or written flat.
    """
    flat = _flatten_numbers(values)
    if len(flat) != 9:
        raise ValueError(
            f"extrinsicRot must hold 9 numbers (row-major 3x3), got {len(flat)}")

    R = np.array(flat, dtype=float).reshape(3, 3)
    # A matrix that is not a rotation -- transposed sign errors, a dropped term,
    # a scale factor -- would otherwise pass silently into every frame's twist.
    if not np.allclose(R @ R.T, np.eye(3), atol=_ROTATION_TOLERANCE):
        raise ValueError(f"extrinsicRot is not orthonormal:\n{R}")
    if np.linalg.det(R) < 0.0:
        raise ValueError(f"extrinsicRot is a reflection (det < 0):\n{R}")
    return R


def extrinsic_from_mapping(node):
    """Build a 4x4 T_complementary_to_lidar from a mapping. None if absent.

    Two spellings are accepted:

    ``extrinsicTrans`` / ``extrinsicRot``
        The node's own, as in ``config/anymal.yaml``: a 3-vector and a
        row-major 3x3. Either may be omitted, defaulting to zero translation
        and identity rotation exactly as the ROS parameter declarations do.
    ``translation`` / ``rotation_quat_xyzw``
        What the run's ``complementary_odom_meta.yaml`` records, since a
        resolved TF is a quaternion by the time the node writes it.

    A mapping that is present but holds neither pair raises ValueError rather
    than being ignored: an extrinsic that silently does not apply shows up much
    later as an unexplained scale, having quietly used the run's own mount.
    """
    if not isinstance(node, dict):
        return None

    if "extrinsicRot" in node or "extrinsicTrans" in node:
        R = (rotation_from_sequence(node["extrinsicRot"])
             if node.get("extrinsicRot") is not None else np.eye(3))
        trans = _flatten_numbers(node.get("extrinsicTrans") or [0.0, 0.0, 0.0])
        if len(trans) != 3:
            raise ValueError(f"extrinsicTrans must hold 3 numbers, got {len(trans)}")
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = trans
        return T

    translation = node.get("translation")
    rotation = node.get("rotation_quat_xyzw")
    if translation is None or rotation is None:
        raise ValueError(
            "extrinsic must hold either extrinsicTrans/extrinsicRot (row-major 3x3, "
            f"as the node's parameters do) or translation/rotation_quat_xyzw; got keys "
            f"{sorted(node)}")
    tx, ty, tz = (float(v) for v in translation)
    qx, qy, qz, qw = (float(v) for v in rotation)
    return quat_to_matrix(tx, ty, tz, qx, qy, qz, qw)


#: Words a source's ``extrinsic`` may be written as instead of a mount.
#:   run      - this stream is the recorded sensor's own, so use the run's mount.
#:              complementary_odom_stream.tum is exactly that.
#:   identity - the stream is expressed in the LiDAR frame already.
#: Both are spelled out rather than inferred: an absent mount for a stream that
#: needs one is refused, because a wrong mount comes back out as a scale.
EXTRINSIC_SENTINELS = ("run", "identity")


def extrinsic_from_source_mapping(section):
    """The extrinsic of a ``complementary_source`` block, wherever it is written.

    The node spells these two keys directly in ``complementaryOdom``, so they
    are accepted at the top of the source block as well as inside an
    ``extrinsic:`` sub-mapping. A bare word is one of
    :data:`EXTRINSIC_SENTINELS`.
    """
    if "extrinsicRot" in section or "extrinsicTrans" in section:
        return extrinsic_from_mapping(section)
    value = section.get("extrinsic")
    if isinstance(value, str):
        word = value.strip()
        if word not in EXTRINSIC_SENTINELS:
            raise ValueError(
                f"complementary_source.extrinsic must be a mount or one of "
                f"{EXTRINSIC_SENTINELS}, got {value!r}")
        return word
    return extrinsic_from_mapping(value)


def estimation_body_frame(extrinsic, params):
    """The body frame ``params`` says the correction is measured and applied in.

    ``extrinsic`` is ``T_complementary_to_lidar`` -- the odometry sensor's frame
    written in LiDAR coordinates, which is where the mount already lives in both
    the config and the run's meta file. Nothing new has to be calibrated for
    this: the transform that carries the odometry *into* the LiDAR frame is the
    same one that says where to go to get out of it.

    Under ``estimationFrame: complementary`` only the *origin* moves by default.
    The lever arm between the two origins is what makes a body rotation look
    like odometry error, and moving the origin is what removes it. The axes are
    a separate question with a separate answer: they decide the plane the
    degenerate lines are fitted in, what ``ignore_dz`` drops and which way a
    simulated drift points, and the odometry's own axes are not always a sane
    choice for that -- a camera optical frame is z-forward, so adopting it would
    fit the lines in the vertical plane and quietly return plausible nonsense.
    ``estimationFrameUseExtrinsicRot`` takes the rotation too, for an odometry
    frame that does share the robot's convention.

    Returns the LiDAR frame itself for ``estimationFrame: lidar``, for absent
    params, and for a run with no extrinsic to place anything by.
    """
    if params is None or getattr(params, "estimation_frame", "lidar") != "complementary":
        return BodyFrame()
    if extrinsic is None:
        return BodyFrame()
    T = np.asarray(extrinsic, dtype=float)
    if params.estimation_frame_use_extrinsic_rot:
        return BodyFrame(T)
    origin_only = np.eye(4, dtype=float)
    origin_only[:3, 3] = T[:3, 3]
    return BodyFrame(origin_only)


def load_complementary_odom_meta(csv_path):
    """Read T_complementary_to_lidar from the run's meta file; None if absent."""
    meta_path = os.path.join(os.path.dirname(csv_path), META_NAME)
    if yaml is None or not os.path.isfile(meta_path):
        return None
    with open(meta_path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    if not isinstance(raw, dict):
        return None
    return extrinsic_from_mapping(raw.get("T_complementary_to_lidar"))


# ---------------------------------------------------------------------------
# YAML loading
# ---------------------------------------------------------------------------

def _require_yaml():
    if yaml is None:
        raise RuntimeError("PyYAML is required to load replay configuration. Install pyyaml.")


def _read_yaml(yaml_path):
    _require_yaml()
    with open(yaml_path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _extract_ros_parameters_root(raw_yaml):
    if not isinstance(raw_yaml, dict):
        return {}

    if "/**" in raw_yaml and isinstance(raw_yaml["/**"], dict):
        candidate = raw_yaml["/**"].get("ros__parameters")
        if isinstance(candidate, dict):
            return candidate

    for _, value in raw_yaml.items():
        if isinstance(value, dict):
            candidate = value.get("ros__parameters")
            if isinstance(candidate, dict):
                return candidate

    # Fallback: assume user passed just the parameters subtree.
    return raw_yaml


def validate_replay_params(params):
    """Return params after checking the fields only meaningful together.

    ``ReplayParams`` is a plain dataclass so that the estimator can be handed
    one from anywhere; this is the shared check both the YAML loader and the
    GUI editor run, so neither can produce a configuration the other would
    reject. Raises ValueError.
    """
    if params.complementary_correction not in COMPLEMENTARY_CORRECTIONS:
        raise ValueError(f"complementaryOdom.complementaryCorrection must be one of "
                         f"{COMPLEMENTARY_CORRECTIONS}, got "
                         f"{params.complementary_correction!r}")
    if params.estimation_frame not in ESTIMATION_FRAMES:
        raise ValueError(f"complementaryOdom.estimationFrame must be one of "
                         f"{ESTIMATION_FRAMES}, got {params.estimation_frame!r}")
    if params.scale_line_fit_norm not in LINE_FIT_NORMS:
        raise ValueError(f"complementaryOdom.scaleLineFitNorm must be one of "
                         f"{LINE_FIT_NORMS}, got {params.scale_line_fit_norm!r}")
    if params.scale_line_history < 0:
        raise ValueError(f"complementaryOdom.scaleLineHistory must be >= 0, got "
                         f"{params.scale_line_history}")
    if params.scale_line_history_step < 1:
        raise ValueError(f"complementaryOdom.scaleLineHistoryStep must be >= 1, got "
                         f"{params.scale_line_history_step}")
    # Both are "at least this much"; inf and 0 are the off switches, so the
    # only unusable values are negative ones.
    if params.scale_line_max_scale_sigma < 0.0:
        raise ValueError(f"complementaryOdom.scaleLineMaxScaleSigma must be >= 0, got "
                         f"{params.scale_line_max_scale_sigma}")
    if params.scale_line_min_leg_lines < 0:
        raise ValueError(f"complementaryOdom.scaleLineMinLegLines must be >= 0, got "
                         f"{params.scale_line_min_leg_lines}")
    if not 0.0 <= params.scale_line_min_leg_separation_deg < 180.0:
        raise ValueError(f"complementaryOdom.scaleLineMinLegSeparationDeg must be in "
                         f"[0, 180), got {params.scale_line_min_leg_separation_deg}")
    if params.scale_smoothing_mode not in SMOOTHING_MODES:
        raise ValueError(f"complementaryOdom.scaleSmoothingMode must be one of "
                         f"{SMOOTHING_MODES}, got {params.scale_smoothing_mode!r}")
    if params.scale_min < 0.0:
        raise ValueError(f"complementaryOdom.scaleMin must be >= 0, got {params.scale_min}")
    if params.scale_max <= params.scale_min:
        raise ValueError(f"complementaryOdom.scaleMax must exceed scaleMin, got "
                         f"scaleMin={params.scale_min}, scaleMax={params.scale_max}")
    # Symmetric, so a single non-negative number is the whole bound: the lines
    # are as free to put the robot left of the odometry as right of it.
    if params.scale_lateral_max < 0.0:
        raise ValueError(f"complementaryOdom.scaleLateralMax must be >= 0, got "
                         f"{params.scale_lateral_max}")
    return params


#: Older spellings still accepted, mapped onto the current ones. The key was
#: "scaleSampleSource" while every method produced a scalar scale; once one of
#: them could also move the odometry sideways, what is being chosen is the
#: correction, not the sample -- and "line_meet" had to say *which* part of the
#: meeting point it meant.
_LEGACY_CORRECTION_KEY = "scaleSampleSource"
_LEGACY_CORRECTION_VALUES = {"line_meet": "lines_meet_x"}


def replay_params_from_mapping(root):
    """Build ReplayParams from an already-parsed ros__parameters mapping."""
    params = ReplayParams()

    comp = root.get("complementaryOdom", {}) if isinstance(root, dict) else {}
    if isinstance(comp, dict):
        # -- every method ---------------------------------------------------
        params.translation_scale = float(comp.get("translationScale", params.translation_scale))
        params.scale_estimation_apply = bool(comp.get("scaleEstimationApply", params.scale_estimation_apply))
        params.scale_min_nondegenerate_speed = float(
            comp.get("scaleMinNonDegenerateSpeed", params.scale_min_nondegenerate_speed))
        params.scale_baseline_frame_lag = int(comp.get("scaleBaselineFrameLag", params.scale_baseline_frame_lag))
        params.ignore_dz = bool(comp.get("ignore_dz", params.ignore_dz))
        correction = str(comp.get("complementaryCorrection",
                                  comp.get(_LEGACY_CORRECTION_KEY,
                                           params.complementary_correction)))
        params.complementary_correction = _LEGACY_CORRECTION_VALUES.get(correction, correction)
        params.estimation_frame = str(comp.get("estimationFrame", params.estimation_frame))
        params.estimation_frame_use_extrinsic_rot = bool(
            comp.get("estimationFrameUseExtrinsicRot",
                     params.estimation_frame_use_extrinsic_rot))

        # -- lines_meet_x / lines_meet_xy ------------------------------------
        params.scale_line_history = int(comp.get("scaleLineHistory", params.scale_line_history))
        params.scale_line_history_step = int(
            comp.get("scaleLineHistoryStep", params.scale_line_history_step))
        params.scale_line_fit_norm = str(comp.get("scaleLineFitNorm", params.scale_line_fit_norm))
        params.scale_lateral_max = float(comp.get("scaleLateralMax", params.scale_lateral_max))
        params.scale_line_max_scale_sigma = float(
            comp.get("scaleLineMaxScaleSigma", params.scale_line_max_scale_sigma))
        params.scale_line_min_leg_lines = int(
            comp.get("scaleLineMinLegLines", params.scale_line_min_leg_lines))
        params.scale_line_min_leg_separation_deg = float(
            comp.get("scaleLineMinLegSeparationDeg",
                     params.scale_line_min_leg_separation_deg))

        # -- smoothing ------------------------------------------------------
        params.scale_smoothing_window_size = int(comp.get("scaleSmoothingWindowSize", params.scale_smoothing_window_size))
        params.scale_smoothing_mode = str(comp.get("scaleSmoothingMode", params.scale_smoothing_mode))
        params.scale_min = float(comp.get("scaleMin", params.scale_min))
        params.scale_max = float(comp.get("scaleMax", params.scale_max))

    params.scale_baseline_frame_lag = max(1, params.scale_baseline_frame_lag)
    params.scale_smoothing_window_size = max(1, params.scale_smoothing_window_size)
    return validate_replay_params(params)


def replay_tool_settings_from_mapping(section):
    """Build ReplayToolSettings from an already-parsed replay_scale_tool mapping."""
    settings = ReplayToolSettings()
    if isinstance(section, dict):
        settings.scale_mode = str(section.get("scale_mode", settings.scale_mode))
        settings.scales = [float(s) for s in section.get("scales", settings.scales)]
        settings.input_path = str(section.get("input_path", settings.input_path))
        settings.base_dir = str(section.get("base_dir", settings.base_dir))
        settings.output_dir = str(section.get("output_dir", settings.output_dir))
        settings.output_subdir = str(section.get("output_subdir", settings.output_subdir))
        settings.no_correction = bool(section.get("no_correction", settings.no_correction))
        settings.validate = bool(section.get("validate", settings.validate))
        settings.correction_mode = str(section.get("correction_mode", settings.correction_mode))

        recorded_section = section.get("recorded_odometry", {})
        if isinstance(recorded_section, dict):
            settings.recorded_odometry.extrinsic = extrinsic_from_source_mapping(
                recorded_section)
            if isinstance(settings.recorded_odometry.extrinsic, str):
                raise ValueError(
                    "recorded_odometry.extrinsic must be a mount, not "
                    f"{settings.recorded_odometry.extrinsic!r}: the run's own is what "
                    "an omitted one falls back to already")

        source_section = section.get("complementary_source", {})
        if isinstance(source_section, dict):
            source = settings.complementary_source
            source.path = str(source_section.get("path", source.path))
            source.max_match_dt_s = float(
                source_section.get("max_match_dt_s", source.max_match_dt_s))
            source.match_mode = str(source_section.get("match_mode", source.match_mode))
            source.extrinsic = extrinsic_from_source_mapping(source_section)

        drift_section = section.get("complementary_drift", {})
        if isinstance(drift_section, dict):
            drift = settings.complementary_drift
            drift.alpha = float(drift_section.get("alpha", drift.alpha))
            drift.axis = str(drift_section.get("axis", drift.axis))

        reference_section = section.get("reference_trajectory", {})
        if isinstance(reference_section, dict):
            reference = settings.reference_trajectory
            reference.path = str(reference_section.get("path", reference.path))
            reference.label = str(reference_section.get("label", reference.label))
            reference.align = str(reference_section.get("align", reference.align))
            reference.time_offset_s = float(
                reference_section.get("time_offset_s", reference.time_offset_s))

    return settings.validated()


def load_replay_params_from_ros_yaml(yaml_path):
    """Load the complementaryOdom parameters from a ROS parameters YAML."""
    return replay_params_from_mapping(_extract_ros_parameters_root(_read_yaml(yaml_path)))


def load_replay_tool_settings(yaml_path):
    """Load the replay_scale_tool section (sibling of the ROS ros__parameters tree)."""
    raw = _read_yaml(yaml_path)
    return replay_tool_settings_from_mapping(
        raw.get("replay_scale_tool", {}) if isinstance(raw, dict) else {})


def config_to_mapping(settings, params):
    """Round-trippable dict in the on-disk config layout.

    Keeps the two trees separate exactly as the file does: ``ros__parameters``
    for what the node itself reads, ``replay_scale_tool`` for this tool.
    """
    def _mount(target, extrinsic):
        """Write a mount into a block, in the node's own spelling."""
        if extrinsic is None:
            return
        if isinstance(extrinsic, str):
            target["extrinsic"] = extrinsic
            return
        # The matrix itself rather than a quaternion: a config saved here is
        # meant to be pasted back and read.
        T = np.asarray(extrinsic, dtype=float)
        target["extrinsicTrans"] = [float(v) for v in T[:3, 3]]
        target["extrinsicRot"] = [float(v) for v in T[:3, :3].reshape(9)]

    recorded = {}
    _mount(recorded, settings.recorded_odometry.extrinsic)

    source = {
        "path": settings.complementary_source.path,
        "max_match_dt_s": float(settings.complementary_source.max_match_dt_s),
        "match_mode": settings.complementary_source.match_mode,
    }
    _mount(source, settings.complementary_source.extrinsic)
    # Written in the same order the bundled config is grouped in -- what the
    # correction is, then each method's own settings, then what they share --
    # so a saved file reads like the documented one rather than like a dump.
    return {
        "/**": {
            "ros__parameters": {
                "complementaryOdom": {
                    # every method
                    "complementaryCorrection": params.complementary_correction,
                    "estimationFrame": params.estimation_frame,
                    "estimationFrameUseExtrinsicRot": bool(
                        params.estimation_frame_use_extrinsic_rot),
                    "scaleEstimationApply": bool(params.scale_estimation_apply),
                    "scaleMinNonDegenerateSpeed": float(params.scale_min_nondegenerate_speed),
                    "scaleBaselineFrameLag": int(params.scale_baseline_frame_lag),
                    "ignore_dz": bool(params.ignore_dz),
                    "translationScale": float(params.translation_scale),
                    # lines_meet_x / lines_meet_xy
                    "scaleLineHistory": int(params.scale_line_history),
                    "scaleLineHistoryStep": int(params.scale_line_history_step),
                    "scaleLineFitNorm": params.scale_line_fit_norm,
                    "scaleLateralMax": float(params.scale_lateral_max),
                    "scaleLineMaxScaleSigma": float(params.scale_line_max_scale_sigma),
                    "scaleLineMinLegLines": int(params.scale_line_min_leg_lines),
                    "scaleLineMinLegSeparationDeg": float(
                        params.scale_line_min_leg_separation_deg),
                    # smoothing and bounds
                    "scaleSmoothingWindowSize": int(params.scale_smoothing_window_size),
                    "scaleSmoothingMode": params.scale_smoothing_mode,
                    "scaleMin": float(params.scale_min),
                    "scaleMax": float(params.scale_max),
                },
            },
        },
        "replay_scale_tool": {
            # which run
            "input_path": settings.input_path,
            "base_dir": settings.base_dir,
            # what to replay
            "scale_mode": settings.scale_mode,
            "scales": [float(s) for s in settings.scales],
            "correction_mode": settings.correction_mode,
            "no_correction": bool(settings.no_correction),
            "validate": bool(settings.validate),
            # what to replay it against
            "recorded_odometry": recorded,
            "complementary_source": source,
            "complementary_drift": {
                "alpha": float(settings.complementary_drift.alpha),
                "axis": settings.complementary_drift.axis,
            },
            "reference_trajectory": {
                "path": settings.reference_trajectory.path,
                "label": settings.reference_trajectory.label,
                "align": settings.reference_trajectory.align,
                "time_offset_s": float(settings.reference_trajectory.time_offset_s),
            },
            # where the output goes
            "output_dir": settings.output_dir,
            "output_subdir": settings.output_subdir,
        },
    }


def _dump_yaml(mapping):
    """YAML text for a config mapping.

    ``default_flow_style=None`` keeps a list of numbers on one line, which is
    how a mount or a set of scales is read -- a row-major 3x3 written one number
    per line is not a matrix any more.
    """
    return yaml.safe_dump(mapping, sort_keys=False, default_flow_style=None)


def read_mapping(yaml_path):
    """A config file (or fragment) as a raw mapping; ``{}`` when it is empty.

    For a caller assembling its own layer -- the viewer's per-run state -- that
    has to merge before anything is parsed, like every other layer here.
    """
    return _read_yaml(yaml_path) or {}


def dump_mapping(mapping, header=""):
    """A config mapping (or fragment) as YAML text, with an optional header."""
    _require_yaml()
    return header + (_dump_yaml(mapping) if mapping else "")


def dump_config(settings, params):
    """Serialize settings + params to config YAML text."""
    _require_yaml()
    return _dump_yaml(config_to_mapping(settings, params))


def save_config(yaml_path, settings, params):
    """Write settings + params as a config file readable by :func:`load_config`.

    Comments in a hand-edited source file are not preserved -- this writes the
    values, not the document.
    """
    with open(yaml_path, "w", encoding="utf-8") as fh:
        fh.write(dump_config(settings, params))


def load_config(yaml_path):
    """Load both halves of a config file in one pass.

    Returns ``(settings, params)``. Preferred over calling the two loaders
    separately: it parses the file once and keeps the pair consistent.
    """
    return config_from_mapping(_read_yaml(yaml_path))


def config_from_mapping(raw):
    """``(settings, params)`` from an already-parsed config mapping."""
    settings = replay_tool_settings_from_mapping(
        raw.get("replay_scale_tool", {}) if isinstance(raw, dict) else {})
    params = replay_params_from_mapping(_extract_ros_parameters_root(raw))
    return settings, params


# ---------------------------------------------------------------------------
# Per-run configuration
# ---------------------------------------------------------------------------

#: A run's own configuration file, beside its scale_replay_frames.csv. Not under
#: the replay output directory, which the tool overwrites.
RUN_CONFIG_NAME = "replay_scale.yaml"

#: Keys a per-run file may not set: the run is the directory the file is in, so
#: naming another one is at best redundant and at worst a file that replays
#: something else entirely.
RUN_CONFIG_IGNORED = ("input_path", "base_dir")

#: Sections of ``replay_scale_tool`` that describe one particular run rather
#: than a method swept across runs. An external reference trajectory is a file
#: recorded alongside *this* run, on *this* run's clock, so carrying one from
#: the run last looked at to the next would draw a stranger's trajectory beside
#: it -- aligned, plausible, and about a different drive. They therefore reach a
#: replay from the run's own file or not at all: the viewer keeps them out of
#: what it remembers between loads, so switching runs shows the run's own
#: reference, or none. A configuration named explicitly (``--ros-params-yaml``,
#: "Load YAML...") is a deliberate act and still gets to set one for every run
#: it is applied to.
RUN_SCOPED_SECTIONS = ("reference_trajectory",)

_RUN_CONFIG_HEADER = """\
# replay_scale configuration for this run.
#
# Merged over the base configuration when this run is replayed, so it holds only
# what is specific to the run itself -- the mounts, the reference trajectory and
# its clock offset -- and inherits everything else. Delete it to fall back to
# the base configuration entirely, or pass --no-run-config to ignore it once.
"""


def deep_merge(base, override):
    """``base`` with ``override`` applied, merging mappings all the way down.

    Merging the raw mappings rather than the settings objects is what keeps this
    to one function: every loader, default and validator below already works on
    one mapping, and none of them has to learn that a configuration can now come
    from two files. A key present in ``override`` wins outright unless both
    sides are mappings, in which case their keys are merged -- so a run file
    naming one parameter does not discard its siblings.
    """
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    merged = dict(base)
    for key, value in override.items():
        merged[key] = deep_merge(merged.get(key), value) if key in merged else value
    return merged


def mapping_delta(mapping, base):
    """The part of ``mapping`` that differs from ``base``, as a mapping.

    The inverse of :func:`deep_merge`: ``deep_merge(base, mapping_delta(m, base))``
    is ``m`` again. Writing a run file as a delta is what keeps it to the handful
    of keys that are genuinely run-specific -- a full copy would silently pin
    every method setting, so a later change to a shared default would never
    reach the run.
    """
    if not isinstance(mapping, dict) or not isinstance(base, dict):
        return mapping
    out = {}
    for key, value in mapping.items():
        if key not in base:
            out[key] = value
            continue
        if isinstance(value, dict) and isinstance(base[key], dict):
            nested = mapping_delta(value, base[key])
            if nested:
                out[key] = nested
        elif value != base[key]:
            out[key] = value
    return out


@dataclass
class ConfigSources:
    """Where one run's configuration came from, and what it was built on.

    ``base_mapping`` is the configuration *without* the run's own file, which is
    what a per-run file is written as a delta against; keeping it here is what
    lets the viewer save one without re-reading anything.
    """

    base_path: str = ""
    base_mapping: dict = field(default_factory=dict)
    run_path: str = ""
    #: Top-level sections the run's file contributed to.
    run_sections: tuple = ()
    #: Keys in the run's file that were ignored; see RUN_CONFIG_IGNORED.
    ignored_keys: tuple = ()
    merged: dict = field(default_factory=dict)
    #: The configuration as the files on disk describe it -- base plus the run's
    #: own file -- before any layer a frontend adds on top. What such a layer is
    #: written as a delta against, the way a run file is written against
    #: ``base_mapping``. The viewer keeps what was last applied to a run here;
    #: see :mod:`replay_scale.gui.run_state`. A CLI replay has no such layer:
    #: it depends on its arguments and the files it was given, and nothing else.
    inherited: dict = field(default_factory=dict)
    #: Where that frontend layer came from, and what it contributed. Empty for
    #: every configuration the CLI resolves.
    state_path: str = ""
    state_sections: tuple = ()

    def describe(self):
        """The provenance line every frontend prints or shows."""
        lines = [f"config: {self.base_path}"]
        if self.run_path:
            named = ["complementaryOdom" if key == "/**" else key
                     for key in self.run_sections]
            sections = ", ".join(named) or "nothing"
            lines.append(f"    + {self.run_path}  ({sections})")
        if self.ignored_keys:
            lines.append(f"    ignored in the run's file: "
                         f"{', '.join(self.ignored_keys)} -- the run is where the file is")
        if self.state_path:
            sections = ", ".join(self.state_sections) or "nothing"
            lines.append(f"    + {self.state_path}  ({sections})  "
                         f"-- last applied in the viewer, which only it reads")
        return "\n".join(lines)


def run_config_path(csv_path):
    """Where the per-run configuration for this run lives."""
    return os.path.join(os.path.dirname(os.path.abspath(csv_path)), RUN_CONFIG_NAME)


def _strip_tool_keys(raw, keys):
    """(mapping without those ``replay_scale_tool`` keys, the names dropped)."""
    tool = raw.get("replay_scale_tool") if isinstance(raw, dict) else None
    if not isinstance(tool, dict):
        return raw, ()
    dropped = tuple(k for k in keys if k in tool)
    if not dropped:
        return raw, ()
    pruned = dict(raw)
    pruned["replay_scale_tool"] = {k: v for k, v in tool.items() if k not in dropped}
    return pruned, dropped


def _strip_ignored(raw):
    """(mapping without the keys a run file may not set, the names dropped)."""
    return _strip_tool_keys(raw, RUN_CONFIG_IGNORED)


def strip_run_scoped(raw):
    """(mapping without the run-scoped sections, the names dropped).

    For a configuration that is about to be reused across runs; see
    :data:`RUN_SCOPED_SECTIONS`.
    """
    return _strip_tool_keys(raw, RUN_SCOPED_SECTIONS)


def same_apart_from_run_scoped(a, b):
    """Whether two ``(settings, params)`` pairs differ only in run-scoped parts.

    Compared as the mappings a config file is written from rather than as the
    dataclasses: those hold mounts as arrays, whose ``==`` is an array and not
    a truth value. What it is for: deciding that an edit needs no replay,
    because the only thing that changed is drawn rather than computed.
    """
    return (strip_run_scoped(config_to_mapping(*a))[0]
            == strip_run_scoped(config_to_mapping(*b))[0])


def without_run_scoped(settings):
    """``settings`` with every run-scoped section back at its default.

    The settings-object counterpart of :func:`strip_run_scoped`, for the places
    that hold a :class:`ReplayToolSettings` rather than the mapping it came
    from. Derived from the dataclass's own defaults, so a section added to
    :data:`RUN_SCOPED_SECTIONS` needs nothing here.
    """
    return settings.evolve(**{f.name: f.default_factory()
                              for f in fields(settings)
                              if f.name in RUN_SCOPED_SECTIONS})


def config_sources_for_run(csv_path, base_path=DEFAULT_CONFIG_PATH, use_run_config=True):
    """Resolve the configuration layering for one run; see :class:`ConfigSources`."""
    base_mapping = _read_yaml(base_path) or {}
    sources = ConfigSources(base_path=base_path, base_mapping=base_mapping,
                            merged=base_mapping, inherited=base_mapping)
    if not use_run_config:
        return sources

    path = run_config_path(csv_path)
    if not os.path.isfile(path):
        return sources

    run_mapping = _read_yaml(path) or {}
    run_mapping, ignored = _strip_ignored(run_mapping)
    sources.run_path = path
    sources.ignored_keys = ignored
    sources.run_sections = tuple(sorted(run_mapping)) if isinstance(run_mapping, dict) else ()
    sources.merged = deep_merge(base_mapping, run_mapping)
    sources.inherited = sources.merged
    return sources


def load_config_for_run(csv_path, base_path=DEFAULT_CONFIG_PATH, use_run_config=True):
    """``(settings, params, sources)`` for one run, run file merged over the base.

    The per-run file is picked up automatically when it is there, which is what
    makes it useful -- and why :meth:`ConfigSources.describe` is printed rather
    than the layering being silent.
    """
    sources = config_sources_for_run(csv_path, base_path, use_run_config)
    settings, params = config_from_mapping(sources.merged)
    return settings, params, sources


def save_run_config(csv_path, settings, params, base_mapping=None, path=None):
    """Write this configuration as the run's own file; returns the path.

    With ``base_mapping``, only what differs from it is written, which is what
    a per-run file is for. Without one the whole configuration is written, which
    is still a valid run file -- just one that pins everything.
    """
    _require_yaml()
    mapping, _ = _strip_ignored(config_to_mapping(settings, params))
    if base_mapping is not None:
        # Normalized through the same writer before comparing: a hand-written
        # base leaves defaults implicit, and diffing against it would call every
        # default an override and write the whole tree out as "differences".
        stripped_base, _ = _strip_ignored(
            config_to_mapping(*config_from_mapping(base_mapping)))
        mapping = mapping_delta(mapping, stripped_base)
    path = path or run_config_path(csv_path)
    body = _dump_yaml(mapping) if mapping else ""
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(_RUN_CONFIG_HEADER)
        fh.write("#\n# Nothing differs from the base configuration.\n" if not body else "")
        fh.write(body)
    return path
