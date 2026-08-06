"""Configuration: the settings objects and the YAML that populates them.

The module is named ``settings`` rather than ``config`` because ``config/`` is
the package's data directory holding ``default.yaml``.

Everything a replay does is decided here. A frontend that wants to drive the
tool programmatically (the GUI) builds a :class:`ReplayToolSettings` plus a
:class:`ReplayParams` and hands them to :func:`replay_scale.pipeline.run_replay`;
the YAML loaders below are just one way of producing them.
"""

import os
from dataclasses import dataclass, field, replace

from .core.model import CORRECTION_MODES, SCALE_MODES, ReplayParams
from .core.odom_source import DRIFT_AXES
from .core.se3 import matrix_to_quat, quat_to_matrix

try:
    import yaml
except ImportError:  # Optional dependency until used.
    yaml = None

DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config", "default.yaml")
META_NAME = "complementary_odom_meta.yaml"


@dataclass
class ComplementarySourceSettings:
    # Empty path keeps the twist baked into scale_replay_frames.csv by the online run.
    path: str = ""
    max_match_dt_s: float = 0.25
    # 4x4 T_complementary_to_lidar override; None falls back to the run's meta file.
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
class ReplayToolSettings:
    scale_mode: str = "estimated"
    scales: list = field(default_factory=lambda: [1.0])
    output_dir: str = ""
    output_subdir: str = "replay"
    no_correction: bool = False
    validate: bool = False
    correction_mode: str = "twist6"
    complementary_source: ComplementarySourceSettings = field(
        default_factory=ComplementarySourceSettings)
    complementary_drift: ComplementaryDriftSettings = field(
        default_factory=ComplementaryDriftSettings)

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
        if self.complementary_drift.axis not in DRIFT_AXES:
            raise ValueError(f"replay_scale_tool.complementary_drift.axis must be one of "
                             f"{DRIFT_AXES}, got {self.complementary_drift.axis!r}")
        return self

    def evolve(self, **changes):
        """Copy with fields replaced -- the GUI's edit primitive."""
        return replace(self, **changes)


# ---------------------------------------------------------------------------
# Extrinsics
# ---------------------------------------------------------------------------

def extrinsic_from_mapping(node):
    """Build a 4x4 T_complementary_to_lidar from a translation/quaternion mapping."""
    if not isinstance(node, dict):
        return None
    translation = node.get("translation")
    rotation = node.get("rotation_quat_xyzw")
    if translation is None or rotation is None:
        return None
    tx, ty, tz = (float(v) for v in translation)
    qx, qy, qz, qw = (float(v) for v in rotation)
    return quat_to_matrix(tx, ty, tz, qx, qy, qz, qw)


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


def replay_params_from_mapping(root):
    """Build ReplayParams from an already-parsed ros__parameters mapping."""
    params = ReplayParams()

    comp = root.get("complementaryOdom", {}) if isinstance(root, dict) else {}
    if isinstance(comp, dict):
        params.translation_scale = float(comp.get("translationScale", params.translation_scale))
        params.scale_estimation_apply = bool(comp.get("scaleEstimationApply", params.scale_estimation_apply))
        params.scale_min_nondegenerate_speed = float(
            comp.get("scaleMinNonDegenerateSpeed", params.scale_min_nondegenerate_speed))
        params.scale_baseline_frame_lag = int(comp.get("scaleBaselineFrameLag", params.scale_baseline_frame_lag))
        params.scale_smoothing_window_size = int(comp.get("scaleSmoothingWindowSize", params.scale_smoothing_window_size))
        params.ignore_dz = bool(comp.get("ignore_dz", params.ignore_dz))

    params.scale_baseline_frame_lag = max(1, params.scale_baseline_frame_lag)
    params.scale_smoothing_window_size = max(1, params.scale_smoothing_window_size)
    return params


def replay_tool_settings_from_mapping(section):
    """Build ReplayToolSettings from an already-parsed replay_scale_tool mapping."""
    settings = ReplayToolSettings()
    if isinstance(section, dict):
        settings.scale_mode = str(section.get("scale_mode", settings.scale_mode))
        settings.scales = [float(s) for s in section.get("scales", settings.scales)]
        settings.output_dir = str(section.get("output_dir", settings.output_dir))
        settings.output_subdir = str(section.get("output_subdir", settings.output_subdir))
        settings.no_correction = bool(section.get("no_correction", settings.no_correction))
        settings.validate = bool(section.get("validate", settings.validate))
        settings.correction_mode = str(section.get("correction_mode", settings.correction_mode))

        source_section = section.get("complementary_source", {})
        if isinstance(source_section, dict):
            source = settings.complementary_source
            source.path = str(source_section.get("path", source.path))
            source.max_match_dt_s = float(
                source_section.get("max_match_dt_s", source.max_match_dt_s))
            source.extrinsic = extrinsic_from_mapping(source_section.get("extrinsic"))

        drift_section = section.get("complementary_drift", {})
        if isinstance(drift_section, dict):
            drift = settings.complementary_drift
            drift.alpha = float(drift_section.get("alpha", drift.alpha))
            drift.axis = str(drift_section.get("axis", drift.axis))

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
    source = {
        "path": settings.complementary_source.path,
        "max_match_dt_s": float(settings.complementary_source.max_match_dt_s),
    }
    extrinsic = settings.complementary_source.extrinsic
    if extrinsic is not None:
        tx, ty, tz, qx, qy, qz, qw = matrix_to_quat(extrinsic)
        source["extrinsic"] = {
            "translation": [float(tx), float(ty), float(tz)],
            "rotation_quat_xyzw": [float(qx), float(qy), float(qz), float(qw)],
        }
    return {
        "/**": {
            "ros__parameters": {
                "complementaryOdom": {
                    "scaleEstimationApply": bool(params.scale_estimation_apply),
                    "scaleMinNonDegenerateSpeed": float(params.scale_min_nondegenerate_speed),
                    "scaleBaselineFrameLag": int(params.scale_baseline_frame_lag),
                    "scaleSmoothingWindowSize": int(params.scale_smoothing_window_size),
                    "ignore_dz": bool(params.ignore_dz),
                    "translationScale": float(params.translation_scale),
                },
            },
        },
        "replay_scale_tool": {
            "scale_mode": settings.scale_mode,
            "scales": [float(s) for s in settings.scales],
            "output_dir": settings.output_dir,
            "output_subdir": settings.output_subdir,
            "no_correction": bool(settings.no_correction),
            "validate": bool(settings.validate),
            "correction_mode": settings.correction_mode,
            "complementary_source": source,
            "complementary_drift": {
                "alpha": float(settings.complementary_drift.alpha),
                "axis": settings.complementary_drift.axis,
            },
        },
    }


def dump_config(settings, params):
    """Serialize settings + params to config YAML text."""
    _require_yaml()
    return yaml.safe_dump(config_to_mapping(settings, params), sort_keys=False)


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
    raw = _read_yaml(yaml_path)
    settings = replay_tool_settings_from_mapping(
        raw.get("replay_scale_tool", {}) if isinstance(raw, dict) else {})
    params = replay_params_from_mapping(_extract_ros_parameters_root(raw))
    return settings, params
