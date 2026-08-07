"""The GUI edits settings objects and can write them back out.

That only stays reproducible if what it writes reloads to what it had, which is
what these check. No Qt involved -- the panel produces plain settings objects.
"""

import numpy as np
import pytest

from replay_scale.core.se3 import quat_to_matrix
from replay_scale.settings import (
    ComplementaryDriftSettings,
    ComplementarySourceSettings,
    ReplayToolSettings,
    config_to_mapping,
    dump_config,
    load_config,
    replay_params_from_mapping,
    replay_tool_settings_from_mapping,
    save_config,
)
from replay_scale.core.model import ReplayParams

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

pytestmark = pytest.mark.skipif(yaml is None, reason="PyYAML not installed")


def _edited():
    settings = ReplayToolSettings(
        scale_mode="estimated", scales=[0.9, 1.1], output_subdir="replay",
        no_correction=True, validate=True, correction_mode="translation",
        complementary_source=ComplementarySourceSettings(
            path="/tmp/t265.tum", max_match_dt_s=0.4, match_mode="interpolate"),
        complementary_drift=ComplementaryDriftSettings(alpha=0.12, axis="y"))
    params = ReplayParams(
        translation_scale=1.5, scale_estimation_apply=False,
        scale_min_nondegenerate_speed=0.25, scale_baseline_frame_lag=7,
        scale_smoothing_window_size=33, ignore_dz=True)
    return settings, params


def _reload(mapping):
    return (replay_tool_settings_from_mapping(mapping["replay_scale_tool"]),
            replay_params_from_mapping(mapping["/**"]["ros__parameters"]))


def test_every_edited_field_survives_a_round_trip():
    settings, params = _edited()
    back_s, back_p = _reload(config_to_mapping(settings, params))

    assert back_p == params
    assert back_s.correction_mode == settings.correction_mode
    assert back_s.scale_mode == settings.scale_mode
    assert back_s.scales == settings.scales
    assert back_s.no_correction == settings.no_correction
    assert back_s.validate == settings.validate
    assert back_s.output_subdir == settings.output_subdir
    assert back_s.complementary_source.path == settings.complementary_source.path
    assert back_s.complementary_source.max_match_dt_s == pytest.approx(0.4)
    assert back_s.complementary_source.match_mode == "interpolate"
    assert back_s.complementary_drift.alpha == pytest.approx(0.12)
    assert back_s.complementary_drift.axis == "y"


def test_extrinsic_is_written_in_the_nodes_own_spelling():
    """Saved as extrinsicTrans/extrinsicRot, so it can be pasted between configs."""
    settings, params = _edited()
    T = quat_to_matrix(0.1, -0.2, 0.3, 0.0, 0.0, 1.0, 0.0)
    settings.complementary_source.extrinsic = T
    source = config_to_mapping(settings, params)["replay_scale_tool"]["complementary_source"]

    assert source["extrinsicTrans"] == pytest.approx([0.1, -0.2, 0.3])
    assert len(source["extrinsicRot"]) == 9
    np.testing.assert_allclose(np.array(source["extrinsicRot"]).reshape(3, 3),
                               T[:3, :3], atol=1e-9)


def test_extrinsic_survives_a_round_trip_exactly():
    """The matrix is written as itself, so nothing is lost on the way out."""
    settings, params = _edited()
    T = quat_to_matrix(0.1, -0.2, 0.3, 0.0, 0.0, 1.0, 0.0)
    settings.complementary_source.extrinsic = T
    back_s, _ = _reload(config_to_mapping(settings, params))
    np.testing.assert_allclose(back_s.complementary_source.extrinsic, T, atol=1e-12)


def test_absent_extrinsic_is_omitted_rather_than_written_as_identity():
    settings, params = _edited()
    assert settings.complementary_source.extrinsic is None
    source = config_to_mapping(settings, params)["replay_scale_tool"]["complementary_source"]
    assert "extrinsicRot" not in source
    assert "extrinsicTrans" not in source
    assert "extrinsic" not in source


def test_written_file_reloads_through_the_normal_loader(tmp_path):
    settings, params = _edited()
    path = tmp_path / "edited.yaml"
    save_config(str(path), settings, params)

    back_s, back_p = load_config(str(path))
    assert back_p == params
    assert back_s.correction_mode == "translation"
    assert back_s.complementary_source.path == "/tmp/t265.tum"


def test_dump_is_plain_yaml_text():
    text = dump_config(*_edited())
    assert "replay_scale_tool" in text
    assert "ros__parameters" in text
    assert yaml.safe_load(text)["replay_scale_tool"]["correction_mode"] == "translation"


def test_unknown_match_mode_is_rejected():
    settings, _ = _edited()
    settings.complementary_source.match_mode = "linear"
    with pytest.raises(ValueError, match="match_mode"):
        settings.validated()


def test_unknown_drift_axis_is_rejected():
    settings, _ = _edited()
    settings.complementary_drift.axis = "forward"
    with pytest.raises(ValueError, match="complementary_drift.axis"):
        settings.validated()


def test_output_tag_separates_drift_sweeps():
    """A sweep over alpha must not have each run overwrite the last."""
    from replay_scale.io.paths import complementary_source_tag

    settings, _ = _edited()
    settings.correction_mode = "twist6"
    settings.complementary_source.path = ""
    tags = set()
    for alpha in (0.0, 0.05, 0.1, -0.1):
        settings.complementary_drift.alpha = alpha
        tags.add(complementary_source_tag(settings))
    assert len(tags) == 4
