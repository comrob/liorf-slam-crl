"""Spelling the complementary source's mount out in a config.

The extrinsic is the one setting whose mistakes are invisible: a wrong mount
does not fail, it quietly rotates every complementary displacement and comes
back out as a scale. So these pin the convention (row-major, as the node reads
its parameters), and pin that a malformed one is refused rather than dropped.
"""

import numpy as np
import pytest
import yaml

from replay_scale.settings import (
    extrinsic_from_mapping,
    extrinsic_from_source_mapping,
    replay_tool_settings_from_mapping,
    rotation_from_sequence,
)

# Rotation by +90 deg about z. Asymmetric, so reading it column-major instead of
# row-major gives the inverse -- which is the mistake worth catching.
YAW_90 = np.array([[0.0, -1.0, 0.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]])


def _yaml(text):
    return yaml.safe_load(text)


# ---------------------------------------------------------------------------
# The node's spelling
# ---------------------------------------------------------------------------

def test_ros_parameter_spelling_is_accepted():
    section = _yaml("""
        path: "/tmp/vo.tum"
        extrinsicTrans: [-0.310, 0.0, 0.159]
        extrinsicRot: [-1.0, 0.0, 0.0,
                       0.0, -1.0, 0.0,
                       0.0, 0.0, 1.0]
    """)
    T = extrinsic_from_source_mapping(section)
    np.testing.assert_allclose(T[:3, 3], [-0.310, 0.0, 0.159])
    np.testing.assert_allclose(T[:3, :3], np.diag([-1.0, -1.0, 1.0]))


def test_nine_numbers_are_read_row_major():
    """``Eigen::Map<..., RowMajor>`` in utility.h; column-major would invert it."""
    np.testing.assert_allclose(
        rotation_from_sequence([0, -1, 0, 1, 0, 0, 0, 0, 1]), YAW_90)


def test_rows_may_be_nested_or_flat():
    flat = rotation_from_sequence([0, -1, 0, 1, 0, 0, 0, 0, 1])
    nested = rotation_from_sequence([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    np.testing.assert_allclose(flat, nested)


def test_the_keys_are_accepted_inside_an_extrinsic_block_too():
    section = _yaml("""
        extrinsic:
          extrinsicTrans: [1.0, 2.0, 3.0]
          extrinsicRot: [0, -1, 0, 1, 0, 0, 0, 0, 1]
    """)
    T = extrinsic_from_source_mapping(section)
    np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(T[:3, :3], YAW_90)


def test_either_key_may_be_omitted_as_the_node_allows():
    rot_only = extrinsic_from_mapping({"extrinsicRot": [0, -1, 0, 1, 0, 0, 0, 0, 1]})
    np.testing.assert_allclose(rot_only[:3, 3], np.zeros(3))
    np.testing.assert_allclose(rot_only[:3, :3], YAW_90)

    trans_only = extrinsic_from_mapping({"extrinsicTrans": [1.0, 2.0, 3.0]})
    np.testing.assert_allclose(trans_only[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(trans_only[:3, :3], np.eye(3))


def test_the_settings_loader_reaches_it_through_the_source_block():
    settings = replay_tool_settings_from_mapping(_yaml("""
        complementary_source:
          path: "/tmp/vo.tum"
          extrinsicTrans: [0.0, 0.0, 1.0]
          extrinsicRot: [0, -1, 0, 1, 0, 0, 0, 0, 1]
    """))
    np.testing.assert_allclose(settings.complementary_source.extrinsic[:3, :3], YAW_90)


# ---------------------------------------------------------------------------
# The meta file's spelling
# ---------------------------------------------------------------------------

def test_the_recorded_quaternion_form_is_still_accepted():
    """complementary_odom_meta.yaml is written by the node in this form."""
    T = extrinsic_from_mapping(_yaml("""
        translation: [-0.310, 0.0, 0.159]
        rotation_quat_xyzw: [0.0, 0.0, 1.0, 0.0]
    """))
    np.testing.assert_allclose(T[:3, 3], [-0.310, 0.0, 0.159])
    np.testing.assert_allclose(T[:3, :3], np.diag([-1.0, -1.0, 1.0]), atol=1e-12)


# ---------------------------------------------------------------------------
# What is refused
# ---------------------------------------------------------------------------

def test_an_absent_extrinsic_is_none_not_an_error():
    """Omitting it is the normal case: the run's own meta file is then used."""
    assert extrinsic_from_source_mapping({"path": "/tmp/vo.tum"}) is None
    assert extrinsic_from_mapping(None) is None


def test_an_unrecognized_extrinsic_block_is_refused():
    """Silently ignoring it would replay the run's mount and blame the odometry."""
    with pytest.raises(ValueError, match="extrinsicTrans/extrinsicRot"):
        extrinsic_from_mapping({"translation": [0, 0, 0],
                                "rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1]})


def test_a_rotation_of_the_wrong_length_is_refused():
    with pytest.raises(ValueError, match="9 numbers"):
        rotation_from_sequence([1, 0, 0, 0, 1, 0])


def test_a_translation_of_the_wrong_length_is_refused():
    with pytest.raises(ValueError, match="extrinsicTrans"):
        extrinsic_from_mapping({"extrinsicTrans": [1.0, 2.0]})


def test_a_matrix_that_is_not_orthonormal_is_refused():
    with pytest.raises(ValueError, match="orthonormal"):
        rotation_from_sequence([2, 0, 0, 0, 1, 0, 0, 0, 1])


def test_a_reflection_is_refused():
    """A single flipped sign is a plausible typo and not a rotation."""
    with pytest.raises(ValueError, match="reflection"):
        rotation_from_sequence([-1, 0, 0, 0, 1, 0, 0, 0, 1])


def test_a_hand_rounded_matrix_is_accepted():
    """Six decimals is what a calibration gets pasted in as."""
    R = rotation_from_sequence([-0.038648, -0.005344, -0.999239,
                                -0.963966, 0.263597, 0.035874,
                                0.263205, 0.964618, -0.015339])
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-3)
