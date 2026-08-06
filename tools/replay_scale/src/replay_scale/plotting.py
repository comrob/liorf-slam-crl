"""Figure construction for replay trajectories.

Builds and returns matplotlib ``Figure`` objects; it never calls ``show()`` or
``savefig()``. The CLI saves what it gets back, a GUI embeds it in a canvas.

matplotlib is imported lazily so that importing :mod:`replay_scale` -- and
therefore running the replay itself -- does not require it.
"""

import glob
import os

from .core.estimator import reconstruct_complementary_only
from .io.tum import load_tum

#: Colors cycled through for replay curves, in order.
REPLAY_COLORS = ("tab:orange", "tab:red", "tab:purple", "tab:brown",
                 "tab:pink", "tab:gray", "tab:olive")
REFERENCE_COLOR = "tab:blue"
AUXILIARY_COLOR = "tab:green"

#: Curve kinds accepted by build_trajectory_figure.
KINDS = ("reference", "replay", "auxiliary")


def _xy(trajectory):
    xs = [T[0, 3] for _, T in trajectory]
    ys = [T[1, 3] for _, T in trajectory]
    return xs, ys


def label_from_tum_path(path):
    """Return (label, is_reference) derived from a trajectory_*.tum filename."""
    name = os.path.splitext(os.path.basename(path))[0]
    if name.startswith("trajectory_"):
        name = name[len("trajectory_"):]
    is_reference = name == "recorded_effective"
    if name.startswith("replay_"):
        name = name[len("replay_"):]
    return name.replace("_", " "), is_reference


def curves_from_tum_dir(traj_dir):
    """Load every ``*.tum`` in a trajectories/ folder as plottable curves.

    Returns a list of ``(label, trajectory, kind)``. Raises FileNotFoundError
    when the folder holds no trajectory files.
    """
    tum_paths = sorted(glob.glob(os.path.join(traj_dir, "*.tum")))
    if not tum_paths:
        raise FileNotFoundError(
            f"No .tum files found in {traj_dir}. "
            "Run replay-scale-trajectory first to populate it.")
    curves = []
    for tum_path in tum_paths:
        label, is_reference = label_from_tum_path(tum_path)
        curves.append((label, load_tum(tum_path), "reference" if is_reference else "replay"))
    return curves


def complementary_only_curve(frames, params):
    """The raw complementary odometry integrated on its own, as an aux curve.

    Not written to a file by the replay, so it is recomputed from the frames
    whenever it is plotted.
    """
    traj = reconstruct_complementary_only(
        frames, translation_scale_multiplier=params.translation_scale)
    return ("additional odometry (complementary, raw)", traj, "auxiliary")


def build_trajectory_figure(curves, *, title="", figsize=(9, 9)):
    """Top-down XY plot of the given curves; returns the Figure.

    ``curves`` is a sequence of ``(label, trajectory, kind)`` with kind in
    :data:`KINDS`. Trajectories are lists of ``(stamp, 4x4 pose)``.
    """
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=figsize)

    color_idx = 0
    for label, trajectory, kind in curves:
        xs, ys = _xy(trajectory)
        if kind == "reference":
            ax.plot(xs, ys, label=f"original ({label})", color=REFERENCE_COLOR, linewidth=1.5)
        elif kind == "replay":
            color = REPLAY_COLORS[color_idx % len(REPLAY_COLORS)]
            color_idx += 1
            ax.plot(xs, ys, label=f"replay ({label})", color=color, linewidth=1.5, linestyle="--")
        elif kind == "auxiliary":
            ax.plot(xs, ys, label=label, color=AUXILIARY_COLOR, linewidth=1.0, alpha=0.8)
        else:
            raise ValueError(f"Unknown curve kind {kind!r}; expected one of {KINDS}")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", linewidth=0.5)
    ax.legend()
    fig.tight_layout()
    return fig
