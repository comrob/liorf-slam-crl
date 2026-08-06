# replay_scale

Offline replay of LILI's complementary-odometry scaling and degeneracy pose
correction. Reads the per-frame diagnostics a run recorded and reconstructs the
map-frame trajectory it *would* have produced under different scale settings,
different correction rules or a different odometry source — without re-running
SLAM.

The estimator here is a port of
[`src/mapOptimization/mapOptimization_degeneracy.cpp`](../../src/mapOptimization/mapOptimization_degeneracy.cpp).
It must track that file; see [Relation to the node](#relation-to-the-node).

## Install

Self-contained project with its own environment:

```bash
cd tools/replay_scale
uv sync                     # creates .venv and installs the tool + dev deps
```

Or into an environment you already have:

```bash
pip install -e tools/replay_scale
```

## Quickstart

```bash
# replay the newest run under ~/.ros/lili_logs
replay-scale-trajectory --latest

# ...or a specific run directory
replay-scale-trajectory ~/.ros/lili_logs/run_20260805_192954_kdtree_lm

# plot everything the replay wrote, plus the raw complementary odometry
replay-scale-plot-trajectories ~/.ros/lili_logs/run_20260805_192954_kdtree_lm
```

Outputs land in `<run>/replay/` — see [Outputs](#outputs). Run the replay more
than once with different settings before plotting: the plot draws every `.tum`
it finds, so successive replays stack up as curves to compare.

## Input

Produced by `LiliDiagnostics` when `log.diagnostics.enable_scale_replay` is
true (see [`config/anymal.yaml`](../../config/anymal.yaml)). A run directory
holds:

| File | Role |
| --- | --- |
| `scale_replay_frames.csv` | **Required.** One row per LiDAR frame: poses, LiDAR increment, complementary twist, degeneracy flags and basis, applied scale. |
| `complementary_odom_meta.yaml` | `T_complementary_to_lidar` extrinsic, used when replaying an alternative odometry source. |
| `complementary_odom_stream.tum` | The raw odometry stream the online run consumed. Can be fed back in as an alternative source. |

Both positional forms work: pass the run directory, or the CSV itself.

## CLI

```
replay-scale-trajectory [input] [--latest] [--base-dir DIR] [--ros-params-yaml YAML]
replay-scale-plot-trajectories [same] [--output PNG] [--show]
```

| Flag | Meaning |
| --- | --- |
| `input` | Run directory or path to `scale_replay_frames.csv`. Omit to use `--latest`. |
| `--latest` | Use `<base-dir>/latest`, else the newest `run_*` by mtime. |
| `--base-dir` | Where `run_*` folders live. Default `~/.ros/lili_logs`. |
| `--ros-params-yaml` | Config file. Default: bundled [`config/default.yaml`](src/replay_scale/config/default.yaml). |
| `--output` | Plot only. PNG destination. Default `<out>/trajectories/trajectories_2d.png`. |
| `--show` | Plot only. Also open an interactive window. |

Everything else — scale mode, correction mode, output location, alternative
odometry source — is set in the YAML rather than on the command line, so a
replay is fully described by one file you can keep next to your results. Copy
the bundled default and edit it:

```bash
cp src/replay_scale/config/default.yaml my_replay.yaml
replay-scale-trajectory --latest --ros-params-yaml my_replay.yaml
```

## Configuration

The file carries two independent trees. `/**: ros__parameters:
complementaryOdom:` mirrors the node's own parameters; `replay_scale_tool:` is
this tool's own settings and is ignored by any ROS node loading the same file.
That means a run's `run_parameters.yaml` can be used directly to replay with
the exact parameters that run used.

Every key is documented inline in
[`config/default.yaml`](src/replay_scale/config/default.yaml). The two choices
that matter most:

**`scale_mode`** — where the translation scale applied in degenerate directions
comes from.

| Mode | Behaviour |
| --- | --- |
| `estimated` | Re-run the online estimator's logic over the replayed poses, using the `complementaryOdom` parameters. This is what you change to tune the estimator offline. |
| `recorded` | Replay with the per-frame scale the online run actually applied. |
| `fixed` | Apply each constant in `scales:`, producing one trajectory per value. |

**`correction_mode`** — how the degeneracy override is applied.

| Mode | Behaviour |
| --- | --- |
| `twist6` | Node parity: projects the full 6D twist onto the degenerate basis. The inner product mixes metres and radians, so residual angular content in the basis turns the correction into a rotation. |
| `translation` | Projects only translation onto the degenerate directions, leaving orientation to LiDAR. Use where rotation is observable (tunnel walls constrain yaw) and any injected heading is therefore spurious. |

Set `validate: true` to additionally replay with the recorded scale and report
position drift against the pose the online run used — a check that the replay
chain reproduces the original run. `no_correction: true` also emits the
LiDAR-only trajectory with no degeneracy override at all.

## Replaying a different odometry source

`complementary_source.path` takes a TUM file of absolute odometry poses. The
replay re-synchronizes it against the same LiDAR frame stamps using the same
nearest-sample matching the node performs, so it is a fair substitution rather
than a re-timed one.

```yaml
replay_scale_tool:
  complementary_source:
    path: "/path/to/t265.tum"
    max_match_dt_s: 0.25          # reject matches farther than this, as the node does
    # extrinsic:                  # omit to reuse the run's complementary_odom_meta.yaml
    #   translation: [0.0, 0.0, 0.0]
    #   rotation_quat_xyzw: [0.0, 0.0, 0.0, 1.0]
```

Set `extrinsic` when the alternative sensor sits on a different mount than the
one the run recorded. Frames whose match fails a gate lose their complementary
twist, and get no degeneracy override — a source covering only part of the run
replays only that part, which is expected and reported in the output:

```
  matched frames: 774/3819
```

## Outputs

Written under `<output_dir or the CSV's directory>/<output_subdir>/`, default
`<run>/replay/`:

```
replay/
├── trajectories/
│   ├── trajectory_recorded_effective.tum      the pose the online run used (the reference)
│   ├── trajectory_replay_<tag><src>.tum       one per replayed configuration
│   ├── trajectory_replay_validate.tum         only when validate: true
│   ├── trajectory_replay_lidar_only.tum       only when no_correction: true
│   └── trajectories_2d.png                    written by the plot command
└── log/
    ├── scale_replay_estimator_trace<src>.csv  per-frame gate / raw / smoothed / applied scale
    └── scale_replay_vectors<src>.csv          per-frame geometry behind each scale sample
```

`<tag>` is `estimated_scale`, `recorded_scale`, or `scale_<value>` per fixed
scale. `<src>` is a suffix identifying the configuration — `_src_<name>` for an
alternative odometry source and `_corr_<mode>` for a non-default correction
mode. It exists so replays of different configurations sit side by side instead
of overwriting each other, which is what makes them plottable against one
another.

The two `log/` traces are only written in `estimated` mode; they are the raw
material for diagnosing why the estimator settled where it did.

## Python API

Both CLI commands are thin wrappers over one function, and any other frontend
is expected to use the same one:

```python
from replay_scale import DEFAULT_CONFIG_PATH, load_config, run_replay

settings, params = load_config(DEFAULT_CONFIG_PATH)
settings = settings.evolve(scale_mode="fixed", scales=[0.9, 1.0, 1.1])

result = run_replay("run_.../scale_replay_frames.csv", settings, params,
                    write=False, on_progress=print)

for tag, trajectory in result.replays:
    ...  # trajectory is [(stamp, 4x4 pose), ...], in memory
```

`write=False` computes without touching the filesystem. `on_progress` receives
the status lines the CLI prints. `ReplayResult` also carries `frames`,
`recorded_effective`, `lidar_only`, `validation` (`.mean` / `.max` / `.rms`),
`scale_trace`, `vector_trace` and `written`.

Plotting is separate and returns a figure rather than saving one:

```python
from replay_scale.plotting import build_trajectory_figure, complementary_only_curve

fig = build_trajectory_figure(
    result.curves() + [complementary_only_curve(result.frames, params)],
    title="my replay")
```

## Architecture

Layers, innermost first. Each may import the ones above it, **never** the ones
below:

| Layer | Contents | Rule |
| --- | --- | --- |
| `core/` | SE(3) math, data model, estimator, odom sync | Pure computation. Reads no files, writes no files, prints nothing. |
| `io/` | CSV / TUM readers and writers, run-directory paths | Serialization only. May use `core.model` and `core.se3`, never `core.estimator`. |
| `settings.py` | Settings objects and the YAML that fills them | Named `settings` because `config/` is the data directory. |
| `pipeline.py` | `run_replay()` — CSV in, trajectories out | The seam every frontend goes through. Never prints. |
| `plotting.py` | `build_trajectory_figure()` — curves in, `Figure` out | Never calls `show()` or `savefig()`. |
| `cli.py` | argparse, printing, exit codes | Peer of any future `gui/`. |

A GUI belongs at `gui/`, alongside `cli.py`, calling `run_replay` directly —
not shelling out to the CLI. Subprocessing would force results back through
files, make progress and cancellation awkward, and require serializing every
parameter change to a temp YAML. The invariant worth keeping is *the GUI cannot
do anything the CLI cannot*, and sharing `pipeline.run_replay` gives that
without the round trip. A GUI should show the equivalent YAML and command line
so any session stays reproducible headlessly.

[`tests/test_layering.py`](tests/test_layering.py) enforces the import
direction by walking the AST, so this stays true.

## Tests

```bash
uv run pytest
```

`test_layering.py` runs anywhere. `test_pipeline_smoke.py` needs a real run and
skips without one; point it at yours with:

```bash
REPLAY_SCALE_TEST_RUN=~/.ros/lili_logs/run_20260805_192954_kdtree_lm uv run pytest
```

## Relation to the node

This is a **port**, not a shared implementation — the online logic lives in C++
and is reimplemented here in Python. Parity is deliberate and documented at the
call sites:

- `core/estimator.py` mirrors `buildAdditionalOdomCorrectionResult`, including
  the observability gate, the smoothing-history update rule, and the causal
  ordering where the scale applied at frame *k* comes from history up to *k-1*.
- `core/odom_source.py` mirrors `inferComplementaryOdomTwist` — nearest-sample
  matching rather than interpolation, the same 0.25 s gate, and the same
  exclusion rule when two LiDAR stamps land on one odometry sample.

Two intentional divergences, both noted in the source:

- The degeneracy override additionally requires a complementary prediction to
  exist. The node leaves its prediction at the previous pose when a match fails
  and projects it anyway, substituting *zero motion* along the degenerate axis.
  That is unreachable online (its own odometry matches every frame) but routine
  when replaying a source with gaps, where it would make the trajectory slide
  sideways.
- Orientation is read from each frame's logged absolute pose rather than
  chained from the increments, which are referenced to the *online* corrected
  pose. Chaining them would accumulate orientation error the moment a replay
  departs from the online trajectory — which is the entire point of replaying.

When the C++ changes, change it here too.
