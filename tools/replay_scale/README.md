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
uv sync --extra gui         # ...and the optional PySide6 viewer
```

Or into an environment you already have:

```bash
pip install -e tools/replay_scale          # CLI only
pip install -e 'tools/replay_scale[gui]'   # with the viewer
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

## Simulating a complementary-odometry error

`replay_scale_tool.complementary_drift` injects a *known* systematic error into
the complementary odometry before replaying. Each frame's displacement gains
`alpha × |displacement|` along the chosen body axis, applied to the raw twist —
before `translationScale` and before any estimated scale — so it behaves like a
sensor fault rather than a correction.

```yaml
replay_scale_tool:
  complementary_drift:
    alpha: 0.02     # 2% of distance travelled, per frame
    axis: y         # body axis it is added along (lateral by default)
```

Proportional to distance is the right model: slip on a legged or wheeled
platform accumulates with ground covered, not with time and not as white noise.

The axis decides what the experiment tests:

| Axis | What it is | What to expect |
| --- | --- | --- |
| `y` (lateral, default) | An error the scale cannot express — but it lands on the *non-degenerate* component the estimate is divided by. | It does **not** stay lateral. Inflating the denominator drives the estimated scale down, and that wrong scale is then applied to the along-tunnel motion. A sideways odometry error comes back out as a *longitudinal* one. This is a direct test of the method's core assumption: that observable cross-tunnel motion is a sound yardstick for unobservable along-tunnel motion. |
| `x` (along travel) | A pure scale error — the odometry reports `1 + alpha` times too far. | The estimator *should* recover it; with `alpha = 0.1` the estimate should move toward `1/1.1 ≈ 0.909`. A recovery check rather than a stress test. |

**Choosing alpha for the lateral case.** The injected offset is sized by the
*whole* displacement, but it corrupts only the non-degenerate part, so the
relative damage is roughly `alpha / (|t_comp_nondeg| / |t_comp|)`. On the
bundled example that ratio has a median of 0.52 over observable frames, so:

| alpha | approximate corruption of the scale denominator |
| --- | --- |
| 0.005 | ~1% |
| 0.01 | ~2% |
| 0.02 | ~4% |
| 0.05 | ~10% |
| 0.1 | ~19% |

Sweep the small end first — the interesting behaviour is well below `alpha =
0.1`. On a run with a straighter tunnel the ratio drops and the same alpha bites
considerably harder.

Outputs are tagged with the alpha used (`_drift_xp0.1`), so a sweep over alpha
leaves one trajectory per value instead of overwriting itself. The GUI exposes
the same two fields, so you can sweep alpha and watch the estimate move.

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

## GUI viewer

```bash
replay-scale-gui                      # opens the newest run under ~/.ros/lili_logs
replay-scale-gui ~/.ros/lili_logs/run_20260805_192954_kdtree_lm
```

A frame scrubber, not a second way to run a replay: it calls the same
`run_replay` the CLI does, with `write=False`, so it produces no files and can
do nothing the CLI cannot. It forces `scale_mode: estimated` — the per-frame
vectors it draws only exist on that path — and says so in the status bar.

Axes follow the robotics convention: **x up the page, y to the left**.

Each frame is drawn with the **lagged anchor pose at the origin**. Three frame
choices, selectable in the toolbar:

- **`robot (comp forward)` (default)** — turn the whole picture back by the
  complementary vector's own angle, so the complementary displacement points
  straight up on every frame. This removes both the run's heading changes and
  its out-and-back reversal, so the degenerate direction can be compared across
  frames instead of spinning with the robot. Frames with no complementary
  vector have no angle to align to and fall back to map orientation, noted in
  the title.
- **`map`** — translate only; axes stay map-aligned, so the picture lines up
  with the trajectory plot and with the tunnel.
- **`robot (anchor)`** — rotate by `R_anchor`ᵀ so the anchor's forward is +x.
  This is the frame the estimator itself works in, but its orientation means
  something different on every frame.

| Mark | Meaning |
| --- | --- |
| Blue dot at origin | The anchor pose, `scaleBaselineFrameLag` frames back. |
| Green arrow | Complementary displacement over the lag window (`t_comp_map`), rooted at the anchor. Absent when the window did not close. |
| Orange dashed line | Degenerate translational direction(s) through the latest position — where LiDAR constrains nothing and the complementary prediction is substituted. |
| Orange dotted lines | The previous degenerate frames' lines (`history`, default 5, taking every `step`-th, default 3), fading with age. |

The history overlay is not simply the last *N* frames redrawn — each earlier
line was computed against **its own** anchor, and how it is placed depends on
the mode:

- **In metres**, earlier lines are re-referenced onto the current frame's
  origin, so the overlay shows how the degenerate direction and the robot's
  position genuinely moved relative to each other.
- **With `|comp| = 1`**, they cannot be: each frame is expressed in units of
  *its own* complementary length, so an offset measured against the current
  anchor divided by a different frame's length would mean nothing. Instead
  every overlaid frame is drawn in its own normalized geometry — its own anchor
  at the origin, its own complementary vector at unit length. Superimposed,
  that shows the spread of the scale ratio and of the degenerate direction over
  the last *N* frames.

Consecutive frames barely differ, so **step** (default 3) strides through the
earlier degenerate frames rather than redrawing almost the same line five
times — with the defaults the overlay spans the last 15 degenerate frames.
Frames without a usable basis are skipped, so a history of 5 always shows 5
real lines. **Observable only** applies here too: with it checked, the overlay
only looks back at frames that passed the gate, so it never mixes a meaningful
line with a noise-dominated one. It restricts the overlay only — the line for
the frame you are actually looking at is always drawn. When normalizing, frames
that had no complementary window have nothing to divide by and are dropped, so
fewer than *N* may appear; the legend reports how many were actually drawn. Set
it to 0 to turn the overlay off.

The **coverage strip** under the slider is two rows, because the two facts are
independent: complementary-window state on top (none / window / gate-observable,
light to dark), degeneracy below. With an alternative odometry source most
degenerate frames have no window — on the bundled example that is 2699 of 3029
frames — so a single-row strip would hide exactly the interesting population.
Click or drag it to seek.

**Zoom** defaults to *fixed (whole run)*: one extent, snapped to a round ladder
step, covering every frame — so the axes never move and vector lengths are
comparable between frames. Be aware these runs are bimodal (median window
displacement 0.11 m, upper quartile 0.85 m), so on the slow stretches the
vectors are short at that scale; *auto* re-snaps per frame if you want to see
them, at the cost of frames no longer being comparable. Fixed steps from the
ladder are also selectable.

**|comp| = 1** divides the whole frame through by the length of the
complementary vector, so it always draws at unit length (a faint unit circle
marks it) and the axes become dimensionless. The distance from the origin to
the latest LiDAR position then reads directly as **|LiDAR| / |comp|** — the
ratio the scale estimate is built from — so you can see at a glance whether a
frame is arguing for a scale above or below 1, without comparing two arrow
lengths by eye. Frames with no complementary window have nothing to divide by
and stay in metres, with a note in the title. It is applied at draw time, so
toggling it is instant and the underlying geometry is untouched. Under
normalization the *fixed* zoom switches to ±2 in units of |comp|.

**Observable only** is on by default: the slider only lands on — and the
history overlay only looks back at — frames whose scale sample passed the
estimator's observability gate
(`complementaryOdom.scaleMinNonDegenerateSpeed`). Because that gate is a *speed*
gate, it is exactly the filter that removes noise-dominated frames. On the
bundled example it keeps 1821 of 3819 frames and lifts the median window
displacement from 0.11 m to 0.89 m; frames under 0.15 m fall from 54% to 7%.
Uncheck it to reach every frame.

### Editing the configuration

The **Configuration** dock edits the same `complementaryOdom` parameters and
`replay_scale_tool` settings the YAML carries — `translationScale`,
`scaleMinNonDegenerateSpeed`, `scaleBaselineFrameLag`,
`scaleSmoothingWindowSize`, `scaleEstimationApply`, `ignore_dz`,
`correction_mode`, and the complementary source path and match gate.

- **Apply & re-run** replays with the edited values (on the worker thread, still
  writing nothing) and redraws.
- **Revert** returns to the configuration the current view was produced with —
  so a failed run does not lose your baseline.
- **Save YAML…** writes those values as a config file, and the status bar shows
  the `replay-scale-trajectory … --ros-params-yaml <file>` command that
  reproduces the same replay headlessly.

The panel produces the very same `ReplayToolSettings` / `ReplayParams` objects
the YAML loader does, so it cannot express a configuration the CLI could not
run. `scale_mode` is deliberately not editable: the viewer needs the estimator's
per-frame vectors, which only that path produces. Note that saving writes
*values*, not the document — comments in a hand-edited config are not preserved,
so save to a new file if you want to keep the annotated original.

### Reading the vectors

Two properties of real runs make the view look wrong when it isn't, and are the
reason the filter above defaults to on.

**Most frames barely move.** Unfiltered, 54% of windows on the bundled example
displace under 0.15 m — about 1 cm/frame. At that speed direction is dominated
by noise and by rotation about a point offset from the body origin, so the
complementary arrow wanders and often points sideways. Real motion, faithfully
drawn, and meaningless.

**A robot that reverses looks like a sign error.** The bundled example walks out
and back *without turning around* — two legs of 783 and 738 frames, 152.6 m of
path for 0.9 m of net displacement, heading smooth throughout (max 4.2°/frame,
no jumps). So the vector points one way for the first half of the run and the
opposite way for the second. Before concluding a vector is wrong, check the
frame's displacement and whether the run has a return leg.

A useful invariant when something does look off: the complementary arrow and
the LiDAR position reach the plot through independent paths, and on a healthy
run they agree to about 1° in direction. A frame-convention bug shows up as
those two disagreeing — not as both pointing somewhere unexpected together.

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
| `core/` | SE(3) math, data model, estimator, odom sync, `local_view` | Pure computation. Reads no files, writes no files, prints nothing. |
| `io/` | CSV / TUM readers and writers, run-directory paths | Serialization only. May use `core.model` and `core.se3`, never `core.estimator`. |
| `settings.py` | Settings objects and the YAML that fills them | Named `settings` because `config/` is the data directory. |
| `pipeline.py` | `run_replay()` — CSV in, trajectories out | The seam every frontend goes through. Never prints. |
| `plotting.py` | `build_trajectory_figure()`, `draw_local_frame()` | Never calls `show()` or `savefig()`. |
| `cli.py` | argparse, printing, exit codes | Peer of `gui/`. |
| `gui/` | PySide6 viewer; `sources.py` is Qt-free | Calls `run_replay` directly, never shells out to the CLI. |

The GUI calls `run_replay` rather than subprocessing the CLI. Shelling out
would force results back through files, make progress and cancellation awkward,
and require serializing every parameter change to a temp YAML. The invariant
worth keeping is *the GUI cannot do anything the CLI cannot*, and sharing
`pipeline.run_replay` gives that without the round trip.

### The view seam

`core/local_view.py` has two stages, and the split is the thing that lets a
second data source be added cheaply later:

- **`FrameGeometry`** — a neutral per-frame record: everything the view needs,
  in map frame, rotations as 3×3 matrices. One **adapter** per data source
  produces these. Today there is one, `geometry_from_replay`; a reader for
  `scale_replay_vectors.csv` would be a second adapter.
- **`build_local_frame_views`** — the single **builder**, projecting geometry
  into the anchor frame. There must never be a second one: two builders is how
  two sources silently drift apart while both pass their own tests.

Quaternions deliberately do not appear in `FrameGeometry`. They are a
serialization concern; letting them in would import a future file format back
into the computation layer. `has_window` is likewise an explicit flag rather
than a NaN check, so every adapter agrees on the answer.

The estimator owns the anchor rule. `ScaleVectorFrame.anchor_frame_idx` is
recorded unconditionally — the anchor *pose* exists even on frames where the
complementary window never closed — so no frontend re-derives
`k - scaleBaselineFrameLag`. Note that degeneracy and window-validity are
independent: `degenerate_axes_map` must be available on every degenerate frame,
not only on frames that produced a scale sample.

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
