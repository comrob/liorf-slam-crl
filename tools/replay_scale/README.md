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
| `input` | Run directory or path to `scale_replay_frames.csv`. Omit to use the config's `input_path`, then the latest run. |
| `--latest` | Use `<base-dir>/latest`, else the newest `run_*` by mtime. Ignores a configured `input_path`. |
| `--base-dir` | Where `run_*` folders live. Overrides the config's `base_dir`; default `~/.ros/lili_logs`. |
| `--ros-params-yaml` | Config file. Default: bundled [`config/default.yaml`](src/replay_scale/config/default.yaml). |
| `--output` | Plot only. PNG destination. Default `<out>/trajectories/trajectories_2d.png`. |
| `--show` | Plot only. Also open an interactive window. |

Everything else — scale mode, correction mode, output location, alternative
odometry source — is set in the YAML rather than on the command line, so a
replay is fully described by one file you can keep next to your results. That
includes *which run* to replay:

```yaml
replay_scale_tool:
  input_path: "~/.ros/lili_logs/run_20260805_192954_kdtree_lm"   # "" = newest run
  base_dir: ""                                                   # "" = ~/.ros/lili_logs
```

Resolution is the same for the CLI and the GUI: an explicit path on the command
line wins, then `input_path`, then the newest run under the base directory —
the `latest` symlink if there is one, else the most recently modified `run_*`.
`--latest` forces that last step regardless of what the config names, and
`--base-dir` overrides `base_dir`. Leaving both keys empty is exactly the old
behaviour, so pinning a run is opt-in.

Copy the bundled default and edit it:

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

### The observability gate

**`scaleMinNonDegenerateSpeed`** decides which frames produce a usable scale
sample. In replay it is measured on the LiDAR's own non-degenerate displacement
over the LiDAR window:

```
|t_lidar_nondeg| / dt_lidar  >=  scaleMinNonDegenerateSpeed
```

The node instead uses `|t_lidar_nondeg_proj| / dt_complementary` — the LiDAR
displacement *projected onto the complementary non-degenerate direction*, over
the complementary window. Both halves of that carry the odometry: the projection
shortens by cos θ between the two vectors, and the divisor is the odometry's own
matched interval. A frame where the LiDAR moved plenty is then called
unobservable because the *complementary* vector was short, noisy or misaligned —
which inverts what the gate is for. It asks whether enough motion was **observed**
to measure a ratio against, and it should not be answered with the quantity under
test; those are exactly the frames worth looking at.

The scale ratio itself is unchanged and still uses the projection — only the gate
moved. On a 6519-frame run at `scaleMinNonDegenerateSpeed: 0.1` this admits 3755
samples where the node's rule admits 3648; at 0.2 the two agree to within a
couple of frames. The difference is small on well-aligned data and grows exactly
where the complementary odometry is poor.

**`complementaryCorrection`** — what the complementary displacement is corrected
by before it is substituted into the degenerate directions. (Spelled
`scaleSampleSource` before one of them stopped being a scale; the old key and
its `line_meet` value still load, as `lines_meet_x`.)

| Correction | What it measures | Applies |
| --- | --- | --- |
| `ratio` | Node parity: `\|lidar_nondeg projected on comp\| / \|comp_nondeg\|`. One frame measured against its own odometry. | a scale |
| `line_x_axis` | Where that same frame's degenerate line crosses the complementary axis, in units of `\|comp\|`. | a scale |
| `lines_meet_x` | The along-complementary coordinate of the point where the recent degenerate lines meet. | a scale |
| `lines_meet_xy` | That whole point: the displacement is moved onto it. | a scale **and** a rotation |

All four are read in the same normalized geometry — every frame divided by its
own `|comp|` and rotated so that vector is +x, which is exactly what the viewer
draws with `|comp| = 1`. In that frame the complementary displacement is
`(1, 0)`, and each degenerate line is a statement about the same quantity: *the
robot is somewhere along here, in units of what the odometry claimed*.

**`line_x_axis`** is the ratio, measured on the picture. Follow one frame's own
line to where it crosses `y = 0` and that x *is* the scale. With normal `n`
perpendicular to the degenerate direction, the node computes `|p·n| / |comp·n|`
and this computes `(p·n) / (comp·n)` — the same two numbers, quotiented in the
same order. On 500 random planar geometries the two agree to `1e-11` wherever
the crossing is positive. The difference is the sign: taking norms first folds a
*backwards* crossing onto the positive side, so where the geometry says the
robot moved the other way along the odometry's direction, the node reports the
distance as a scale and this reports no sample at all. That is a third of the
samples on the bundled run (2233 of 6480), and their median value is 2.34 — the
population that pushes the node's estimate up. Use it to find out how much of an
estimate is that artefact; it needs no history and no turn, so it costs nothing.

**`lines_meet_*`** is a genuinely different measurement, not a smoothing of the
first. The ratio needs the LiDAR displacement to be observable in the direction
being scaled; where the lines *meet* needs them to have turned relative to each
other, which is a property of the trajectory rather than of one frame. So it
says nothing on a straight stretch (`nan`, no sample) and answers where the
ratio is weakest.

`scaleLineHistory` and `scaleLineHistoryStep` set which earlier lines the fit
sees: `history` of them, taking every `step`-th, so the window reaches back
`history × step` frames. **Span is what matters, not count** — consecutive
frames' lines are nearly identical, and it is the turning between them that
makes them meet at all. The same rule as the viewer's overlay controls, and
`50 × 4` is the same window it draws. Per-frame samples on the bundled
6519-frame run, all with `l1`:

| Correction | history | step | span | lines | samples | median | p10–p90 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `ratio` | — | — | — | — | 6480 | 0.843 | 0.18–8.86 |
| `line_x_axis` | — | — | — | 1 | 4247 | — | — |
| `lines_meet_x` | 50 | 1 | 50 | 51 | 2225 | 1.552 | 0.21–9.47 |
| `lines_meet_x` | 200 | 1 | 200 | 201 | 5983 | 0.443 | 0.17–1.38 |
| `lines_meet_x` | 50 | 4 | 200 | 51 | 5959 | 0.444 | 0.17–1.35 |
| `lines_meet_x` | 50 | 8 | 400 | 51 | 5908 | 0.432 | 0.15–1.24 |

A stride of 4 reproduces the 200-frame result with a quarter of the lines, and
a third off the replay time. Fifty consecutive frames barely turn at all, which
is why that row is the worst of them.

With enough span the two sources agree on the median (0.44 against 0.84 raw,
0.48 against 0.49 once smoothed) while `lines_meet_x` produces a far tighter
distribution — which is the point of it.

### Correcting sideways as well: `lines_meet_xy`

The meeting point has two coordinates and `lines_meet_x` reads one of them. The
`y` is not noise: it is how far *sideways* of its own direction the lines say
the robot went, and **no scale can express it** — a scale only makes the
odometry's own vector longer or shorter.

`lines_meet_xy` applies the whole point. With the frame's complementary
displacement `v`, its in-plane length `d`, its own direction `ê_x = v/d` and the
left normal `ê_y`:

```
(cx, cy)      = the meeting point, in units of |comp|
v_corrected   = d · (cx · ê_x + cy · ê_y)
```

which is simply the vector *to the meeting point*, back in metres: the
odometry's arrow moved onto where the lines say the robot ended up. Equivalently
it scales `v` by `|(cx, cy)|` and turns it by `atan2(cy, cx)`. `cy = 0` recovers
`lines_meet_x` exactly in the plane; `z` is left alone, since the fit is 2D and
has nothing to say about it (a scale, by contrast, scales `z` too).

It is applied per frame, to each one-frame step in that step's own frame — the
same assumption the overlay it is read from already makes. Superimposing frames
after normalizing each by its own `|comp|` and turning each by its own odometry
angle is only meaningful if the error is a fixed multiple of `|comp|` in a fixed
direction relative to the odometry, i.e. a per-step body-frame quantity. It is
also exactly the form [the drift simulator](#simulating-a-complementary-odometry-error)
injects an error in, which makes the two directly comparable.

**The recovery check.** Inject `alpha = -0.2` on the body `y` axis and the
odometry's direction is wrong by `atan(0.2) = 11.3°`. The lines see it: on the
bundled run the median `cy` moves from `-0.005` to `+0.078`, and `cy / cx =
0.078 / 0.408 = 0.19` — the injected `tan(11.3°) = 0.20`, recovered. (`cy`
scales with `cx`, not with 1: the sideways offset is proportional to how far the
robot actually went, not to how far the odometry claimed.) `lines_meet_x` reads
the same geometry and can only report a slightly shorter scale.

How much of the injected error each correction absorbs, as the distance between
the drifted trajectory and its own undrifted one:

| Correction | alpha | mean | rms | max | endpoint |
| --- | --- | --- | --- | --- | --- |
| `ratio` | −0.2 | 25.71 | 30.15 | 54.78 | 10.45 |
| `ratio` | +0.2 | 22.39 | 27.23 | 52.08 | 6.21 |
| `lines_meet_x` | −0.2 | 1.66 | 2.24 | 4.16 | 4.07 |
| `lines_meet_x` | +0.2 | 1.58 | 1.86 | 3.60 | 2.17 |
| `lines_meet_xy` | −0.2 | 2.30 | 2.53 | 4.10 | 3.05 |
| `lines_meet_xy` | +0.2 | 1.01 | 1.26 | 2.47 | 0.26 |

The ratio is an order of magnitude more sensitive than either line method, as
the `axis: y` note predicts. Between the two line methods the result is honestly
mixed: at `+0.2` the vector correction absorbs most of what is left (endpoint
2.17 m → 0.26 m), at `−0.2` it helps at the endpoint (4.07 → 3.05) and is worse
in the mean (1.66 → 2.30). It is a real effect and not a uniform improvement;
sweep it on your own run before trusting it.

**`scaleLateralMax`** bounds `|cy|` before it enters the smoothing filter, the
way `scaleMin`/`scaleMax` bound the along-track coordinate, and clamps rather
than drops for the same reason. It is a bound on *how far the correction may
turn the displacement*: 0.2 is about 11°, 0.5 about 27°. Only `lines_meet_xy`
reads it. The two coordinates go through the smoothing window **as a pair** —
one observation of where the robot is, and smoothing the halves over different
windows would apply a mixture of two answers.

Where a frame produces no sample at all — the lines too parallel to meet, or the
speed gate — nothing is appended and the filter keeps applying the last pair the
window agreed on. Holding, not falling back to "no correction": a `cy` reset to
zero would be an assertion that the sideways error had vanished.

**`scaleLineFitNorm`** — what that fit minimises over the perpendicular
distances: `l2` (least squares, closed form) or `l1` (least absolute deviations,
by IRLS). These lines are not equally trustworthy — a frame whose degenerate
direction came from a poor basis contributes a line that is simply wrong — and a
squared cost lets such a line pull the answer in proportion to how wrong it is.
`l1` bounds each line to one vote, the same argument as the median above. On ten
lines through one point plus one badly wrong line, `l2` lands 0.5 away and `l1`
lands on the point. It also decides how the anchor view draws the meeting point,
so what you see is what the estimator would use.

**`scaleSmoothingMode`** — which statistic the smoothing window collapses to.

| Mode | Behaviour |
| --- | --- |
| `mean` | Node parity: the arithmetic mean of the window. One 20× sample shifts the applied scale by 0.4 on a 50-wide window, and keeps it shifted for the next 50 frames. |
| `median` | The middle sample. A new observation moves it by at most one order statistic, always towards the side it fell on — so a sustained change arrives in full, while a lone excursion moves nothing but its own vote. Robust, but half the window only votes on which side the middle lies. |
| `trimmed` | Quartiles locate the bulk, samples outside a 1.5 × IQR Tukey fence are dropped, and the rest are averaged. On a clean window the fence catches nothing and this *is* the mean; on a window with a tail it is the mean of the inliers. Falls back to the median for windows shorter than four samples, where quartiles mean nothing. |

The samples are a ratio of two short displacements, so the distribution is
heavy-tailed and this is not a cosmetic choice. On a window of
`[1.0, 1.1, 0.9, 1.05, 0.95, 20.0]` the three give 4.17, 1.03 and 1.00; on a
clean window they agree exactly. Over the bundled 6519-frame run:

| Mode | applied median | p90 | max |
| --- | --- | --- | --- |
| `mean` | 0.741 | 4.73 | 24.26 |
| `median` | 0.493 | 1.69 | 2.81 |
| `trimmed` | 0.494 | 2.11 | 6.09 |

`trimmed` sits where you would expect: it agrees with the median on where the
bulk is, and its wider tail is windows whose *bulk* really was high — not
outliers, and not something a filter should hide.

All three read the same trailing window, so all three are causal: the window
holds past samples only, and the value applied at frame *k* is built from
samples up to *k−1*, which is the node's own ordering. The bundled config
selects `trimmed`; `mean` is the default in code so an unconfigured replay still
reproduces the node.

**`scaleMin` / `scaleMax`** — the range a scale sample is **clamped into**
before it enters the smoothing filter:

```yaml
      scaleMin: 0.5
      scaleMax: 2.0
```

Samples are *clamped, not dropped*. A 12× sample still says the LiDAR moved
much further than the odometry claimed, and discarding it would let the filter
average on as though the frame had never happened; capping it keeps the vote
while limiting how far one frame can pull the mean. The applied scale is the
mean of clamped samples, so it stays inside the range as well.

`scale_instant_raw` is logged as measured either way, so the estimator trace and
the viewer's **Scale** tab still show what each frame really produced — the raw
curve running outside the dashed bound lines is exactly the population being
clamped.

Clamping is independent of observability: a sample still has to pass
`scaleMinNonDegenerateSpeed` to enter the filter at all, and the bounds do not
change which frames count as observable. This is not a node parameter — the node
has no such limit, so the defaults (`0.0` and `.inf`) clamp nothing and preserve
parity.

The two knobs overlap. Under `scaleSmoothingMode: median` an outlier already
counts only as one vote regardless of its size, so clamping changes little — on
the bundled run, `[0.5, 2.0]` moves the median-filtered p90 not at all (1.690
either way) and only caps the extreme (2.81 → 2.00). Under `mean` the same
bounds matter a great deal (p90 4.73 → 1.70). Reach for the bounds when you need
a hard guarantee on what can be applied; reach for the median first.

`scaleLateralMax` is the same idea for the cross-track coordinate under
`lines_meet_xy` — symmetric, since left and right are the same size of error.
See [Correcting sideways as well](#correcting-sideways-as-well-lines_meet_xy).

Set `validate: true` to additionally replay with the recorded scale and report
position drift against the pose the online run used — a check that the replay
chain reproduces the original run. `no_correction: true` also emits the
LiDAR-only trajectory with no degeneracy override at all.

## Replaying a different odometry source

`complementary_source.path` takes a TUM file of absolute odometry poses. The
replay re-synchronizes it against the same LiDAR frame stamps, so it is a fair
substitution rather than a re-timed one.

```yaml
replay_scale_tool:
  complementary_source:
    path: "/path/to/t265.tum"
    match_mode: nearest           # or: interpolate
    max_match_dt_s: 0.25          # reject matches farther than this, as the node does
    extrinsicTrans: [-0.310, 0.0, 0.159]     # omit both to reuse the run's
    extrinsicRot: [-1.0, 0.0, 0.0,           # complementary_odom_meta.yaml
                   0.0, -1.0, 0.0,
                   0.0, 0.0, 1.0]
```

### The extrinsic

Set it when the alternative sensor sits on a different mount than the one the
run recorded; omit it and the run's own `complementary_odom_meta.yaml` is used.

`extrinsicTrans` / `extrinsicRot` are spelled and read exactly as the node's
`complementaryOdom.extrinsicRot` parameters in `config/anymal.yaml` — a
3-vector and a **row-major** 3×3, nine numbers (nested rows also accepted) —
so a mount can be pasted between the two configs unchanged. Either key may be
left out, defaulting to zero translation and identity rotation as the ROS
parameter declarations do. They may sit directly in `complementary_source` as
above, or inside an `extrinsic:` sub-mapping.

The quaternion form the node writes into `complementary_odom_meta.yaml` is
accepted too:

```yaml
    extrinsic:
      translation: [-0.310, 0.0, 0.159]
      rotation_quat_xyzw: [0.0, 0.0, 1.0, 0.0]
```

An `extrinsic:` block that holds neither pair is a **hard error**, as is a
rotation that is not nine numbers, not orthonormal to 1e-3, or a reflection.
A wrong mount never fails loudly on its own — it silently rotates every
complementary displacement, and comes back out as a scale — so a spelling the
loader does not understand is refused rather than dropped in favour of the run's
recorded mount. Saving from the GUI writes the matrix form, unconverted.

### Matching a source that is not much faster than the LiDAR

Each frame's complementary displacement is measured between two LiDAR stamps,
and the scale estimate is a *ratio* taken from it. `match_mode` decides how the
source pose at those two stamps is obtained.

| Mode | What it does | Interval the twist spans |
| --- | --- | --- |
| `nearest` (default) | The node's own rule: take the sample closest to each stamp; reject the frame if either is farther than `max_match_dt_s`. | The odometry pair's own interval. |
| `interpolate` | Evaluate the source pose *at* each LiDAR stamp, between the two samples bracketing it. | Exactly the LiDAR interval. |

`nearest` quantizes both ends of the window onto the source's sample grid, so
the window is off by up to one sample period at each end — which lands directly
in the displacement the ratio is built from. At 200 Hz against 10 Hz LiDAR that
is under 5%; at 25 Hz it is up to 20% per frame, and at 2.5 Hz the measured
window can be twice the real one. `interpolate` removes it: the poses are at the
stamps, so `dt_complementary` is the LiDAR interval itself.

Interpolation is along the SE(3) geodesic between the bracketing samples — the
constant twist connecting them, the same motion model the reconstruction
integrates, not a straight line for position with the rotation handled apart
from it. It therefore reproduces a turn as an arc rather than a chord.

It cannot invent motion the source did not observe: across a gap it assumes
constant twist, so a source too sparse to resolve the real motion yields a
smoothed one. That is what `max_match_dt_s` gates in this mode — it becomes the
widest gap that may be interpolated across, and must exceed the source's own
sample period or every frame is rejected. Stamps outside the stream are never
extrapolated. Because the gate means something different in each mode, the
status line spells out which is in force:

```
    match mode: interpolate (max interpolated gap: 0.25 s)
```

`interpolate` is a replay-time improvement, not node parity — outputs are tagged
with it so an interpolated replay sits beside a `nearest` one in
`trajectories/` rather than overwriting it. Frames whose match fails a gate lose their complementary
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
    ├── scale_replay_estimator_trace<src>.csv  per-frame gate / raw / smoothed / applied
    └── scale_replay_vectors<src>.csv          per-frame geometry behind each sample
```

`<tag>` is `estimated_scale`, `recorded_scale`, or `scale_<value>` per fixed
scale. `<src>` is a suffix identifying the configuration — `_src_<name>` for an
alternative odometry source and `_corr_<mode>` for a non-default correction
mode. It exists so replays of different configurations sit side by side instead
of overwriting each other, which is what makes them plottable against one
another.

The two `log/` traces are only written in `estimated` mode; they are the raw
material for diagnosing why the estimator settled where it did. Both carry the
cross-track half of the correction alongside the scale (`lateral_*`, all `nan`
unless `lines_meet_xy` is applying one), and the vector trace also carries
`meet_x` / `meet_y` — where the lines met on that frame, recorded whenever there
is a line history to fit through, whatever correction is being applied.

## GUI viewer

```bash
replay-scale-gui                      # opens the newest run under ~/.ros/lili_logs
replay-scale-gui ~/.ros/lili_logs/run_20260805_192954_kdtree_lm
replay-scale-gui --ros-params-yaml my_replay.yaml
```

A frame scrubber, not a second way to run a replay: it calls the same
`run_replay` the CLI does, with `write=False`, so it produces no files unless
you ask, and can do nothing the CLI cannot. It forces `scale_mode: estimated` —
the per-frame vectors it draws only exist on that path — and says so in the
status bar.

### Choosing and saving a run

The left panel lists every run under the base directory, newest first. Selecting
a row does nothing on its own; **Load** replays the selected run, and a
double-click does both at once. Selection is cheap and reversible, a load is a
second or so of work, so they are kept separate. **Open other…** takes a run
directory from anywhere on disk, and adds it to the list once loaded.

The run being shown is marked `▶` and bold, with its full path under the list —
every other panel in the window describes that run, and a list where the
selection has wandered off should not be able to imply otherwise.

**Save trajectories…** is the only thing that writes. It asks for a directory
and re-runs the replay through the pipeline's writing path with the
configuration the panel currently shows, so what lands is exactly what the CLI
would have written: `<chosen>/trajectories/*.tum` and `<chosen>/log/*.csv`, with
the same source tagging, and no extra `replay/` level since you already chose
where it goes. Editing a parameter and saving therefore needs no round trip
through a YAML file — though **Save YAML…** in the configuration dock still
writes one if you want the run reproducible headlessly.

One load feeds three tabs: **Anchor frame**, the per-frame vector view described
below; **Trajectory**, the whole-run plot; and **Scale**, the estimate over
time.

The **scrubber sits under the tabs, not inside one**. The frame index is a
property of the session rather than of the view looking at it, so every tab
marks the same frame and switching tabs keeps you on it: find a suspicious step
in the applied scale, switch to the anchor view, and you are already on the
frame that caused it. The coverage strip, the frame counter and the
**Observable only** filter are shared for the same reason; only options that
change how a tab *draws* (zoom, frame, history, `|comp| = 1`, log axis) live
inside their tab.

Axes on the anchor-frame tab follow the robotics convention: **x up the page,
y to the left**. (The trajectory tab keeps the plotting CLI's orientation, x
right and y up, so the two figures can be compared with what is already saved.)

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
| Orange dotted lines | The previous degenerate frames' lines (`history`, default 50, taking every `step`-th, default 4), fading with age. |
| Dashed green arrow, ending in `×` | The point closest to every line drawn — where they agree the robot is. Absent when they are too parallel to say. |
| Filled green ellipse | The 1σ uncertainty of that point, from how far the lines miss it. |
| Small green dots (**meet trail**) | The point the *estimator* fitted on each earlier frame in the overlay — the trail of what the correction has been reading. Off by default. |

**meet trail** is a different population from the `×`: one dot per earlier
frame, each fitted from *that* frame's own line window, where the `×` is one fit
through the lines currently on screen. It answers "is the point the correction
reads jittering or drifting", which one frame cannot. Each dot is in units of
its own frame's `|comp|`, so it is only drawn in the complementary-forward frame
with `|comp| = 1`, and silently skipped anywhere else. It follows
`scaleLineHistory: 0`, which turns the estimator's fit off altogether.

**Where the lines meet.** Each degenerate line says only "the truth lies
somewhere along here", but the direction rotates as the robot turns, so lines
from different frames cross. The point with the least total squared distance to
all of them — the current line plus every history line on screen — is what those
statements agree on, and it is a position estimate LiDAR alone could not give.
It is drawn as an arrow from the anchor, deliberately in the same green as the
complementary arrow and dashed rather than solid: same kind of quantity, a
displacement from the anchor, but inferred from the lines rather than measured.
Reading the two against each other is the point — where the dashed arrow lands
short of the solid one, the odometry claims more motion than the lines support.

The **ellipse** around it is the 1σ covariance of the fit, `σ² A⁻¹`: `A` says how
well the line directions pin each axis down, `σ` how far the lines miss the
point. Its shape is the useful part — long and thin means one direction is
pinned and the other is a guess, which a bare point would hide. It assumes the
lines are independent, which consecutive frames are not, so read it as the
spread of the lines rather than as a calibrated confidence region.
`scaleLineFitNorm` selects the fit: `l1` is drawn with a MAD-based σ, so one
wild line widens the ellipse as little as it moved the point.

Both are computed from what is actually drawn, so they follow the `|comp| = 1`
toggle and the history controls. Two refusals keep it honest: lines parallel to
within `MIN_LINE_SPREAD` (about 3.6° for a pair) have no meaningful crossing and
nothing is drawn, and a crossing further than three view half-widths away is
dropped rather than drawn as an arrow off the edge. Both are the normal case on
a straight stretch, where there is genuinely nothing to say.

Expect it far more often normalized than in metres. Normalized, each line is
drawn in its own frame's geometry, which de-rotates it and leaves a real spread
of directions; in metres they are re-referenced onto the current anchor and stay
nearly parallel through a straight tunnel. On the bundled 6519-frame run that is
5665 frames with a meeting point (5661 inside the view) normalized, against 806
in metres.

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

Consecutive frames barely differ, so **step** (default 4) strides through the
earlier degenerate frames rather than redrawing almost the same line fifty
times — with the defaults the overlay spans the last 200 degenerate frames.
Frames without a usable basis are skipped, so a history of 50 always shows 50
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

**|comp| = 1** is on by default: it divides the whole frame through by the length of the
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
(`complementaryOdom.scaleMinNonDegenerateSpeed`, measured on the LiDAR
displacement — see [The observability gate](#the-observability-gate)). Because
that gate is a *speed* gate, it is exactly the filter that removes
noise-dominated frames. On the
bundled example it keeps 1821 of 3819 frames and lifts the median window
displacement from 0.11 m to 0.89 m; frames under 0.15 m fall from 54% to 7%.
Uncheck it to reach every frame. `scaleMin`/`scaleMax` do not enter here: they
clamp a sample's value, not whether it is observable.

### Trajectory tab

The second tab is the whole-run, top-down XY plot — the same figure
`replay-scale-plot` writes, with the same curves and colours: the recorded
effective trajectory, every replay this configuration produced, and the raw
complementary odometry integrated on its own.

It is drawn from the replay held in memory, not from the `trajectories/` folder:
the GUI runs with `write=False`, so there may be no files to read, and reading
them would show whatever an earlier CLI run left behind rather than the settings
currently applied. **Apply & re-run** therefore redraws this tab too — which is
the quickest way to see what a parameter change did to the *shape* of the run,
where the anchor-frame view only shows one window of it.

A black dot marks the frame the slider is on. The matplotlib toolbar above the
plot pans and zooms; these runs are long and thin, so the interesting stretch is
usually a small part of the extent. Scrubbing only moves the marker, and only
while this tab is on screen — the curves themselves are redrawn once per load.

### Scale tab

The estimate over the run, as three curves per frame:

| Curve | What it is |
| --- | --- |
| `instant raw` (grey) | `scale_instant_raw` — what this frame's window alone argues for. Noisy by nature; it is a ratio of two short displacements. |
| `smoothed estimate` (green) | `scale_smooth` — the `scaleSmoothingMode` statistic over the last `scaleSmoothingWindowSize` **observable** samples. |
| `applied` (orange, dashed) | `scale_applied` — what the trajectory was actually built with. It is the smoothed value from *previous* frames, so it lags by one and steps rather than glides. |

Frames that failed the observability gate are shaded, which is the answer to
"why is the green curve flat here" — nothing was admitted to the history, so
the mean could not move. Configured `scaleMin` / `scaleMax` are drawn as dashed
red lines: raw samples outside them are the ones entering the filter clamped,
and the smoothed curve can never leave the band between them.

Under `lines_meet_xy` the sample is a point rather than a number, and the
cross-track coordinate gets **its own plot underneath, on the same timeline**
(purple: `lateral raw` and `lateral applied`, with `scaleLateralMax` as its
bounds). They are different quantities — one centred on 1, one on 0 — so
stacking them beats sharing a y range that suits neither, and it lets the scale
go logarithmic while the signed coordinate stays linear. The lower plot is
absent entirely for the corrections that produce no such coordinate.

The scale y-axis is **linear**, so a deviation reads as the number it is. The
**log scale** checkbox switches to a logarithmic one, which is the axis a ratio
deserves — 2 and 0.5 are the same error in opposite directions — and which keeps
a run whose estimate spans decades readable near 1.

Either way the range is fitted to the bulk of the estimate (its 1st–99th
percentile) rather than to its extremes: the raw ratio reaches 60× on real runs
and is allowed to clip, because otherwise it flattens everything worth reading
into a single line. The lateral range is symmetric about 0, because that
quantity is a direction and an axis that says otherwise reads as a trend. Zoom
with the toolbar to follow a spike out of frame.

A black vertical line marks the scrubbed frame here too, so the anchor view and
this one always describe the same sample.

### Seeing all three at once

**All views**, next to the frame counter, replaces the tabs with all three
panels at once: the anchor frame and the trajectory side by side, the scale
curve across the bottom, on draggable splitters. Reading the scale curve against
the trajectory it produced is the whole argument for having both, and on a wide
screen there is no reason to alternate. It is the same widgets either way — Qt
reparents them — so the frame you are on, the zoom you set and the figures
themselves survive the switch.

### Editing the configuration

The **Configuration** dock edits the same `complementaryOdom` parameters and
`replay_scale_tool` settings the YAML carries — `complementaryCorrection`,
`translationScale`, `scaleMinNonDegenerateSpeed`, `scaleBaselineFrameLag`,
`scaleSmoothingWindowSize`, `scaleSmoothingMode`, `scaleLineHistory`,
`scaleLineHistoryStep`, `scaleLineFitNorm`, `scaleLateralMax`, `scaleMin`,
`scaleMax`, `scaleEstimationApply`, `ignore_dz`, `correction_mode`, and the
complementary source path, `match_mode` and match gate. `scaleMax` and
`scaleLateralMax` show **unbounded** at 0, which is how an infinite bound
round-trips through a spin box.

It is **grouped the way the config file is**, and each group folds. Most of the
form describes a method that is not running: selecting a correction unfolds the
settings it reads and folds away the ones it does not, greying them out. Folding
never edits — a folded section keeps its values and still contributes them, so
switching methods and back loses nothing.

- **Apply & re-run** replays with the edited values (on the worker thread, still
  writing nothing) and redraws.
- **Revert** returns to the configuration the current view was produced with —
  so a failed run does not lose your baseline.
- **Load YAML…** reads a configuration file in and immediately replays the
  current run with it — loading a file but waiting for *Apply* would leave the
  window showing one configuration and holding another. The frame you were on is
  kept. A file that does not parse is reported and changes nothing.
- **Save YAML…** writes those values as a config file, and the status bar shows
  the `replay-scale-trajectory … --ros-params-yaml <file>` command that
  reproduces the same replay headlessly.

The file the values came from is named at the top of the dock, and both loading
and saving update it. The viewer starts from the bundled
[`config/default.yaml`](src/replay_scale/config/default.yaml) unless
`replay-scale-gui --ros-params-yaml <file>` names another — the same flag the
CLI takes.

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
| `plotting.py` | `draw_trajectories()`, `draw_local_frame()`, `draw_scale_history()`, and the `build_*_figure()` wrappers around them | Never calls `show()` or `savefig()`. The `draw_*` half takes an axes, so a GUI canvas and a saved PNG share one drawing routine. |
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
  the smoothing-history update rule and the causal ordering where the scale
  applied at frame *k* comes from history up to *k-1*. The window statistic is
  the node's mean in its default `scaleSmoothingMode: mean`; the observability
  gate is measured differently, see below.
- `core/odom_source.py` mirrors `inferComplementaryOdomTwist` in its default
  `match_mode: nearest` — nearest-sample matching rather than interpolation, the
  same 0.25 s gate, and the same exclusion rule when two LiDAR stamps land on
  one odometry sample. `match_mode: interpolate` is opt-in and deliberately not
  parity; see [Matching a source that is not much faster than the
  LiDAR](#matching-a-source-that-is-not-much-faster-than-the-lidar).

Three intentional divergences, all noted in the source:

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
- `scaleMinNonDegenerateSpeed` is measured on the LiDAR displacement alone. The
  node gates on `|t_lidar_nondeg_proj| / dt_complementary` — the LiDAR
  displacement *projected onto the complementary non-degenerate direction*, over
  the complementary window — so both the magnitude and the divisor carry the
  odometry being tested. See [The observability
  gate](#the-observability-gate).

Everything else that is not the node is opt-in and defaults to off:
`complementaryCorrection` other than `ratio`, `scaleSmoothingMode` other than
`mean`, `scaleMin` / `scaleMax` / `scaleLateralMax` (the node has no bounds),
and `match_mode: interpolate`. An unconfigured replay reproduces the node.

`lines_meet_xy` is the one that is not a scale at all: it applies a 2D
similarity to the complementary displacement, where the node's correction is a
single multiplier. There is nothing in C++ to be parity with, and porting it
back would mean the node applying a rotation it currently cannot express.

When the C++ changes, change it here too.
