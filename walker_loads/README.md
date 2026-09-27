# walker_loads

Splits the walker user's body weight between the two legs and the two handles
of the rollator, combining:

- handle force readings (calibrated to kg via a spline, see `config/handle_calib.yaml`),
- foot position/speed from `walker_step_detector`,
- the user's declared weight (`/user_desc`),

into a per-leg load estimate (`/left_loads`, `/right_loads`) and a
calibrated per-handle load (`/left_hand_loads`, `/right_hand_loads`).

## Node: `partial_loads` (C++)

### Topics

| Direction | Topic (param)                                          | Type                          | Notes |
|-----------|---------------------------------------------------------|--------------------------------|-------|
| sub       | `left_handle_topic_name` (`/left_handle`)                | `walker_msgs/ForceStamped`     | raw gauge reading |
| sub       | `right_handle_topic_name` (`/right_handle`)              | `walker_msgs/ForceStamped`     | raw gauge reading |
| sub       | `left_steps_topic_name` (`/detected_step_left`)          | `walker_msgs/StepStamped`      | from `walker_step_detector` |
| sub       | `right_steps_topic_name` (`/detected_step_right`)        | `walker_msgs/StepStamped`      | from `walker_step_detector` |
| sub       | `user_desc_topic_name` (`/user_desc`)                    | `walker_msgs/UserDesc`         | published **once, latched** (`transient_local`) — see below |
| pub       | `left_loads_topic_name` (`/left_loads`)                  | `walker_msgs/StepStamped`      | input step message with `.load` filled in |
| pub       | `right_loads_topic_name` (`/right_loads`)                | `walker_msgs/StepStamped`      | input step message with `.load` filled in |
| pub       | `left_hand_loads_topic_name` (`/left_hand_loads`)        | `walker_msgs/ForceStamped`     | gauge reading converted to kg |
| pub       | `right_hand_loads_topic_name` (`/right_hand_loads`)      | `walker_msgs/ForceStamped`     | gauge reading converted to kg |

`/user_desc` is a one-shot config value (like a parameter set once from the
web GUI), not a continuous stream, so the node subscribes to it with a
`transient_local`/`reliable` QoS: this is what lets it still pick up the
profile if it (re)starts *after* the GUI already published it, as long as
the GUI's own publisher (or whatever republished it) is still alive to
retain the sample.

### Parameters

**Topic names**: `scan_topic`, `left_loads_topic_name`, `right_loads_topic_name`,
`left_hand_loads_topic_name`, `right_hand_loads_topic_name`,
`left_handle_topic_name`, `right_handle_topic_name`, `left_steps_topic_name`,
`right_steps_topic_name`, `user_desc_topic_name`, `handle_calibration_file`
(defaults: see table above; `handle_calibration_file` defaults to
`config/handle_calib.yaml` inside this package's share directory).

| Parameter | Default | Meaning |
|---|---|---|
| `ms_period` | 500 | Period (ms) of the timer that computes and publishes the leg-load split. |
| `debug_output` | false | Sets the logger to DEBUG and enables `DiffTracker`'s CSV dump (`diff_tracker_measurements.csv`, written to the node's working directory) — see [Debugging the Kalman tracker](#debugging-the-kalman-tracker). |
| `data_timeout_s` | 1.0 | A stream (handle force, step) not received for longer than this is treated as **no data**, not as "the last value it ever sent" — the leg-load computation pauses rather than freezing on stale input. Must be kept above `2 * ms_period/1000`, or inputs may be flagged stale between successive timer ticks (a startup warning fires if not). Does not apply to `/user_desc`: see above. |
| `speed_delta` | 0.05 | Floor (m/s) for the double-support ramp half-width (see below); also the sole threshold when `speed_delta_ratio` scaling can't apply yet. |
| `speed_delta_ratio` | 0.3 | Fraction of the estimated swing-speed amplitude (`DiffTracker::get_speed_amplitude()`) used as the double-support ramp half-width, instead of one fixed value for every user. |
| `speed_delta_max` | 1.0 | Ceiling (m/s) on that same ramp half-width — protects against the amplitude estimate spiking on bad upstream data (see [Known limitations](#known-limitations)). |
| `kalman_v0`, `kalman_va`, `kalman_vb` | 0.01, 1.0, 0.0 (m/s) | Initial speed-diff state: DC offset, in-phase and quadrature components. |
| `kalman_f0`, `kalman_fa`, `kalman_fb` | 4.0, 15.0, 0.0 (kg) | Initial force-diff state, same layout. |
| `kalman_w` | 2.0 (rad/s) | Initial shared gait angular frequency (~3 s/cycle). |
| `kalman_theta` | 0.20 (rad) | Initial absolute phase. |
| `w_estimate_min`, `w_estimate_max` | 0.3, 8.0 (rad/s) | Plausible range for a gait-cycle frequency measured from step-alternation timing (~20 s down to ~0.8 s per cycle); estimates outside this range are dropped. |
| `zero_crossing_refractory_s` | 0.2 | Minimum time between accepted zero-crossings of the L/R speed difference, so sensor noise wobbling around zero isn't mistaken for a stance/swing swap. |

All the `kalman_*` values and the noise covariances inside `DiffTracker` are
physically-reasoned starting points, not values fitted against real data —
see [Known limitations](#known-limitations).

## How the leg-load split works

1. `leg_load = weight - left_hand_force - right_hand_force`, clamped to
   `[0, weight]` (a noisy or out-of-calibration-range handle reading must
   never produce a negative or over-100% load; `SplineFunction::interp()`
   itself also clamps to the calibrated force range instead of
   extrapolating past it).
2. The user is assumed to be either fully on one leg (single support) or
   sharing the load between both (double support). Instead of a hard
   threshold on the left/right foot speed difference — which jumps straight
   from 50/50 to 100/0 the instant it's crossed, and chatters when the
   speed difference is noisy near it — the split ramps smoothly across a
   band `[-band, +band]` around zero:
   `right_fraction = 0.5 * (1 + clamp(speed_diff / band, -1, 1))`.
3. `band` scales with how big this user's actual swing/stance speed gap is
   (`speed_delta_ratio * amplitude`, floored at `speed_delta` and capped at
   `speed_delta_max`) instead of one fixed number for every user — a slow
   walker's speed difference would otherwise never reach a one-size-fits-all
   threshold, and a fast walker's would blow past it immediately.
4. `speed_diff` and `amplitude` both come from `DiffTracker`, a Kalman
   filter over the left/right speed and force differences — see below.

## The Kalman tracker (`DiffTracker`)

`DiffTracker` models the left/right speed-diff and force-diff signals as two
sinusoids sharing one gait frequency `w` and absolute phase `theta`:

```
speed_diff(t) = v0 + va*sin(theta) + vb*cos(theta)
force_diff(t) = f0 + fa*sin(theta) + fb*cos(theta)
theta_k+1 = theta_k + w*dt
```

State: `[v0, va, vb, f0, fa, fb, w, theta]` (`kalman/DiffSystemModel.hpp`).

### Why Cartesian (va,vb), not amplitude+phase

An earlier version tracked amplitude (`v1`/`f1`) and phase directly, i.e.
`speed_diff = v0 + v1*sin(phase)`. That is a *polar* parameterization of the
same sinusoid, and polar coordinates have a singularity at the origin: the
measurement's sensitivity to `v1` is exactly `sin(phase)`, which vanishes
whenever the signal is near zero — at that point an arbitrarily large `v1`
paired with a near-zero `sin(phase)` explains the data exactly as well as
the correct, small `v1`. On real (if imperfect) offline replay data this
made the estimated amplitude settle on stable but physically nonsensical
values (~100, two orders of magnitude above a plausible gait speed). The
Cartesian form removes that: the measurement Jacobian for `(va,vb)` is
`(sin(theta), cos(theta))`, and `sin²+cos²=1` always, so it's never
simultaneously blind to both. Amplitude/phase, when needed, are derived
(`amplitude = sqrt(va²+vb²)`) rather than tracked directly. This also drops
the old model's separate force/speed delay parameter `d`: it's now implicit
in how `(fa,fb)` relate to `(va,vb)` at the same `theta`.

### Why `w` is measured, not just estimated

Neither the speed nor the force measurement has any Jacobian sensitivity to
`w` (see `updateJacobians()` in `SpeedMeasurementModel.hpp` /
`ForceMeasurementModel.hpp`) — it can only move through weak, indirect
coupling via `theta`. In practice this let `w` drift toward zero, which
stalls `theta`'s rotation and reintroduces the same amplitude degeneracy the
Cartesian form was meant to fix (if `theta` barely moves, every measurement
constrains the same one direction in the `(va,vb)` plane, and the
orthogonal one is free to grow).

`PartialLoads::update_gait_frequency()` fixes this by measuring `w`
directly: a zero-crossing of the L/R speed difference marks the moment the
swing/stance roles swap, which happens twice per gait cycle, so the time
between two consecutive crossings is half a gait period
(`w = pi / half_period`, validated against `[w_estimate_min, w_estimate_max]`
before being fed in). This is `DiffTracker::add_frequency_measurement()`,
backed by a dedicated linear measurement model
(`kalman/FrequencyMeasurementModel.hpp`, `h(x) = w`) with a real, unambiguous
Jacobian — unlike speed/force, this one really does observe `w` directly.

### Why speed/force updates can't touch `w` (`partialUpdate()`)

Even with a direct frequency channel, it fires only once per half gait-cycle
while the speed/force updates fire far more often and, despite having zero
*direct* sensitivity to `w`, still perturb it through the covariance's
`w`-`theta` cross term built up in every `predict()` step. Two tuning
attempts confirmed this empirically before landing on the real fix:

- **A floor/ceiling on `w`** (`clampFrequency()`) stopped it collapsing to
  zero, but the indirect drag between corrections still made the amplitude
  spike into the tens/hundreds.
- **Tightening the process noise `Q(W,W)`** to resist that drag backfired:
  it also weakened the Kalman gain of the rare, *good* direct corrections
  (both route through the same `P(W,W)`), so `w` spent *more* time pinned
  at the floor, not less.

The actual fix: `add_speed_measurement()`/`add_force_measurement()` now go
through `DiffTracker::partialUpdate()`, which performs the same update math
as `Kalman::ExtendedKalmanFilter::update()` but forces the `w` component of
the Kalman gain to zero and explicitly restores `w`'s row *and* column of
the covariance matrix afterward (zeroing the gain alone only protects the
row; the column can still leak through the other states' own gains).  Only
`add_frequency_measurement()` still goes through the library's normal,
unmasked update, and is the only thing allowed to move `w`. This required
one small addition to the vendored `kalman/` headers:
`LinearizedMeasurementModel::computeJacobian()`, which exposes the model's
`H` (otherwise `protected`, only usable by `ExtendedKalmanFilter` itself)
so `partialUpdate()` doesn't have to duplicate each model's Jacobian
formula by hand.

`clampFrequency()` is kept as a cheap backstop (e.g. before the first
frequency measurement ever arrives) even though it should now rarely, if
ever, trigger.

## Reproducing / testing this offline

`launch/replay_offline.launch.py` replays a labeled rosbag (from
`soma13kp/datasets/labeled_bags`, e.g. `MF_test05`) that is missing
`/user_desc`, `/handle_height` and part of the walker's tf tree, and
reconstructs them from `bagsFolder_unified/test_config.txt`:

```bash
ros2 launch walker_loads replay_offline.launch.py \
    bag_path:=/path/to/soma13kp/datasets/labeled_bags/MF_test05 \
    rate:=3.0
```

It brings up the walker's static tf tree (`walker_description`), the handle
tf (from the bag's configured `handle_height`), `walker_step_detector`
(`km_detect_steps`) and `partial_loads` itself, then plays back only the
topics needed (`/left_handle`, `/right_handle`, `/scan_filtered`, `/tf`,
`/tf_static`). See the file's own docstring for the exact field mapping from
`test_config.txt`.

### Debugging the Kalman tracker

Set `debug_output:=true` on `partial_loads` (e.g. edit
`launch/partial_loads.launch.py`) to write
`diff_tracker_measurements.csv` (in the node's working directory) on every
`predict`+`update`: raw measurement (`sp_m`/`fo_m`/`fw_m`, whichever of the
three fired that row), the tracker's own prediction (`sp_pred`/`fo_pred`),
and the full state (`amp`, `va`, `vb`, `w`, `theta`). Useful for checking
`w`/`amp` actually hold steady between corrections instead of drifting.

## Known limitations

- The Kalman noise covariances (`configureNoise()` in `diff_tracker.cpp`)
  and the `kalman_*` priors are physically-reasoned starting points, not
  values fitted against real data — retune once `walker_step_detector`
  gives trustworthy step speeds to validate against.
- The estimated amplitude still reflects some of `walker_step_detector`'s
  own known noise, not purely the user's real gait — `speed_delta_max`
  bounds how much that can affect the leg-load ramp, but doesn't fix the
  upstream signal.
- `theta` isn't re-wrapped to `[0, 2*pi)` after a measurement update (only
  `SystemModel::f()`'s own prediction step wraps it) — harmless since
  `sin`/`cos` are periodic regardless, and it self-corrects at the next
  `predict()`, but worth knowing if you inspect the debug CSV directly.
