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
| `ms_period` | 50 | Period (ms) of the grid, **on data time**, the leg loads are sampled and published on (see [Time](#time-the-stamps-of-the-data-not-the-clock)). The step loads counted downstream (`gait_monitor_speed`: a leg's load going from 0 to positive) depend on it, as the sampling is what filters the chatter of the leg-load ramp. |
| `max_wait_s` | 5.0 | How long (data time) the inputs are held back waiting for a silent one before it is given up on. Must be longer than the largest lag between inputs, or the outputs depend on timing again (see [Time](#time-the-stamps-of-the-data-not-the-clock)). |
| `debug_output` | false | Sets the logger to DEBUG and enables `DiffTracker`'s CSV dump (`diff_tracker_measurements.csv`, written to the node's working directory) — see [Debugging the Kalman tracker](#debugging-the-kalman-tracker). |
| `data_timeout_s` | 1.0 | A stream (handle force, step) not received for longer than this is treated as **no data**, not as "the last value it ever sent" — the leg-load computation pauses rather than freezing on stale input. "Not received for longer" is measured in data time (against the stamp of the sample being processed). Does not apply to `/user_desc`: see above. |
| `speed_delta` | 0.05 | Floor (m/s) for the double-support ramp half-width (see below); also the sole threshold when `speed_delta_ratio` scaling can't apply yet. |
| `speed_delta_ratio` | 0.3 | Fraction of the estimated swing-speed amplitude (`DiffTracker::get_speed_amplitude()`) used as the double-support ramp half-width, instead of one fixed value for every user. |
| `speed_delta_max` | 1.0 | Ceiling (m/s) on that same ramp half-width — protects against the amplitude estimate spiking on bad upstream data (see [Known limitations](#known-limitations)). |
| `kalman_v0`, `kalman_va`, `kalman_vb` | 0.01, 1.0, 0.0 (m/s) | Initial speed-diff state: DC offset, in-phase and quadrature components. |
| `kalman_f0`, `kalman_fa`, `kalman_fb` | 4.0, 15.0, 0.0 (kg) | Initial force-diff state, same layout. |
| `kalman_w` | 2.0 (rad/s) | Initial shared gait angular frequency (~3 s/cycle). |
| `kalman_theta` | 0.20 (rad) | Initial absolute phase. |
| `kalman_theta_sigma` | π (rad) | Uncertainty (1 sigma) of that initial phase: the gait phase is unknown when the filter starts, so it is wide and the first measurements set it. With 1 rad the filter trusted 0.20 rad and could lock on a wrong phase (MF_test07: right-leg fraction mean 0.73 instead of ~0.5, dominant-leg agreement with the mocap 59 % instead of 85 %); before the filter worked on data time this uncertainty was effectively unbounded by accident (the first prediction stepped by the absolute clock, ~1.7e9 s). |
| `w_estimate_min`, `w_estimate_max` | 0.3, 8.0 (rad/s) | Plausible range for a gait-cycle frequency measured from step-alternation timing (~20 s down to ~0.8 s per cycle); estimates outside this range are dropped. |
| `zero_crossing_refractory_s` | 0.2 | Minimum time between accepted zero-crossings of the L/R speed difference, so sensor noise wobbling around zero isn't mistaken for a stance/swing swap. |

All the `kalman_*` values and the noise covariances inside `DiffTracker` are
physically-reasoned starting points, not values fitted against real data —
see [Known limitations](#known-limitations).

## Time: the stamps of the data, not the clock

`partial_loads` has no timer and never reads the clock (so `use_sim_time` is irrelevant to it): everything it
does is driven by the `header.stamp` of its inputs (the step position's stamp for `/detected_step_*`). A
message is processed once no input can still deliver anything older (the smallest of the four inputs' last
stamps has passed it, see `walker_stamp_utils/stamp_sequencer.h`), in stamp order; the loads are sampled and
published on a fixed grid of data time (`ms_period`), with the state as of each tick, if anything new arrived
since the previous one. Consequences:

- **The same inputs give the same outputs, bit for bit**, whatever the DDS delivery order, the machine load or
  the replay rate (`ros2 bag play --rate`): checked at rates 1, 2, 4 and 8. Before, the order in which the
  topics happened to arrive and the wall-clock tick decided the Kalman updates, the loads and so the gait
  parameters downstream.
- **Latency**: a message waits for the slowest input, i.e. up to about one handle period (the handles publish
  at ~2 Hz, ~0.5 s) plus the grid. `/left_hand_loads` and `/right_hand_loads` depend on one message each and go
  out immediately. If an input stops altogether the others wait at most `max_wait_s`; what the silent input
  delivers afterwards, if older than what was already processed, is dropped (warning).
- The Kalman filter steps by the difference of the stamps (the first measurement starts the clock), so the
  result does not depend on how large the stamps are (a recording's date).
- Inputs are subscribed with a 500-deep queue and outputs published with a 1000-deep one: releasing in stamp
  order makes the outputs bursty, and a reliable writer with a short history overwrites what a slower
  subscriber has not yet taken.

The python nodes downstream share the same helpers (`walker_loads/stamp_sequencer.py`): `StampSequencer` and `Ticks` for the
stamp-ordered, reproducible behaviour, and `Watermark` for a node that publishes on every message it receives instead
(`walker_centroid_support`, `walker_stability` by default): it only tracks the data time every stream has reached, the smallest of
the streams' newest stamps, and gives up on a silent stream after `max_wait_ns` as the sequencer does.

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

## Node: `gait_monitor_speed.py`

Gait parameters (SpT, SdT, NoS, SpL, SdL, Tr, d, CAD, WV; definitions in the script's docstring) from the step loads
(`/left_loads`, `/right_loads`: a *step load* is a leg's `load` going from 0 to positive) and the odometry
(`/odom`, only for the distance). Publishes `/left_gait_stats`, `/right_gait_stats` (`GaitStamped`) and
`/global_gait_stats` (`WalkStats`).

**Refractory period per leg** (`min_step_interval_s`, 0.8 s of data time; 0 counts every transition). Near 0 the load of a leg
flickers (0, 0.4, 0, 0.6 kg in 100 ms) and every flicker was a step; the false ones are short "steps" that inflate any
variability or symmetry index. After a leg registers a step its next step loads are ignored for that long (`step_onsets.py`,
tested). It runs from the last *registered* step, so a long flicker cannot extend it, and it is per leg: the other leg's step may
follow closely (double support). The interval between two steps of the SAME leg is a stride, and no adult's is under ~0.9 s (the users
of the definitive bags walk with strides of 1.2 to 2 s), so 0.8 s cannot cut a real step.

It was 0.35 s at first (WalKit#23), judged against the old mocap stance detector, which had flicker of its own (it counted 75 steps in
CA_test01 where the corrected detector of `mocap_loads/gait_quality.py` finds 51 with gaps of 1.4 to 2.2 s between a leg's steps). Against the
corrected mocap onsets (12 definitive bags: CA 3, LA 2, MF 4, MT 3, between the two `/tag` messages), with the same interval on both sides:

| `min_step_interval_s` | steps walker / mocap (mean per bag) | mean abs difference |
|---|---|---|
| 0 | 117 / 68 | 52 |
| 0.35 | 100 / 61 | 39 |
| 0.5 | 90 / 61 | 30 |
| 0.7 | 77 / 60 | 22 |
| 0.9 | 71 / 60 | 20 |
| 1.1 | 62 / 53 | 12 |

The count keeps coming down towards the mocap's until the interval passes a stride, so the false onsets are not only flicker within a third of a
second. 0.8 s is the floor that cannot cut real steps; what is left (the walker still counts ~20 % more than the mocap) comes from the split of
the load between the legs (`partial_loads`: the swinging leg's load ramps up at times), not from the filter. The mean step time that the
walker gets is 0.6 s where the mocap's is 0.83 s.

Like `partial_loads` it works on data time: the three inputs are put in stamp order (`walker_loads.stamp_sequencer`;
the loads by the stamp of the step position, which repeats while the load is recomputed) and the cumulative
summary is published each time a new step load is registered, **stamped with that step's stamp** (before: with the
clock at a timer tick, every `period`). The same inputs give the same summaries whatever the delivery order or the
replay rate. Parameter `max_wait_s` (5 s): how long to wait for a silent input before giving up on it. The odometry only
feeds a running sum, so if its stamps lag so far behind the loads' that it is overtaken (a mocap whose clock drifts, as in
MF_test07) it is still counted, not dropped; only the `d` of the intermediate summaries then depends on the arrival
order (the final one does not).

## Node: `gait_quality.py`

Gait quality from the leg loads, published as `/gait_quality` (`walker_msgs/GaitQuality`) at every stance onset, over the last
`window_s` (20) seconds of data time (0 = since the first onset):

| Field | Definition (`walker_loads/gait_quality.py`, shared with the mocap counterpart in `mocap_loads`) |
|---|---|
| `step_width` | mean lateral gap (m) between the feet at the onsets: `\|y\|` of an onset against the opposite leg's nearest onset (within 1 s), in the walker's frame |
| `double_support` | fraction of the window with both legs in stance |
| `step_time`, `step_time_si`, `step_time_cv` | step time (from the opposite leg's onset to this one's, 0.2 to 2 s), its symmetry index (Robinson 1987, needs 2 steps per leg) and its coefficient of variation (needs 4 steps); `NaN` until then |

A leg is **in stance** when it carries at least `stance_load_fraction` (0.15) of the user's weight (`/user_desc`). It is not "load above 0" as
in `gait_monitor_speed`: the swinging leg's load ramps with the relative speed of the feet and hardly ever reaches 0, so both legs
would be in stance almost always. The onsets have the refractory period `min_step_interval_s` of `gait_monitor_speed`, and a load that
flickers back within it is the same stance. Data time like the other nodes (`StampSequencer`); the same bag gives the same numbers at
any replay rate. `ros2 launch walker_loads gait_quality.launch.py` (`sim_time`, `debug`, `window_s`, `min_step_interval_s`,
`stance_load_fraction`). The comparison against the mocap is in `mocap4w/mocap_loads` (`compare_gait_quality.launch.py`).

Against the mocap (47 definitive bags, between the two `/tag` messages, bag means compared across bags): the **step width** orders the bags as
the mocap does (correlation 0.87, slope 0.62; 16.2 against 24.9 cm); the other four agree little (double support 56 % against 27 %, correlation
0.39; step time 0.75 against 0.82 s, 0.50; symmetry index 0.00; coefficient of variation 0.32) and the walker counts ~17 % more steps (28 against
24 per window). Only the step width is a usable measure for now; the table and the reading are in the README of `mocap_loads`.

## Reproducing / testing this offline

The comparison of walker_loads against the mocap (and the `replay_offline.launch.py` pipeline that replays a definitive
bag through `partial_loads` and `gait_monitor_speed`) lives in `mocap4w/mocap_loads`, so this package knows nothing
about mocap:

```bash
ros2 launch mocap_loads replay_offline.launch.py bag_name:=MF_test05 rate:=3.0
ros2 launch mocap_loads compare_loads.launch.py bag_name:=MF_test05 rate:=4.0 out_dir:=OUT
```

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
