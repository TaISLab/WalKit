# walker_stability

`user_stability` (`scripts/user_stability.py`) publishes `/user_stability` (`walker_msgs/StabilityStamped`): how far the
user's centre of mass is from the edge of the polygon they are supported by. It is computed from the loads the walker
measures, `/left_loads` and `/right_loads` (foot position and load, from `walker_loads`) and `/left_hand_loads` and
`/right_hand_loads` (handle load, whose position comes from tf), plus `/user_desc` (latched, only for the `uid` and
`tin` fields of the message).

```bash
ros2 launch walker_stability walker_stability.launch.py            # sim_time:=true when replaying a bag, debug:=true
python3 -m pytest test                                              # the geometry (support_polygon) and the model
```

It works on data time, like `walker_centroid_support`: the four inputs go through a `StampSequencer` (`walker_loads`), the
state is evaluated on a 50 ms grid of *data* time and published stamped with the tick. Same bag, same output, at any replay
rate (checked at rates 1 and 4), at the price of the sequencer's latency (up to `max_wait_s`, about a second in practice).

## The model (`walker_stability/stability_model.py`, `support_polygon.py`)

| | |
|---|---|
| **Centre of pressure** | barycentre of the two feet and the two handles weighted by their loads (what `walker_centroid_support` publishes), with each loaded foot pushing at the centre of its sole, `(foot_toe_m − foot_heel_m) / 2` = 5 cm ahead of the point where the legs are tracked. |
| **Centre of mass** (`pose` of the message) | the centre of pressure low-passed with time constant `com_tau_s` (0.5 s), plus `com_moment_kgm / weight` metres forward (6 kg·m; needs `/user_desc`, no correction without it). While walking the centre of pressure swings from foot to foot two to three times wider than the centre of mass does; averaged over about a step it is what the centre of mass does (inverted pendulum). See "Bias of the centre of mass" below for the correction. |
| **Support polygon** | convex hull of the soles of the feet that carry at least `min_foot_load_kg` (5 kg) and of both handles. The legs are tracked at the shank, so a foot is the patch of floor around the tracked point (`foot_heel_m` 0.08 behind, `foot_toe_m` 0.18 in front, `foot_half_width_m` 0.05 to each side, pointing along the walker's heading). The handles always count: the user walks with the hands on the handlebar, and a hand that rests there is a contact they can lean on, loaded or not. |
| **`margin`** | signed distance from the centre of mass to the nearest edge of the polygon, in metres: positive inside, negative outside. |
| **`sta`** | `margin / margin_ref_m` clipped to 0..1 (`margin_ref_m` = 0.10 m is the margin that counts as fully stable: a scale, not a validated threshold). |
| **`contacts`** | feet carrying load plus the two handles (2 to 4). |

The support polygon is also published, as a `geometry_msgs/PolygonStamped` on `/support_polygon` (`polygon_topic_name`), stamped like
`/user_stability`, for rviz (`walker_rviz` draws it) and for anything else that wants to see what the margin is measured in.

The frame of `pose` is that of the step positions (`laser`). Everything depends on the stamps only, not on the clock: the
filter advances with the stamps of the ticks it is evaluated at.

### Bias of the centre of mass

Against the De Leva centre of mass of the mocap (`mocap_stability`; 47 definitive bags, between the two `/tag` messages) the
filtered centre of pressure is on average **12 cm behind it** along the walker's x axis (CA −17.8, LA −11.6, MF −9.1,
MT −13.8; ±1–3 cm between the bags of one user), and level in y. The walker's foot positions themselves have no bias in x
against the mocap's ankles (−0.8 cm in CA_test01), so it is not the frame calibration.

What was found, and what was not:
- The low-pass takes the position error without bias (RMS) from 8.9 cm for the raw centre of pressure to 5.5 cm.
- The bias times the user's weight is constant: `bias ≈ −8.75 kg·m / weight` with no intercept (residual 2.2 cm, 2.5 cm leaving
  one user out; a constant bias leaves 3.8 and 5.3 cm). A pure constant would be the signature of an error in length (a
  marker or a foot offset); this is the signature of a **moment that does not depend on the user**.
- 4 cm of it is physical: a loaded foot pushes at its sole centre, not at the shank. The rest, 6 kg·m, is the
  `com_moment_kgm` correction. With both the mean bias is +0.2 cm, per-user residuals within ±2.2 cm, and the position
  error without bias is unchanged (5.5 cm).
- What it is, is open. A vertical load missing in the handles would need ~15–20 kg more than they read (a third of the body
  weight of the lightest user: implausible); a horizontal push of the hands at that height, ~60–90 N (also high). A
  placement of the mocap markers that shifts the De Leva centre of mass would scale with the body, not as 1/weight, but with
  four users the two cannot be told apart. **Check the handle load calibration (`walker_loads/config/handle_calib.yaml`)
  before trusting the correction**; `com_moment_kgm:=0` switches it off.

### Why not the centre of pressure itself, or the old window

The first thing tried was the margin of the *centre of pressure* in the polygon of the loaded contacts. It is useless: the
centre of pressure is a convex combination of the contacts, so it lies on the boundary of their hull whenever the handles
carry little (the median handle load in CA_test01 is 0.1 kg): on one foot it *is* that foot, on two it lies on the segment
between them. In CA_test01 the margin came out at 0 for most of the walk and the polygon had no area at all in two thirds of it.

The model before this one (an index made of two ramps, the lateral one between the handles and the longitudinal one
between the limits of the horizontal reach of the arms, from an anthropometric regression) was almost always 0 as well,
once its arithmetic was right: the reach window was 4–5 cm wide while the centroid travels ~30 cm. It, its regression
workbook (`config/Correlationes.xlsx`) and its tests were removed in the commit that introduced this model; they are in the
history (`git show c53eff7:walker_stability/walker_stability/bos.py`).

## Interface

`walker_msgs/StabilityStamped` gained two fields: `margin` (m) and `contacts`. `sta` is the same 0..1 index as before, so
`stability_grid_map` (which only reads `sta` and `pose`) is unaffected; `pose.position` is now the estimate of the centre of
mass, no longer the raw centroid. Without `/user_desc` the node publishes anyway (`uid` empty, `tin` NaN).

Parameters: `period` (0.05 s, grid of data time), `max_wait_s` (5), `min_foot_load_kg` (5), `com_tau_s` (0.5), `com_moment_kgm` (6),
`margin_ref_m` (0.10), `foot_heel_m`, `foot_toe_m`, `foot_half_width_m`, the topic names. The foot sole and `margin_ref_m` must be the same
in `mocap_stability` for its numbers to be comparable.

## Validation

`mocap_stability` (mocap4w, private repo `MFernandezCarmona/mocap_stability`) computes the same quantity from the motion capture (centre of mass of the De Leva model, feet
in stance, the same polygon) and compares it with this node's, offline and online. The agreement is partial: see its README.
