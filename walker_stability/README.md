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
| **Centre of pressure** | barycentre of the two feet and the two handles weighted by their loads (what `walker_centroid_support` publishes). |
| **Centre of mass** (`pose` of the message) | the centre of pressure low-passed with time constant `com_tau_s` (0.5 s). While walking the centre of pressure swings from foot to foot two to three times wider than the centre of mass does; averaged over about a step it is what the centre of mass does (inverted pendulum). Against the mocap's De Leva centre of mass it cuts the error (RMS without bias) of the raw centre of pressure by 30–40 % and brings its spread to the mocap's (see `mocap_stability`); a constant offset of 8–15 cm behind the mocap's remains. |
| **Support polygon** | convex hull of the soles of the feet that carry at least `min_foot_load_kg` (5 kg) and of both handles. The legs are tracked at the shank, so a foot is the patch of floor around the tracked point (`foot_heel_m` 0.08 behind, `foot_toe_m` 0.18 in front, `foot_half_width_m` 0.05 to each side, pointing along the walker's heading). The handles always count: the user walks with the hands on the handlebar, and a hand that rests there is a contact they can lean on, loaded or not. |
| **`margin`** | signed distance from the centre of mass to the nearest edge of the polygon, in metres: positive inside, negative outside. |
| **`sta`** | `margin / margin_ref_m` clipped to 0..1 (`margin_ref_m` = 0.10 m is the margin that counts as fully stable: a scale, not a validated threshold). |
| **`contacts`** | feet carrying load plus the two handles (2 to 4). |

The frame of `pose` is that of the step positions (`laser`). Everything depends on the stamps only, not on the clock: the
filter advances with the stamps of the ticks it is evaluated at.

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

Parameters: `period` (0.05 s, grid of data time), `max_wait_s` (5), `min_foot_load_kg` (5), `com_tau_s` (0.5), `margin_ref_m`
(0.10), `foot_heel_m`, `foot_toe_m`, `foot_half_width_m`, the topic names. The foot sole and `margin_ref_m` must be the same
in `mocap_stability` for its numbers to be comparable.

## Validation

`mocap_stability` (mocap4w, private repo `MFernandezCarmona/mocap_stability`) computes the same quantity from the motion capture (centre of mass of the De Leva model, feet
in stance, the same polygon) and compares it with this node's, offline and online. The agreement is partial: see its README.
