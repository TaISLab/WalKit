# walker_stability

`user_stability` (`scripts/user_stability.py`) publishes `/user_stability` (`walker_msgs/StabilityStamped`): a 0–1 index
`sta` of how well placed the user is, from `/support_centroid` (walker_centroid_support), the user's description
(`/user_desc`, latched) and the handle frames (`left_handle_id`, `right_handle_id`, relative to `base_footprint`). It
carries the centroid's own stamp, so it follows the data (same bag, same output, at any replay rate); the centroid is
looked up in `base_footprint` with tf.

```bash
ros2 launch walker_stability walker_stability.launch.py            # sim_time:=true when replaying a bag, debug:=true
python3 -m pytest test                                              # bos.py against the original loops
```

## The index

`sta = x_sta * y_sta`, each factor 1 at the centre of its range and 0 at its limits or beyond (`bos.centered_score`):

- **y**: between the two handles.
- **x**: the distance from the centroid to the handles, within the *base of support in x*: the horizontal reach of the
  arm holding the handlebar at its height with the elbow slightly flexed (extension 20–30°). Shoulder, elbow and fist
  heights come from regressions on the user's height and age (`config/Correlationes.xlsx`, brackets from 50 years: a
  younger user gets the nearest one), and the arm postures that put the fist at the handlebar height (± 1 cm) are
  searched on a 0.1° grid (`walker_stability/bos.py`, cached per user and handle height).

## What was wrong (fixed with the move to data time)

The node did not run at all: the script had no shebang, `tf2_geometry_msgs` was not imported (every transform failed),
the launch file started `partial_loads` instead, and the workbook was read with pandas on every message together with a
pure-python search of 1.2 million iterations. And the model was wrong in four ways:

1. The trigonometry took degrees as radians (the base of support came out ±0.34 m, nonsense).
2. `UserDesc.height` (cm) was used as metres.
3. The two factors were linear ramps from one limit to the other, not "maximum in the centre" as documented.
4. The age lookup silently used the first bracket (50–54) for any younger user (all the test users are 20–44) and ran off
   the table for an age falling in a gap between brackets; it picks the nearest bracket explicitly now.

The executable is `user_stability.py` now: a script called `walker_stability.py` shadows the package of the same name
when it is run.

## Known limitation: the index is almost always 0

With the trigonometry right, the base of support in x is **4–5 cm wide** (e.g. 0.39–0.43 m for the test user CA with
the handles at hole 4), while the support centroid moves over about 30 cm during a walk. Replaying the definitive bags
(rate 4): `sta > 0` 10 % of the time in CA_test01 (mean 0.04), 29 % in MF_test07 (mean 0.07), never in LA_test03. The
centroid is also not the user's shoulder: it is the barycentre of the two feet and the two handles, so it is pulled
towards the handles when they carry load. Whether to recalibrate the window or to replace the model (for example with the
margin of the centre of mass inside the support polygon of the four contacts, which is what the mocap side can compute
too) is open.
