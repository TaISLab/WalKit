# walker_laser_filter

The walker's scan chain, built only from the official [`laser_filters`](https://github.com/ros-perception/laser_filters)
(`scan_to_scan_filter_chain` with `LaserScanBoxFilter`), and the tool that finds where the lidar sees the walker itself.
This package holds the parameter files (`config/`), the launchers other packages include (`launch/`) and the script that
makes `config/self_filter.yaml` from recorded scans. There is no filter code of its own.

```
/scan ──[self_filter]──► /scan_filtered ──[nav_filter]──► /scan_nav    laser odometry, navigation
 raw    the walker's        │                 the active area removed
        own structure       └──[feet_filter]──► /scan_feet              step detectors
        removed                                 only the active area
```

| Launcher | Parameter file | In → out | What it does |
|---|---|---|---|
| `self_filter.launch.py` | `config/self_filter.yaml` (generated) | `/scan` → `/scan_filtered` | one box per patch of the walker's structure the lidar sees (posts, frame); they are removed |
| `nav_filter.launch.py` | `config/nav_filter.yaml` | `/scan_filtered` → `/scan_nav` | the **active area** (where the user's feet are: x −1.2…0.03, y ±0.3 m in the `laser` frame) is removed |
| `feet_filter.launch.py` | `config/feet_filter.yaml` | `/scan_filtered` → `/scan_feet` | the same box with `invert: true`: only the active area is kept |
| `filters.launch.py` | | `/scan` → all three | the three above |

A removed return becomes NaN, as in `LaserScanBoxFilter`; readings that are not returns (NaN, inf, `range_max`) are not touched.
`walker_rviz` draws the active area from `feet_filter.yaml`. A test checks that `nav_filter.yaml` and `feet_filter.yaml` are the same
box with opposite `invert`.

```bash
ros2 launch walker_laser_filter filters.launch.py                    # on the robot (tmule starts self_filter and nav_filter)
ros2 launch walker_laser_filter filters.launch.py sim_time:=true     # replaying a bag; debug:=true for DEBUG logs
ros2 launch walker_laser_filter feet_filter.launch.py in_topic:=/scan_filtered out_topic:=/scan_feet config:=/path/other.yaml
```

Every launcher takes `sim_time` (false), `debug` (false), `config`, `in_topic` and `out_topic`. The nodes are
`/walker_laser_filter/{self,nav,feet}_laser_filter`.

**Including more than one stage from another launch file: use `walker_laser_filter.launching.include_stage`.** The arguments of an
`IncludeLaunchDescription` (`config`, `in_topic`, …) and the defaults the included launcher declares stay in the launch context,
and the next stage then takes them instead of its own, **silently**: an early version of `filters.launch.py` ran the three stages
with `self_filter.yaml` and `/scan → /scan_filtered`. `include_stage('nav_filter.launch.py', sim_time=..., in_topic=...)` includes a
stage in a scope of its own. `test/test_filters_launch.py` starts `filters.launch.py` and checks what each node was given.

## Making `config/self_filter.yaml`

The lidar is fixed to the walker, so the walker's structure returns **the same short range on the same beam in every scan**; the
user's legs, walls and people move with respect to it. `find_self_collision_boxes.py` finds those beams in recorded scans and writes
a box per patch:

```bash
ros2 run walker_laser_filter find_self_collision_boxes.py ~/bagsFolder/definitive                        # report only
ros2 run walker_laser_filter find_self_collision_boxes.py ~/bagsFolder/definitive --out config/self_filter.yaml
```

For each bag: a beam's reference is the 10th percentile of its returns (the structure is the nearest thing there is), only if it is
closer than 1 m (`--max-dist`), and its *persistence* is the fraction of the bag's scans in which it returns within 3 cm
(`--tolerance`) of that. A beam is structure in a bag if its persistence is at least 0.5 (`--min-persistence`), and a beam of the
walker if it is so in at least 3 bags (`--min-bags`: nobody stands still for half of three recordings). Neighbouring beams (up to 2
beams apart, at least 3 of them) form a patch, and its box is the bounding box of their points in every bag where they are
structure, plus 3 cm (`--margin`). The criterion is in `walker_laser_filter/self_collision.py` (numpy only, with tests); the comment
at the top of the generated file says which data and options it came from.

It is done bag by bag and then put together, not on all the scans at once: the edge beams of a post flip between the post and the
background differently from one session to the next, so pooled they never reach the persistence they have inside each session and
their returns stayed in the scan as small clusters. Reading the bags is `walker_replay.bagio`.

**Measured on the 47 definitive bags** (24314 scans of 1080 beams, four users): 5 boxes, in four patches of the walker's structure
(−157…−153°, +144…+151°, −93…−88° and +84…+89° from the lidar's x axis, at 0.23–0.55 m) plus one more next to the first: beams 84–91,
rear right, which is structure in 8 bags, MT_test11 to MT_test18, and in none of CA, LA or MF. The same four patches come out for every
`--min-persistence` between 0.4 and 0.8 (5 boxes; 6 from 0.6, when the rear-left patch splits in two) and for each user's bags on
their own (MT's with the extra one). The boxes remove 51.7 % of the returns closer than 1.5 m, and let through **none** of the 940260
returns that are structure within their own bag. The four hand-made boxes that were here before (x ±0.07 and −0.55…−0.3, y ±(0.16…0.36),
much bigger than the patches) removed 51.8 % and let through 2322 of them (0.25 %); they also removed 0.30 % of the close returns
that are not structure (x −0.55…−0.42 at the rear; what those are was not checked), which the empirical boxes keep.

## What changed from the previous version

The package used to have its own `box_filter`, `mask_filter` and `all_filter` nodes, and the robot's chain was launched from
`walker_bringup` (`laser_filter.launch.py`, `nav_laser_filter.launch.py`). Replaying five bags through the old and the new launchers
at the same time (the same scans, every message compared):

| | old → new |
|---|---|
| `/scan_filtered` | identical in 2198 of 2198 scans (with the same boxes) |
| `/scan_feet` | identical in 2209 of 2209 scans: the home-made `box_filter` and the official one with `invert: true` do the same |
| `/scan_nav` | **the old one did nothing**: it was identical to `/scan_filtered` in 2218 of 2230 scans. Its parameter file started with `nav_laser_filter:` and the node was launched in the namespace `nav`, so it received none of its parameters (0 `filter*` parameters in `/nav/nav_laser_filter`, 20 in the root namespace). The new one removes the active area, and matches the numpy definition in every scan |
| `/scan_feet` input | it was cut from the raw `/scan` in every evaluation; now from `/scan_filtered`, as on the robot, so the walker's structure is not in it |

## Effect on the final metrics

Measured against the mocap, old code and new, on the same bags and with the tools of the mocap4w repos (replayed at rate 4).

**Steps** (8 bags: CA_test03/07, LA_test02/06, MF_test03/12, MT_test13/18; `mocap_step_detector/scripts/run_multibag_eval.py`).
Share of detections with a mocap foot within 0.3 m:

| variant | before (home-made filter on raw `/scan`) | now (`/scan_filtered` → `feet_filter`) |
|---|---|---|
| `rf` left / right | 98.1 / 86.2 % | 99.7 / 96.3 % |
| `km` left / right | 64.5 / 63.0 % | 99.3 / 99.5 % |
| `seg` left / right | 53.6 / 62.2 % | 99.5 / 95.6 % |
| `fused` left / right | 94.3 / 83.7 % | 99.8 / 99.7 % |

The median error per bag and variant falls 3.5 cm on average (from −21.5 to +8.1 cm over the 64 cases). The step evaluations used to cut
`/scan_feet` from the raw scan, where the walker's own structure is inside the active area: that is the difference. With the hand-made
boxes instead of the empirical ones the numbers are almost the same (`rf` identical; `km` right 95.8 instead of 99.5 %). The robot's own
detector, which reads `/scan_filtered`, gives the same with either set of boxes (99.7 / 96.2 % matched, median error 5.7 / 6.5 cm, the
medians differing by 0.04 cm), so it does not change.

**Laser + IMU odometry** (6 bags: CA_test03/05, LA_test04, MF_test07/14, MT_test13; `mocap_odometry` `compare_odometry.launch.py`;
mean of the six): A = the old behaviour (hand-made boxes and a `/scan_nav` that did nothing), B = only the active area removed from
`/scan_nav`, C = the chain as it is now.

| | A | B | C |
|---|---|---|---|
| ATE | 0.267 m | 0.246 m | 0.241 m |
| heading error | 6.2° | 5.7° | 5.5° |
| drift | 1.63 % | 1.53 % | 1.52 % |
| scale | 1.143 | 1.160 | 1.153 |
| 1 s relative error | 0.080 m | 0.077 m | 0.076 m |

A reproduces the figure `mocap_odometry` documents for the old pipeline (CA_test05: ATE 0.198 m, heading 5.7°). C is better than A in 4 of 6
bags for ATE, heading and drift, and worse in 5 of 6 for scale. CA_test05 improves most (0.198 → 0.110 m, 5.7° → 2.6°), LA_test04, MF_test14 and MT_test13
a little, CA_test03 stays (0.345 → 0.347 m) and MF_test07 gets worse (0.101 → 0.122 m). With six bags this is "no worse on average", not an improvement shown. The scan matcher and EKF
parameters (`odom_tuning`) were tuned with the unfiltered `/scan_nav` and have not been tuned again.

## Seeing it in rviz

The views of `walker_rviz` draw this chain: `walker` shows `/scan_filtered` and `/scan_feet` (`/scan` and `/scan_nav` are there, off),
`step_detector` only `/scan_feet`, `bringup_demo` `/scan_filtered`; the `active_area` layer outlines the box of `feet_filter.yaml`
(the same one `nav_filter.yaml` removes). They were tried on screen with a replay of `mocap_analysis/replay_feet.launch.py`, which starts
these launchers; see the README of `walker_rviz`. The laser points of its `test_steps.rviz` are 3 cm: the 1 cm ones were hard to see.

## Notes

- `LaserScanBoxFilter` is a lifecycle node that `laser_filters` names `laser_scan_box_filter` whatever its name in the chain, so a chain
  with several makes `rcl` warn once per extra one ("Publisher already registered for node name"). The launchers raise the level of the
  `rcl.logging_rosout` logger to error for that process; nothing else is lost.
- The filter transforms the scan into `box_frame` (here `laser`, the scan's own frame, so no tf is needed).
- The old mask-filter tools (`find_minimums.py`, `laser_filter_generator.py`: scan indices below a threshold, thresholds hardcoded) are
  gone, with the mask filter they fed; they are in the history. `find_self_collision_boxes.py` does that job from recorded bags and
  with boxes, which do not depend on the beam grid.

## Tests

```bash
python3 -m pytest test        # with the workspace sourced: test_filters_launch.py starts the launchers
```
