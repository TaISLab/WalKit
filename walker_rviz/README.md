# walker_rviz

Everything the walker draws in rviz lives here: the node that turns what it measures into markers, the launchers, and the rviz
views. It replaces `walker_plot`, `walker_py_cluster` (dead code: it referred to attributes it never defined and published
nothing), `walker_loads/plot_loads.py`, `walker_step_detector/plot_steps` and the rviz files that were scattered over
`walker_bringup`, `walker_description`, `walker_navigation2` and `walker_step_detector`.

```bash
ros2 launch walker_rviz walker_view.launch.py                    # rviz2 with the `walker` view + the markers node
ros2 launch walker_rviz walker_view.launch.py view:=robot sim_time:=true markers:=false
ros2 launch walker_rviz walker_markers.launch.py show_detected_steps:=true      # only the node
python3 -m pytest test
```

To see a recorded bag, replay it with whatever computes what you want to see and open the view in another terminal with
`sim_time:=true` (rviz must run on the bag's clock). Both pairs below were tried on screen, see [Tried on screen](#tried-on-screen):

```bash
# feet, loads, centroid, stability, support polygon (views walker, step_detector, bringup_demo)
ros2 launch mocap_analysis replay_feet.launch.py bag_name:=MF_test07          # replay + laser filters + detector + loads + centroid
ros2 launch walker_stability walker_stability.launch.py sim_time:=true         # /user_stability and /support_polygon
ros2 launch walker_rviz walker_view.launch.py view:=walker sim_time:=true

# robot model following the walker, with the laser-inertial odometry (view robot)
ros2 launch mocap_odometry compare_odometry.launch.py bag_name:=MF_test07
ros2 launch walker_rviz walker_view.launch.py view:=robot sim_time:=true
```

## The node: `walker_markers` (namespace `/walker_rviz`)

One node, one layer per kind of data; each layer is a `MarkerArray` topic and can be switched off with `show_<layer>:=false`.
A marker takes the **stamp and frame of the message it draws** (no clock, no timers, nothing kept between messages), so the
picture follows the data: a bag replayed at any rate looks the same, and each thing is drawn in the frame it is expressed in.

| Layer (topic) | From | Draws |
|---|---|---|
| `handles` | `/left_hand_loads`, `/right_hand_loads` (calibrated kg, `walker_loads`); or, with `handles_raw:=true`, the sensors' own `/left_handle`, `/right_handle` turned into kg with `walker_loads`' calibration table (for a bag viewed on its own, with no loads chain; linear interpolation, `partial_loads` uses a spline) | an arrow down onto each handle, as long and as red as the load, and the kg as text. (`plot_forces` drew the *raw* sensor value, not kg.) |
| `steps` | `<detected_steps_topic_name>_left/_right` (a step detector) for where, `/left_loads`, `/right_loads` (`walker_loads`) for how much | each leg's tracked foot on the floor (sphere = left, cube = right), coloured by the share of the user's weight it carries (`/user_desc`), transparent as the detection's confidence, with its label and kg. The foot is drawn at the **detected step** and coloured with the **last load known**, so it stays on the laser scan (a detected step is about as old as the scan, ~0.13 s; the loads message that carries the same step comes out ~0.7 s later, because `walker_loads` waits for the handles, 2.4 Hz). No foot is drawn until its first load has arrived. `steps_at_detected:=false` draws it where the loads message puts it (the old behaviour) |
| `detected_steps` (off) | `<detected_steps_topic_name>_left/_right` (a step detector) | the same shapes in a fixed colour per leg, for detectors that have no load |
| `centroid` | `/support_centroid` (`walker_centroid_support`) | a sphere on the floor |
| `stability` | `/user_stability` (`walker_stability`) | the centre-of-mass estimate as a sphere, red to green by `sta`, and `margin:…cm\|sta:…\|contacts:…` (no spaces: this RViz draws a space far too wide) above it |
| `gait` | `/global_gait_stats` (`walker_loads`) | the gait summary (steps, cadence, speed, distance) as a text floating over `base_link` |
| `active_area` | the filters' own YAML (`active_area_files`, default `walker_laser_filter/config/feet_filter.yaml`) | the boxes as outlines, published once and latched. (`plot_active_area` had the numbers typed in, and out of date.) |

The **support polygon** of `walker_stability` is not drawn by this node: that node publishes it as a `PolygonStamped`
(`/support_polygon`) and rviz draws that natively (it is in the views).

Parameters: `lifetime_s` (0.5), `handle_max_kg` (20: the load that fills the arrow), `step_size_m` (0.15), `default_weight_kg`
(60, until `/user_desc` arrives), `min_confidence` (0, for `detected_steps`), `steps_at_detected` (true, see below), `handles_raw`
(false), `gait_frame_id` (`base_link`), the topic names, and `show_*`. The launcher exposes `sim_time`, `debug`, `lifetime_s`,
`detected_steps_topic_name`, `handles_raw`, `steps_at_detected` and the `show_*`.

**Labels have no spaces** (`left:45kg`, `margin:+8.3cm|sta:0.80|contacts:4`): this RViz draws every kind of space tens of times too
wide, so `left 45 kg` came out as three words a metre apart. `_text()` in `markers.py` turns any whitespace into `_`, and a test
looks at every label.

### How late each layer is

Each marker carries the stamp of the data it is made from, and the data do not all come out at the same delay. Measured on
MF_test07 replayed at rate 1 (bag clock minus the message's stamp):

| Data | Delay | Why |
|---|---|---|
| `/scan_feet`, `/detected_step_*`; the `steps` and `detected_steps` layers | 0.13 s | one laser sweep: the scan is stamped at the start of the sweep and published at its end. It is the floor |
| `/left_hand_loads` (`handles` layer) | 0.39 s | the handle messages arrive with a stamp 0.41 s older than their arrival (their period; 2.4 Hz) |
| `/left_loads`, `/right_loads` | 0.73 s | `walker_loads` orders its inputs by stamp and waits for the handles |
| `/user_stability` (`stability` layer) | 0.85 s (0.90 s with `stamp_ordered:=true`) | the legs' own delay: it is published with every input message and stamped with the data time the legs have reached; `stamp_ordered:=true` orders the inputs again and waits for the slowest |
| `/support_centroid` (`centroid` layer) | 0.85 s (0.94 s with `stamp_ordered:=true`) | the same |

(The two 0.85 s are the median over MF_test07, CA_test03, LA_test04 and MT_test13 replayed at rate 4, the rest of the table is
MF_test07 at rate 1.) So in a replay the centroid and the stability marker trail the laser points by ~0.7 s; at walking pace that is
several centimetres to tens of centimetres. They cannot be more recent than the leg loads they are made from, but a change in any
input, a handle's included, shows in them at once instead of 0.1-0.6 s later. The feet, on the other hand, do not trail the scan: with `steps_at_detected:=true` (the default) the `steps` layer draws each foot at the
**detected step** (as old as the scan) and colours it with the **last load** `walker_loads` gave for that side, which changes slowly;
no foot is drawn until its first load has arrived. `steps_at_detected:=false` draws it where the loads message puts it (0.73 s
late). On MF_test07 the distance from each foot marker to the nearest laser point of the last scan, which is what is on screen, went
from a median of 4.9 / 9.0 cm (left / right) and a p90 of 19-20 cm to 2.7 / 5.8 cm and 3.2 / 9.1 cm: what is left is the radius of
the leg, as the marker goes to the centre of the cluster.

## Views (`rviz/`)

`walker_view.launch.py view:=<name>` opens rviz2 with `rviz/<name>.rviz` (or any file with `rviz_config:=`) and starts the
markers node unless `markers:=false`.

| View | What it is | Where it was |
|---|---|---|
| `walker` | **new**: the robot model, scans (filtered and feet on), TF, detector clusters, and all the layers above plus the support polygon | -- |
| `robot` | the robot model with the scan and the odometry | `walker_description/rviz2/walker.rviz` |
| `bringup_demo` | scans, odometry, point cloud, clusters, plus the layers | `walker_bringup/config/demo_view.rviz` |
| `vicon` | the same against a Vicon world (`vicon_world`) | `walker_bringup/config/vicon_view.rviz` |
| `step_detector` | the step detector's work: scans, feet, segments, plus the layers (detected steps on) | `walker_step_detector/config/test_step_detector.rviz` |
| `navigation` | the navigation stack's view: map, planners, particle cloud | `walker_navigation2/rviz2/walker_navigation2.rviz` |

The launchers that opened the old files (`walker_bringup/launch/test/{rviz2,walker_rviz2}.launch.py`,
`walker_step_detector/launch/test_step_detector_variants.launch.py` with `use_rviz`, `walker_navigation2/launch/navigation2.launch.py`)
open these now. The `tmule` sessions `walker_demo.yaml` and `walker_leap.yaml` start `walker_markers` instead of the three old
nodes; in `walker.yaml` the block stays commented, as it was ("until we do a full reworking of the whole visualization"), with
the new command in it.

## What stays elsewhere, on purpose

- The markers that the step detectors publish about their own work (`/clusters`, `/segments_viz`) need the detectors' internal
  state; the views only show them.
- The rviz files of `mocap4w` (`mocap_analysis`, `mocap4w_body`, `human_description`) draw mocap data and live with it. The
  `show_rviz` option of `mocap_analysis/launch/replay_feet.launch.py` launched the removed nodes (and a `plot_centroid` that
  had not existed for a while); it uses `walker_markers` now.
- `stability_grid_map` has its own demo and rviz file for its grid map.

## Tried on screen

The views were rewritten from the old files with a script and first only tested through the node. They were then opened, one by one,
with a definitive bag replayed at rate 1 (MF_test07), with the Displays panel visible to read the status of every display:

| View | Fed by | Result |
|---|---|---|
| `walker` | `mocap_analysis replay_feet` + `walker_stability` | works: model, scans, feet with their kg, handles, centroid, centre of mass, support polygon and active area; every enabled display is green |
| `step_detector` | the same, with `show_detected_steps:=true` | works, the detected steps are drawn next to the feet |
| `bringup_demo` | the same | works; its `PointCloud2` (`/laser_cloud`) shows nothing: no node of the workspace publishes that topic |
| `robot` | `mocap_odometry compare_odometry` | works. It looked at the origin of `odom`, so the walker left the picture a few seconds into the bag, and its odometry arrows (1 m long, one every 10 cm) filled the rest: the camera now follows `base_link` and the arrows are 0.25 m long, one every 25 cm |
| `vicon` | -- | loads, but a bag has no `vicon_world` frame: `Frame [vicon_world] does not exist`. It needs a live Vicon |
| `navigation` | -- | loads (the nav2 plugins are there), but there is no map and no particle cloud: `Frame [map] does not exist`. It needs the navigation stack |

Things to know when you open them:

- The `Gait` layer shows nothing with `replay_feet`: it draws `/global_gait_stats`, which `walker_loads/gait_monitor_speed` publishes (it needs `/odom`) and `replay_feet` does not run.
- The `walker` and `bringup_demo` views start with the left panel collapsed (click the arrow at the left edge to see the Displays).
- `walker_markers` takes `sim_time`; the rviz that `walker_view.launch.py` starts takes it from the same argument.
- The tests (`test/`) cover the markers, the node (it is built with every layer on and its topics are checked; a foot is drawn at the
  detected step with the last load and not before one is known) and the labels.
