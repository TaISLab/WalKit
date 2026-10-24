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

## The node: `walker_markers` (namespace `/walker_rviz`)

One node, one layer per kind of data; each layer is a `MarkerArray` topic and can be switched off with `show_<layer>:=false`.
A marker takes the **stamp and frame of the message it draws** (no clock, no timers, nothing kept between messages), so the
picture follows the data: a bag replayed at any rate looks the same, and each thing is drawn in the frame it is expressed in.

| Layer (topic) | From | Draws |
|---|---|---|
| `handles` | `/left_hand_loads`, `/right_hand_loads` (calibrated kg, `walker_loads`); or, with `handles_raw:=true`, the sensors' own `/left_handle`, `/right_handle` turned into kg with `walker_loads`' calibration table (for a bag viewed on its own, with no loads chain; linear interpolation, `partial_loads` uses a spline) | an arrow down onto each handle, as long and as red as the load, and the kg as text. (`plot_forces` drew the *raw* sensor value, not kg.) |
| `steps` | `/left_loads`, `/right_loads` (`walker_loads`) | each leg's tracked foot on the floor (sphere = left, cube = right), coloured by the share of the user's weight it carries (`/user_desc`), transparent as the detection's confidence, with its label and kg |
| `detected_steps` (off) | `<detected_steps_topic_name>_left/_right` (a step detector) | the same shapes in a fixed colour per leg, for detectors that have no load |
| `centroid` | `/support_centroid` (`walker_centroid_support`) | a sphere on the floor |
| `stability` | `/user_stability` (`walker_stability`) | the centre-of-mass estimate as a sphere, red to green by `sta`, and `margin cm \| sta \| contacts` above it |
| `gait` | `/global_gait_stats` (`walker_loads`) | the gait summary (steps, cadence, speed, distance) as a text floating over `base_link` |
| `active_area` | the filters' own YAML (`active_area_files`, default `walker_laser_filter/config/inverted_box_filter.yaml`) | the boxes as outlines, published once and latched. (`plot_active_area` had the numbers typed in, and out of date.) |

The **support polygon** of `walker_stability` is not drawn by this node: that node publishes it as a `PolygonStamped`
(`/support_polygon`) and rviz draws that natively (it is in the views).

Parameters: `lifetime_s` (0.5), `handle_max_kg` (20: the load that fills the arrow), `step_size_m` (0.15), `default_weight_kg`
(60, until `/user_desc` arrives), `min_confidence` (0, for `detected_steps`), `gait_frame_id` (`base_link`), the topic names, and
`show_*`. The launcher exposes `sim_time`, `debug`, `lifetime_s`, `detected_steps_topic_name` and the `show_*`.

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

## Not tested

The markers node is tested with unit tests (`test/`) and by replaying a definitive bag through the walker chain and reading
back what it publishes. **The rviz views were not opened**: they were rewritten from the old files with a script (the old
marker displays replaced by the layers), and there was no display to check them on.
