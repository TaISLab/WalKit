# walker_bringup

Bringup package of the Walkit walker, in the style of a classic robot bringup package: it holds
the system configuration and the launchers that start the sensors, localization and mapping.
Other packages can reuse its launch files and configs. The system is started with
[TMuLE](tmule/), not with a single top-level launch file.

## Layout

| Folder    | Contents |
|-----------|----------|
| `tmule/`  | TMuLE sessions: the reference for how the system is brought up (`walker.yaml`, `walker_sensors.yaml`, `walker_rtab.yaml`, `walker_demo.yaml`, `walker_leap.yaml`) and `walker_cfg.bash`. |
| `launch/` | Launch files used by the tmule sessions: sensors (`walker_witmotion`, laser filters), localization (`walker_amcl`, `walker_ekf`, `walker_lsm`, `walker_laser_imu_odom`), mapping (`walker_rtabmap`, `rtabmap_stereo`, `walker_stereo_odom`), `walker_map_server`, `bag_recorder`. |
| `launch/test/` | Launchers not used by any tmule session (cameras, rviz, SLAM, bag replay and odometry evaluation). Still installed and launchable with `ros2 launch walker_bringup <file>`. |
| `config/` | Parameter files and rviz configs for the above. |
| `maps/`   | Placeholder for the maps generated on the walker (empty in the repository). |
| `udev/`   | How to create the `/dev/rplidar` and `/dev/witmotion` udev rules. See [udev/README.md](udev/README.md). |
| `scripts/`, `script/` | `record_service.py` (`/rosbag_record` service) and the `command_*.sh` it calls; `walker_config.bash` sourced by the tmule sessions. |

## Setting up a machine

1. Create the udev rules for the lidar and the IMU: [udev/README.md](udev/README.md).
2. Copy `tmule/walker_cfg.bash` to `~/.walker_cfg.bash` and set `ROS2_WORKSPACE`,
   `NETWORK_INTERFACE` and `MAP_YAML` (put your map in `maps/`).
3. Build the workspace, including `witmotion_ros` (`src/witmotion/witmotion_ros`), which is
   not a released package and must be installed manually.

## Running

```bash
tmule -c tmule/walker.yaml launch      # full system
tmule -c tmule/walker.yaml terminate
```

Other sessions: `walker_sensors.yaml` (sensors only), `walker_rtab.yaml` (RTAB-Map),
`walker_demo.yaml`, `walker_leap.yaml`.

## Laser-inertial odometry and the gyro supervisor

`launch/walker_laser_imu_odom.launch.py` builds `odom -> base_footprint` from the 2D lidar and the raw
WitMotion gyro: `ros2_laser_scan_matcher` (gyro-seeded ICP) -> `/scan_odom`, fused with the raw gyro z
(`/bwt901cl/imu`) in a robot_localization EKF (`config/walker_ekf.yaml`). Details and measured results:
[walker_mocap_eval/readme.md](../walker_mocap_eval/readme.md).

By default (`supervise_imu:=true`) `scripts/odom_supervisor.py` sits between the sensors and the EKF
(`/scan_odom -> /scan_odom_ekf`, `/bwt901cl/imu -> /imu_ekf`) because robot_localization cannot switch
covariances by itself. It watches only **intrinsic** gyro failures and, when one persists, gives the laser
heading a strong covariance and the gyro an almost infinite variance:

| Failure | Detected as |
|---|---|
| IMU or its TF missing | no message for `imu_timeout` (1 s) / no `base_footprint <- witmotion` TF (after `startup_grace_s`, 10 s) |
| saturated or corrupt gyro | \|wz\| > `imu_max_rate` or NaN |
| wrong sign / dead gyro | laser turns >= `turn_min_deg` (20 deg) in `window_s` (4 s) and the gyro integrates the opposite or < 30 % |
| bias | on near-straight windows the gyro differs from the laser by > `bias_max` (0.02 rad/s) |

Sign and bias failures are latched: they are only cleared by positive evidence (a turn where the gyro agrees,
or a straight where it matches the laser). A plain laser-vs-gyro disagreement never switches, because in the
scan-matching-anomalous bags the gyro is right. State is published on `/odom_supervisor/status`
(`diagnostic_msgs/DiagnosticStatus`) and logged on every change.

With the supervisor the EKF's `odom0` Mahalanobis gate is raised from 5 to 100 sigma in the launch: at 5 sigma
the corrective laser heading is rejected once a bad gyro has pulled the filter more than ~10 deg.

Fault injection on 12 clean mocap bags (`mocap_odometry/scripts/odom_tuning/supervisor_test.py`), EKF
heading error without / with the supervisor: IMU drops at 25 s 3.3 -> 1.7 deg; inverted gyro sign 67 -> 6.4 deg;
+0.05 rad/s bias 6.3 -> 2.7 deg; normal operation 1.4 -> 1.4 deg with 1 spurious switch in 33 clean bags and
none in the 8 scan-matching-anomalous ones. Bags whose gyro integral is 5-7 % off the laser
(`imu_gyro_scale` in `bag_quality.yaml`) are correctly flagged as bias.

Without the supervisor (`supervise_imu:=false`) the EKF reads `/scan_odom` and `/bwt901cl/imu` directly.

## Dependencies

Declared in `package.xml`. Everything resolves through rosdep except `witmotion_ros`
(see above). The old stereo visual odometry (`viso2_ros`) is no longer a dependency.
