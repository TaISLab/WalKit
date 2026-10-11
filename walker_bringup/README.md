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

## Dependencies

Declared in `package.xml`. Everything resolves through rosdep except `witmotion_ros`
(see above). The old stereo visual odometry (`viso2_ros`) is no longer a dependency.
