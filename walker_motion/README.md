# walker_motion

How the walker's chassis moves, from its IMU (`/bwt901cl/imu`): the vibration the gait puts into it and how much it turns. Node `chassis_motion`
publishes `/chassis_motion` (`walker_msgs/ChassisMotion`) every second of data time over the last 20 s.

```bash
ros2 launch walker_motion chassis_motion.launch.py            # sim_time:=true when replaying a bag, debug:=true, window_s, period_s
python3 -m pytest test
```

Every number is defined in `walker_motion/chassis_motion.py` (pure python), which `mocap_motion` (mocap4w) uses too, so the walker and the
mocap differ only in the signals they feed in: RMS of the AP and ML acceleration, harmonic ratio (Moe-Nilssen 1998), step and stride regularity
(Moe-Nilssen & Helbostad 2004), the stride time found in the signals themselves, and the yaw rate (RMS and the fraction of the window above
15 deg/s). It replaces `walker_mocap_eval/mocap_eval/imu_metrics.py`, whose formulas it keeps.

The accelerations and the yaw rate are those of the IMU rotated into `base_footprint` with the static transform of the IMU frame (it is not
assumed to be mounted aligned; without the transform it takes it as aligned and warns once). The mean is removed from every signal, so the
gravity that a tilt of the chassis puts on x and y does not matter. The vertical axis is left out. The IMU is on the chassis, not on the
user's trunk: the values do not compare with the norms of the literature, only the walker against the mocap.

Data time: the clock is never read; the output carries the stamp of the tick.
