# How the walker's chassis moves over the last window: the vibration the gait puts into it and how much it turns. Computed from the
# chassis IMU by walker_motion (walker_motion/chassis_motion.py defines every number); the mocap counterpart (mocap_motion) computes the
# same from the rigid body of the walker and publishes the same message. NaN when it cannot be computed yet.
std_msgs/Header header

# Data time (s) the numbers cover
float64 window_s

# RMS (m/s2) of the horizontal acceleration along the walker's x axis (antero-posterior) and y axis (medio-lateral), mean removed
float64 rms_ap
float64 rms_ml

# Harmonic ratio (Moe-Nilssen 1998) of both, from the stride time below: even over odd harmonics for AP, odd over even for ML. Higher = smoother
float64 hr_ap
float64 hr_ml

# Autocorrelation of the AP acceleration at the step time (half the stride) and at the stride time (Moe-Nilssen & Helbostad 2004). 1 = periodic
float64 step_regularity
float64 stride_regularity

# Stride time (s) found in the signals themselves (autocorrelation peak between 0.8 and 2.4 s)
float64 stride_time

# RMS (deg/s) of the yaw rate and the fraction (0-1) of the window turning faster than 15 deg/s
float64 yaw_rate_rms
float64 turning_fraction
