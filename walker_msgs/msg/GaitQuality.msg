# Quality of the gait over the stance onsets of the last window (walker_loads/gait_quality.py defines every number; the
# mocap counterpart, mocap_loads, publishes the same message). NaN when it cannot be computed yet.
std_msgs/Header header

# Data time (s) the numbers cover: the window, or the time since the first onset if it is shorter
float64 window_s

# Stance onsets (steps) of both legs inside the window
uint16 steps

# Mean lateral gap (m) between the feet at the stance onsets
float64 step_width

# Fraction (0-1) of the window with both legs in stance
float64 double_support

# Mean step time (s): from the opposite leg's onset to this one's
float64 step_time

# Symmetry index of the step time (%): |left - right| / (0.5 (left + right)) * 100 (Robinson 1987)
float64 step_time_si

# Coefficient of variation of the step time (%)
float64 step_time_cv
