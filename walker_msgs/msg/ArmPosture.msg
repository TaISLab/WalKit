# How far the user's arms reach to the handles. The walker estimates it from the distance between its estimate of the user's centre of
# mass and the handles (walker_stability); the mocap measures it (mocap_stability). Same message, NaN where a side cannot know.
std_msgs/Header header

# Horizontal distance (m) from the projection of the centre of mass to the handles, along the walker's x axis (+ when the user is behind them)
float64 handle_distance

# Horizontal distance (m) from the shoulders to the hands (wrists), + when the hands are ahead of the shoulders. The walker's is
# handle_distance minus a constant (walker_stability/arm_posture.py)
float64 reach

# Interior angle (deg) of the elbows (mean of both). Only the mocap measures it: it cannot be predicted from the reach and the arm lengths
# with centimetre inputs (it is the angle of a nearly straight arm), see walker_stability/README.md. NaN on the walker side.
float64 elbow
