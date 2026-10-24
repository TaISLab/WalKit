"""How far the user's arms reach to the handles, from where the walker thinks the user is. Pure python (no ROS).

The walker has no sensor on the arms. What it has is an estimate of the user's centre of mass (stability_model.py) and the position of
the handles, and the arm reach follows the distance between them: with the mocap's centre of mass, over the 47 definitive bags and
between the two /tag messages, the distance from the centre of mass to the handles tracks the horizontal distance from the shoulders
to the hands with a correlation of 0.82 (0.70 to 0.92 per user) and, with ONE constant, a mean absolute error of 4.1 cm:

    reach = handle_distance - ARM_REACH_OFFSET_M          (0.108 m: the shoulders are ahead of the centre of mass by the lean of the
                                                           trunk, and the wrists are short of the handle)

What it does not give is the elbow angle: with the measured reach, the lengths of the arm segments from the anthropometric regressions
(good to 2-4 cm) and the handle height, the law of cosines gives the angle with a mean absolute error of 17 degrees and no correlation
within a bag; the arm is nearly straight and a centimetre of error in the shoulder-to-wrist distance is several degrees.
"""

ARM_REACH_OFFSET_M = 0.108


def handle_distance(handle_x, com_x):
    """Horizontal distance (m) from the centre of mass to the handles, along x of the walker's frame: + when the user is behind them
    (x grows forward in the frame of the step positions, the laser's)."""
    return handle_x - com_x


def reach(distance, offset=ARM_REACH_OFFSET_M):
    """Estimate of the horizontal shoulder-to-hand distance (m) from the handle distance."""
    return distance - offset
