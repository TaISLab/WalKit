# walker_description

The walker's model and its tf.

| | |
|---|---|
| `urdf/walker_model.xacro` (and `.gazebo`), `models/` | the model and its assets |
| `launch/walker_state_publisher.launch.py` | `robot_state_publisher` (node `walker_state_publisher`): `/robot_description` and the tf of the links |
| `scripts/handle_tf_publisher.py` | the tf of the handle frames (`left_handle_id`, `right_handle_id`), which `walker_centroid_support` and `walker_stability` need to know where the handles are |
| `config/params.yaml` | where each of the handle's five holes (heights 0-4) puts the handle frame: `handle_x`, `handle_y`, `handle_z`, in `base_footprint` |

## Seeing it in rviz

The view that used to live here (`rviz2/walker.rviz`) is the view `robot` of `walker_rviz`
(`ros2 launch walker_rviz walker_view.launch.py view:=robot`: the model, the scan and the odometry; the camera follows `base_link`), and
the model is also in the view `walker`. In a bag replay `walker_replay` publishes `/robot_description` and the static tf with this same
`walker_state_publisher`, so the `RobotModel` display has what it needs. Both views were tried on screen with a definitive bag.
