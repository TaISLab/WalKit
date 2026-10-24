"""The walker's measures as rviz markers (node walker_markers, namespace /walker_rviz).

Arguments: sim_time (false: use_sim_time, true when replaying a bag), debug (false: DEBUG log level), lifetime_s (0.5),
detected_steps_topic_name (/detected_step: the layer reads <that>_left and <that>_right), handles_raw (false: draw the handles
from the sensors' own /left_handle and /right_handle, in kg by walker_loads' calibration table, for a bag with no loads chain), and
show_handles, show_steps, show_detected_steps (false), show_centroid, show_stability, show_gait, show_active_area (all true
but the third). Topics: /walker_rviz/{handles,steps,detected_steps,centroid,stability,gait,active_area}; the support polygon is
/support_polygon (walker_stability), drawn by rviz itself.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LAYERS = (('handles', 'true'), ('steps', 'true'), ('detected_steps', 'false'), ('centroid', 'true'), ('stability', 'true'),
          ('gait', 'true'), ('active_area', 'true'))


def node_actions():
    params = {'use_sim_time': ParameterValue(LC('sim_time'), value_type=bool),
              'lifetime_s': ParameterValue(LC('lifetime_s'), value_type=float)}
    params['detected_steps_topic_name'] = LC('detected_steps_topic_name')
    params['handles_raw'] = ParameterValue(LC('handles_raw'), value_type=bool)
    params.update({'show_' + name: ParameterValue(LC('show_' + name), value_type=bool) for name, _ in LAYERS})
    return [Node(package='walker_rviz', executable='walker_markers', namespace='walker_rviz', name='walker_markers',
                 output='screen', parameters=[params],
                 arguments=['--ros-args', '--log-level',
                            PythonExpression(["'debug' if '", LC('debug'), "' == 'true' else 'info'"])])]


def generate_launch_description():
    args = [DeclareLaunchArgument('sim_time', default_value='false', description='use_sim_time'),
            DeclareLaunchArgument('debug', default_value='false', description='DEBUG log level'),
            DeclareLaunchArgument('lifetime_s', default_value='0.5', description='how long a marker stays on screen'),
            DeclareLaunchArgument('detected_steps_topic_name', default_value='/detected_step',
                                  description='the detector whose steps the detected_steps layer draws, without _left/_right'),
            DeclareLaunchArgument('handles_raw', default_value='false',
                                  description='draw the handles from the raw /left_handle, /right_handle (kg by the calibration table)')]
    args += [DeclareLaunchArgument('show_' + name, default_value=default) for name, default in LAYERS]
    return LaunchDescription(args + node_actions())
