from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Default filenames and where to find them
    # slam_dir = get_package_share_directory('slam_toolbox')
    # slam_launch_dir = os.path.join(slam_dir, 'launch')

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    log_config = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(log_config)

    # Create the launch configuration variables: 
    log_level = LaunchConfiguration('log_level')
    tf_rate_hz = LaunchConfiguration('tf_rate_hz')
    odom_topic_name = LaunchConfiguration('odom_topic_name')
    left_wheel_encoder_topic_name = LaunchConfiguration('left_wheel_encoder_topic_name')
    right_wheel_encoder_topic_name = LaunchConfiguration('right_wheel_encoder_topic_name')
    ticks_meter = LaunchConfiguration('ticks_meter')
    base_width = LaunchConfiguration('base_width')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    encoder_min = LaunchConfiguration('encoder_min')
    encoder_max = LaunchConfiguration('encoder_max')
    wheel_low_wrap = LaunchConfiguration('wheel_low_wrap')
    wheel_high_wrap = LaunchConfiguration('wheel_high_wrap')

    # Map these variables to Arguments: can be set from the command line or a default will be used
    log_level_launch_arg = DeclareLaunchArgument("log_level", default_value=TextSubstitution(text="INFO"))
    
    tf_rate_hz_arg = DeclareLaunchArgument('tf_rate_hz', default_value=TextSubstitution(text="60.0"))
    odom_topic_name_arg = DeclareLaunchArgument('odom_topic_name', default_value=TextSubstitution(text="odom"))
    left_wheel_encoder_topic_name_arg = DeclareLaunchArgument('left_wheel_encoder_topic_name', default_value=TextSubstitution(text="left_wheel"))
    right_wheel_encoder_topic_name_arg = DeclareLaunchArgument('right_wheel_encoder_topic_name', default_value=TextSubstitution(text="right_wheel"))
    ticks_meter_arg = DeclareLaunchArgument('ticks_meter', default_value=TextSubstitution(text="74072.5")) # In theory 76694, as small wheels have 17mm diam. and 4096 ticks )
    base_width_arg = DeclareLaunchArgument('base_width', default_value=TextSubstitution(text="0.489")) # In theory 0.57
    base_frame_id_arg = DeclareLaunchArgument('base_frame_id', default_value=TextSubstitution(text="base_link"))
    odom_frame_id_arg = DeclareLaunchArgument('odom_frame_id', default_value=TextSubstitution(text="odom"))
    encoder_min_arg = DeclareLaunchArgument('encoder_min', default_value=TextSubstitution(text="0"))
    encoder_max_arg = DeclareLaunchArgument('encoder_max', default_value=TextSubstitution(text="4096"))
    wheel_low_wrap_arg = DeclareLaunchArgument('wheel_low_wrap', default_value=TextSubstitution(text="1228"))
    wheel_high_wrap_arg = DeclareLaunchArgument('wheel_high_wrap', default_value=TextSubstitution(text="2867"))

    # Declare defined launch options 
    ld.add_action(log_level_launch_arg)
    
    ld.add_action(tf_rate_hz_arg)
    ld.add_action(odom_topic_name_arg)
    ld.add_action(left_wheel_encoder_topic_name_arg)
    ld.add_action(right_wheel_encoder_topic_name_arg)
    ld.add_action(ticks_meter_arg)
    ld.add_action(base_width_arg)
    ld.add_action(base_frame_id_arg)
    ld.add_action(odom_frame_id_arg)
    ld.add_action(encoder_min_arg)
    ld.add_action(encoder_max_arg)
    ld.add_action(wheel_low_wrap_arg)
    ld.add_action(wheel_high_wrap_arg)

    diff_tf_node = Node(
        package='walker_diff_odom',
        executable='walker_odom_node',
        name='walker_odom',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[ {'tf_rate_hz': tf_rate_hz},
                     {'odom_topic_name': odom_topic_name},
                     {'left_wheel_encoder_topic_name': left_wheel_encoder_topic_name},
                     {'right_wheel_encoder_topic_name': right_wheel_encoder_topic_name},
                     {'ticks_meter': ticks_meter},
                     {'base_width': base_width},
                     {'base_frame_id': base_frame_id},
                     {'odom_frame_id': odom_frame_id},
                     {'encoder_min': encoder_min},
                     {'encoder_max': encoder_max},
                     {'wheel_low_wrap': wheel_low_wrap},
                     {'wheel_high_wrap': wheel_high_wrap}
                    ]
    )
    
    ld.add_action(diff_tf_node)

    return ld