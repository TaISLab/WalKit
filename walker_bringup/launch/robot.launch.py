#!/usr/bin/env python3
 
'''
    Launches nodes controlling walker hardware:
    - robot description
    - two handle controllers
    - two encoder wheels
    - odometry publisher
    - lidar

    TODO: 
        fix usb_arduino nodes
        lidar?
        move args to param file...

'''

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import os

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Default filenames and where to find them
    bringup_dir = get_package_share_directory('walker_bringup')
    diff_odom_dir = get_package_share_directory('walker_diff_odom')

    default_walker_param_file = os.path.join( bringup_dir, 'param', 'walker.yaml')
    walker_state_publisher_launch_file = os.path.join( bringup_dir, 'launch', 'walker_state_publisher.launch.py')
    walker_diff_odom_launch_file = os.path.join( diff_odom_dir, 'launch', 'walker_diff_odom.launch.py')
    

    ld = LaunchDescription()

    # Set env vars
    # to print messages to stdout immediately
    log_config = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(log_config)
   
    # Create the launch configuration variables: 
    log_level = LaunchConfiguration('log_level')
    walker_param_file = LaunchConfiguration( 'walker_param_file', default=default_walker_param_file)
    use_sim_time = LaunchConfiguration('use_sim_time')

    left_wheel_encoder_topic_name = LaunchConfiguration('left_wheel_encoder_topic_name')
    left_wheel_frame_id = LaunchConfiguration('left_handle_frame_id')
    left_wheel_preamble = LaunchConfiguration('left_handle_preamble')

    right_wheel_encoder_topic_name = LaunchConfiguration('right_wheel_encoder_topic_name')
    right_wheel_frame_id = LaunchConfiguration('right_handle_frame_id')
    right_wheel_preamble = LaunchConfiguration('right_handle_preamble')

    left_handle_topic_name = LaunchConfiguration('left_handle_topic_name')
    left_handle_frame_id = LaunchConfiguration('left_handle_frame_id')
    left_handle_preamble = LaunchConfiguration('left_handle_preamble')

    right_handle_topic_name = LaunchConfiguration('right_handle_topic_name')
    right_handle_frame_id = LaunchConfiguration('right_handle_frame_id')
    right_handle_preamble = LaunchConfiguration('right_handle_preamble')

    tf_rate_hz = LaunchConfiguration('tf_rate_hz')
    odom_topic_name = LaunchConfiguration('odom_topic_name')
    ticks_meter = LaunchConfiguration('ticks_meter')
    base_width = LaunchConfiguration('base_width')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    encoder_min = LaunchConfiguration('encoder_min')
    encoder_max = LaunchConfiguration('encoder_max')
    wheel_low_wrap = LaunchConfiguration('wheel_low_wrap')
    wheel_high_wrap = LaunchConfiguration('wheel_high_wrap')

    # Map these variables to Arguments: can be set from the command line or a default will be used    
    log_level_launch_arg = DeclareLaunchArgument( "log_level", default_value=TextSubstitution(text="DEBUG") )
    walker_param_file_arg = DeclareLaunchArgument('walker_param_file', default_value=default_walker_param_file)
    use_sim_time_launch_arg = DeclareLaunchArgument("use_sim_time",default_value='false', description='Use simulation (Gazebo) clock if true')

    left_wheel_encoder_topic_name_arg = DeclareLaunchArgument('left_wheel_encoder_topic_name', default_value=TextSubstitution(text="left"))
    left_wheel_frame_id_arg = DeclareLaunchArgument('left_wheel_frame_id', default_value=TextSubstitution(text="left_wheel_frame"))
    left_wheel_preamble_arg = DeclareLaunchArgument('left_wheel_preamble', default_value=TextSubstitution(text="LW"))

    right_wheel_encoder_topic_name_arg = DeclareLaunchArgument('right_wheel_encoder_topic_name', default_value=TextSubstitution(text="right"))
    right_wheel_frame_id_arg = DeclareLaunchArgument('right_wheel_frame_id', default_value=TextSubstitution(text="right_wheel_frame"))
    right_wheel_preamble_arg = DeclareLaunchArgument('right_wheel_preamble', default_value=TextSubstitution(text="RW"))

    left_handle_topic_name_arg = DeclareLaunchArgument('left_handle_topic_name', default_value=TextSubstitution(text="left"))
    left_handle_frame_id_arg = DeclareLaunchArgument('left_handle_frame_id', default_value=TextSubstitution(text="left_frame"))
    left_handle_preamble_arg = DeclareLaunchArgument('left_handle_preamble', default_value=TextSubstitution(text="LL"))

    right_handle_topic_name_arg = DeclareLaunchArgument('right_handle_topic_name', default_value=TextSubstitution(text="right"))
    right_handle_frame_id_arg = DeclareLaunchArgument('right_handle_frame_id', default_value=TextSubstitution(text="right_frame"))
    right_handle_preamble_arg = DeclareLaunchArgument('right_handle_preamble', default_value=TextSubstitution(text="RL"))

    tf_rate_hz_arg = DeclareLaunchArgument('tf_rate_hz', default_value=TextSubstitution(text="10.0"))
    odom_topic_name_arg = DeclareLaunchArgument('odom_topic_name', default_value=TextSubstitution(text="odom"))
    ticks_meter_arg = DeclareLaunchArgument('ticks_meter', default_value=TextSubstitution(text="50"))
    base_width_arg = DeclareLaunchArgument('base_width', default_value=TextSubstitution(text="0.245"))
    base_frame_id_arg = DeclareLaunchArgument('base_frame_id', default_value=TextSubstitution(text="base_link"))
    odom_frame_id_arg = DeclareLaunchArgument('odom_frame_id', default_value=TextSubstitution(text="odom"))
    encoder_min_arg = DeclareLaunchArgument('encoder_min', default_value=TextSubstitution(text="0"))
    encoder_max_arg = DeclareLaunchArgument('encoder_max', default_value=TextSubstitution(text="4096"))
    wheel_low_wrap_arg = DeclareLaunchArgument('wheel_low_wrap', default_value=TextSubstitution(text="1228"))
    wheel_high_wrap_arg = DeclareLaunchArgument('wheel_high_wrap', default_value=TextSubstitution(text="2867"))

    # Declare defined launch options 
    ld.add_action(log_level_launch_arg)
    ld.add_action(walker_param_file_arg)
    ld.add_action(use_sim_time_launch_arg)

    ld.add_action(left_wheel_encoder_topic_name_arg)
    ld.add_action(left_wheel_frame_id_arg)
    ld.add_action(left_wheel_preamble_arg)
    
    ld.add_action(right_wheel_encoder_topic_name_arg)
    ld.add_action(right_wheel_frame_id_arg)
    ld.add_action(right_wheel_preamble_arg)

    ld.add_action(left_handle_topic_name_arg)
    ld.add_action(left_handle_frame_id_arg)
    ld.add_action(left_handle_preamble_arg)

    ld.add_action(right_handle_topic_name_arg)
    ld.add_action(right_handle_frame_id_arg)
    ld.add_action(right_handle_preamble_arg)

    ld.add_action(tf_rate_hz_arg)
    ld.add_action(odom_topic_name_arg)
    ld.add_action(ticks_meter_arg)
    ld.add_action(base_width_arg)
    ld.add_action(base_frame_id_arg)
    ld.add_action(odom_frame_id_arg)
    ld.add_action(encoder_min_arg)
    ld.add_action(encoder_max_arg)
    ld.add_action(wheel_low_wrap_arg)
    ld.add_action(wheel_high_wrap_arg)

    # Nodes and launchers  ...........................................................

    # handles ........................................................................
    #   left
    left_handle_node = Node(
        package='walker_arduino',
        executable='usb_conn',
        name='left_handle_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[ {'handle_topic_name': left_handle_topic_name},
                     {'handle_frame_id': left_handle_frame_id},
                     {'data_type': 'float'},
                     {'preamble': left_handle_preamble}
                    ]
    )
    ld.add_action(left_handle_node)
    
    #   right
    right_handle_node = Node(
        package='walker_arduino',
        executable='usb_conn',
        name='right_handle_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[ {'handle_topic_name': right_handle_topic_name},
                     {'handle_frame_id': right_handle_frame_id},
                     {'data_type': 'float'},
                     {'preamble': right_handle_preamble}
                    ]
    )
    ld.add_action(right_handle_node)

    # encoder wheels ..................................................................
    #   left
    left_wheel_node = Node(
        package='walker_arduino',
        executable='usb_conn',
        name='left_wheel_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[ {'wheel_topic_name': left_wheel_encoder_topic_name},
                     {'wheel_frame_id': left_wheel_frame_id},
                     {'data_type': 'int'},
                     {'preamble': left_wheel_preamble}
                    ]
    )
    ld.add_action(left_wheel_node)
    
    #   right
    right_wheel_node = Node(
        package='walker_arduino',
        executable='usb_conn',
        name='right_wheel_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[ {'wheel_topic_name': right_wheel_encoder_topic_name},
                     {'wheel_frame_id': right_wheel_frame_id},
                     {'data_type': 'int'},
                     {'preamble': right_wheel_preamble}
                    ]
    )
    ld.add_action(right_wheel_node)

    # odometry publisher encoder ......................................................
    odometry_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(walker_diff_odom_launch_file),
                                          launch_arguments={'log_level': log_level,
                                          'tf_rate_hz': tf_rate_hz,
                                          'odom_topic_name': odom_topic_name,
                                          'left_wheel_encoder_topic_name': left_wheel_encoder_topic_name,
                                          'right_wheel_encoder_topic_name': right_wheel_encoder_topic_name,
                                          'ticks_meter': ticks_meter,
                                          'base_width': base_width,
                                          'base_frame_id': base_frame_id,
                                          'odom_frame_id': odom_frame_id,
                                          'encoder_min': encoder_min,
                                          'encoder_max': encoder_max,
                                          'wheel_low_wrap': wheel_low_wrap,
                                          'wheel_high_wrap': wheel_high_wrap}.items(),
    )

    ld.add_action(odometry_launch)

    # lidar ...........................................................................
    print("WALKER LIDAR YET TO BE INCLUDED...\n")


    # state publisher
    robot_state_publisher_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(walker_state_publisher_launch_file),
                                          launch_arguments={'use_sim_time': use_sim_time,
                                                            'log_level': log_level}.items(),
    )

    ld.add_action(robot_state_publisher_launch)

    return ld


if __name__ == '__main__':
    generate_launch_description()