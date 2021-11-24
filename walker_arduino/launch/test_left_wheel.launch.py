

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Default filenames and where to find them
    # sim_package_dir = get_package_share_directory('walker_simulation')
    # xacro_file = os.path.join(sim_package_dir, 'urdf', 'walker_model.xacro')

    ld = LaunchDescription()

    # Set env vars
    # to print messages to stdout immediately
    log_config = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(log_config)
    
    # Create the launch configuration variables: 
    log_level = LaunchConfiguration('log_level')
    handle_topic_name = LaunchConfiguration('handle_topic_name')
    handle_frame_id = LaunchConfiguration('handle_frame_id')
    serial_preamble = LaunchConfiguration('serial_preamble')
    port_basename = LaunchConfiguration('port_basename')
    data_type = LaunchConfiguration('data_type')
        
    # Map these variables to Arguments: can be set from the command line or a default will be used    
    log_level_arg = DeclareLaunchArgument( 'log_level', default_value=TextSubstitution(text='INFO') )    
    serial_preamble_arg = DeclareLaunchArgument('serial_preamble', default_value=TextSubstitution(text= 'LW' ))
    handle_topic_name_arg = DeclareLaunchArgument('handle_topic_name', default_value=TextSubstitution(text= 'left_wheel' ))
    handle_frame_id_arg = DeclareLaunchArgument('handle_frame_id', default_value=TextSubstitution(text= 'left_wheel_id' ))
    port_basename_arg = DeclareLaunchArgument('port_basename', default_value=TextSubstitution(text= '/dev/ttyUSB' ))
    data_type_arg = DeclareLaunchArgument('data_type', default_value=TextSubstitution(text= 'int' ))
    
    # Declare defined launch options 
    ld.add_action(log_level_arg)
    ld.add_action(serial_preamble_arg)
    ld.add_action(handle_topic_name_arg)
    ld.add_action(handle_frame_id_arg)
    ld.add_action(port_basename_arg)
    ld.add_action(data_type_arg)
 
    # Map arguments to parameters
    robot_state_publisher_node = Node(
        package='walker_arduino',
        executable='usb_conn',
        name='left_wheel_conn',
        output='screen', 
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
                    {'serial_preamble': serial_preamble},
                    {'handle_topic_name': handle_topic_name},
                    {'handle_frame_id': handle_frame_id},
                    {'port_basename': port_basename},
                    {'data_type': data_type}                    
                    ]
    )
    ld.add_action(robot_state_publisher_node)

    visual_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='left_wheel_gui',
        output='screen', 
        arguments=['--ros-args', '--log-level', log_level]
    )
    #ld.add_action(visual_node)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()
