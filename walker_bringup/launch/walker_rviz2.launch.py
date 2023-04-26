import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory("walker_bringup"),
                                   'config', 'demo_view.rviz'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    rviz_cmd = ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2','-d', params_file],
            output='screen'
    ) 

    rviz_cfg = ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/rviz', 'use_sim_time', use_sim_time],
            output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(rviz_cfg)

    return ld
