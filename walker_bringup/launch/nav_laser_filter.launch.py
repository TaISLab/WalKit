from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    declare_params_file_cmd = DeclareLaunchArgument('params_file',
        default_value=PathJoinSubstitution([get_package_share_directory("walker_bringup"),
                                                        "config", "nav_laser_filter.yaml"]),
        description='Full path to the ROS2 parameters file with node configuration'
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
          params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='nav_filter_chain',
        remappings=[ ("scan", "scan_filtered"), ("scan_filtered", "scan_nav") ],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld