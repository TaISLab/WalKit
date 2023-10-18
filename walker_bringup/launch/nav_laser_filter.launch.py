from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name='nav_laser_filter',
            # this is necessary otherwise renaming and remapping wont work correctly
            namespace='nav',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("walker_bringup"),
                    "config", "nav_laser_filter.yaml",
                ])],
            remappings=[ ("scan", "/scan_filtered"), 
                ("scan_filtered", "/scan_nav") ]
        )
    ])