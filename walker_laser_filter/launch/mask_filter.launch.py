from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="walker_laser_filter",
            executable="mask_filter",
            name='laser_filter',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("walker_laser_filter"),
                    "config", "mask_filter.yaml",
                ])]
        )
    ])
