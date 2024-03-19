from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = join(get_package_share_directory('walker_laser_filter'),'config','box_filter.yaml')
    print(config)
    node = Node(
            package="walker_laser_filter",
            executable="box_filter",
            name='nav_laser_filter',
            parameters=[config]
        )
    
    ld.add_action(node)
    return ld
