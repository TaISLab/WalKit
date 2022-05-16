from os.path import join
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ekf_param_dir = LaunchConfiguration(
        'ekf_param_dir',
        default=join(
            get_package_share_directory('walker_bringup'),
            'config',
            'laser_scan_matcher.yaml'))

    return LaunchDescription([
        Node(
            package='ros2_laser_scan_matcher', 
            executable='laser_scan_matcher', 
            name='laser_scan_matcher_node',
            parameters=[ekf_param_dir],
	        output='screen'
            #remappings=[('/scan', '/scan_filtered')]
           )
])
