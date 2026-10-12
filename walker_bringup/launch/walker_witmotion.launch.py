
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        
        # IMU package
        Node(
            package='witmotion_ros', executable='witmotion_ros_node', output='screen', 
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("walker_bringup"),
                    "config", "wt901.yaml",
                ])],
            arguments=['--ros-args', '--log-level', 'warn']
            ),

        # No orientation filter on purpose: the odometry chain (walker_laser_imu_odom.launch.py) uses only the
        # RAW gyro z, and imu_complementary_filter with do_bias_estimation corrupted it when the walker moved
        # (see walker_ekf.yaml). Nothing in the workspace consumes /bwt901cl/imu_filtered; if orientation is
        # needed later, run imu_complementary_filter / imu_filter_madgwick from the consumer's own launch.
    ])
