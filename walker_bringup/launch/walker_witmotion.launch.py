
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


        # Compute quaternion of the IMU
        # Node(
        #     package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        #     parameters=[{'use_mag': True, 
        #                  'world_frame':'enu',
        #                  'fixed_frame': 'witmotion_still',
        #                  'reverse_tf': False, 
        #                  'publish_tf': False}],
        #     remappings=[('imu/data_raw', '/bwt901cl/imu'),
        #                 ('imu/data', '/bwt901cl/imu_filtered'),
        #                 ('imu/mag', '/bwt901cl/magnetometer')]),

        Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                ],
                remappings=[('imu/data_raw', '/bwt901cl/imu'),
                    ('imu/data', '/bwt901cl/imu_filtered'),
                    ('imu/mag', '/bwt901cl/magnetometer')]
            )



    ])
