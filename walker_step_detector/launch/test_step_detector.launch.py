#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from os import getenv
from os.path import join

step_detector_path = get_package_share_directory('walker_step_detector')
partial_loads_path = get_package_share_directory('walker_loads')
rosbag_uri = join( getenv("HOME") , "bagsFolder", "ida_vuelta", "20220728-084555-bag")
forest_file_path = join( step_detector_path, "config", "trained_step_detector_res_0.33.yaml")
rviz2_config_path = join( step_detector_path, "config", "test_step_detector.rviz") 
handle_calibration_file = join(partial_loads_path, "config", "params.yaml") 
laser_segmentation_path = get_package_share_directory('laser_segmentation')
laser_segmentation_cfg_file = join( step_detector_path, "config", "laser_segmentation.yml")


def generate_launch_description():

    ld = LaunchDescription()

    # Launching Rosbag node
    rosbag_cmd_str=['ros2', 'bag', 'play', '-s', 'sqlite3', rosbag_uri, '-r', '0.1', '--topics', 
        '/handle_height',
        '/left_handle',
        '/left_wheel',
        '/right_handle',
        '/right_wheel',
        '/scan_filtered',
        '/sensor/bwt901cl/Angle',
        '/sensor/bwt901cl/Imu',
        '/sensor/bwt901cl/MagneticField',
        '/sensor/bwt901cl/Temperature',
        '/tf',
        '/tf_static',
        '/user_desc'
    ]
    print(' '.join(rosbag_cmd_str))


    rosbag_cmd = ExecuteProcess(
            cmd=rosbag_cmd_str,
            output='screen'
    )
    #ld.add_action(rosbag_cmd)

    # filter laser data to get only legs area
    config = join(get_package_share_directory('walker_laser_filter'),'config','inverted_box_filter.yaml')
    laser_filter_node = Node(
            package="walker_laser_filter",
            executable="box_filter",
            name='step_laser_filter',
            parameters=[config]
    )
    ld.add_action(laser_filter_node)

    # Launching detect_leg_steps node
    detect_steps_node = Node(
            package="walker_step_detector",
            executable="detect_steps",
            name="detect_steps",
            parameters= [
                {"scan_topic" : "/scan_feet"}, 
                {"forest_file" : forest_file_path},
                {"detected_steps_topic_name" : "/detected_step"}, 
                {"kalman_model_d0":                0.001},
                {"kalman_model_a0":                0.001},
                {"kalman_model_f0":                0.001},
                {"kalman_model_p0":                0.001},
                {"detection_threshold": 0.01},
                {"cluster_dist_euclid": 0.13},
                {"max_detect_distance": 1.25}, # meters, in laser frame
                {"max_detected_clusters": 2},
                {"min_points_per_cluster":  3},
                {"publish_clusters": True}
            ]
    )

    detect_steps_node = Node(
            package="walker_step_detector",
            executable="km_detect_steps",
            name="km_detect_steps",
            parameters= [
                {"scan_topic" : "/scan_feet"}, 
                {"detected_steps_topic_name" : "/detected_step"}, 
                {"kalman_model_d0":                0.001},
                {"kalman_model_a0":                0.001},
                {"kalman_model_f0":                0.001},
                {"kalman_model_p0":                0.001},
                {"kalman_enabled":  False},
                {"fit_ellipse":  False},
                {"is_debug": True}              
            ]
    )
    ld.add_action(detect_steps_node)


    # loads on legs
    partial_load_legs_node = Node(
        package="walker_loads",
        executable="partial_loads",
        name="partial_load_legs",
        parameters= [
                    {'handle_calibration_file': handle_calibration_file},
                    {'loads_topic_name': '/loads'},
                    {'loads_topic_name': '/loads'},
                    {'left_handle_topic_name': '/left_handle'},
                    {'right_handle_topic_name': '/right_handle'},
                    {'left_steps_topic_name': '/left_step'},
                    {'right_steps_topic_name': '/right_step'},
                    {'user_desc_topic_name': '/user_desc'},
                    {'period': 0.1},
                    {'speed_delta': 0.05}
                    ],
        on_exit=Shutdown()
    )
    #ld.add_action(partial_load_legs_node)

    # Visualization        
    rviz_cmd = ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2','-d', rviz2_config_path],
            output='screen'
    )
    ld.add_action(rviz_cmd) 

    # View steps node
    plot_leg_clusters_node = Node(
            package="walker_step_detector",
            executable="plot_steps",
            name="plot_step_legs",
            parameters= [
                {"marker_display_lifetime": 0.0},
                {"marker_size": 0.08},
                {"steps_topic_name" : "/detected_step"},
                {"speed_dead_zone": 0.1}
            ]
    )
    ld.add_action(plot_leg_clusters_node)
    
    if False:


        # View forces on handles     
        plot_handle_forces_node = Node(
            package="walker_plot",
            executable="plot_forces",
            name="plot_handle_forces"
        )
        ld.add_action(plot_handle_forces_node)

        # View loads on legs
        plot_load_legs_node = Node(
            package="walker_loads",
            executable="plot_loads",
            name="plot_load_legs",
            on_exit=Shutdown()
        )
        ld.add_action(plot_load_legs_node)

    return ld 
 
