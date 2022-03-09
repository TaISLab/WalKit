#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

step_detector_path = get_package_share_directory('walker_step_detector')
partial_loads_path = get_package_share_directory('walker_loads')
rosbag_uri = os.path.join( os.getenv("HOME") , "bagsFolder", "20211214-142719-bag")
forest_file_path = os.path.join( step_detector_path, "config", "trained_step_detector_res_0.33.yaml")
rviz2_config_path = os.path.join( step_detector_path, "config", "demo_stationary_simple_environment.rviz") 
handle_calibration_file = os.path.join(partial_loads_path, "config", "params.yaml") 

def generate_launch_description():

    ld = LaunchDescription()

    # Launching Rosbag node
    rosbag_cmd = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-s', 'sqlite3', rosbag_uri],
            output='screen'
    )

    rviz_cmd = launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2','-d', rviz2_config_path],
            output='screen'
    )

    # filter laser data to get only legs area
    laser_filter2_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("walker_step_detector"),
                "config", "laser_box_filter.yaml",
            ])],
        remappings=[ ("scan", "scan_filtered"),
                     ("scan_filtered", "scan_filtered2")]
    )

    # Launching detect_leg_clusters node
    detect_leg_clusters_node = Node(
            package="walker_step_detector",
            executable="detect_steps",
            name="detect_steps",
            parameters= [
                {"scan_topic" : "/scan_filtered2"},
                {"fixed_frame" : "laser"},
                {"forest_file" : forest_file_path},
                {"detection_threshold": 0.01},
                {"cluster_dist_euclid": 0.13},
                {"min_points_per_cluster":  3},
                {"detect_distance_frame_id": "base_link"},
                {"max_detect_distance": 0.45},
                {"use_scan_header_stamp_for_tfs": False},
                {"max_detected_clusters": 2}
            ]
    )

    # View steps node
    plot_leg_clusters_node = Node(
            package="walker_step_detector",
            executable="plot_steps",
            name="plot_step_legs",
            parameters= [
                {"marker_display_lifetime": 0.2},
                {"steps_topic_name" : "/detected_step"},
                {"speed_dead_zone": 0.1}
            ]
    )

    # View forces on handles     
    plot_handle_forces_node = Node(
        package="walker_plot",
        executable="plot_forces",
        name="plot_handle_forces"
    )

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
        on_exit=launch.actions.Shutdown()
    )

    # View loads on legs
    plot_load_legs_node = Node(
        package="walker_loads",
        executable="plot_loads",
        name="plot_load_legs",
        on_exit=launch.actions.Shutdown()
    )

    ld.add_action(partial_load_legs_node)
    ld.add_action(plot_load_legs_node)
    ld.add_action(plot_handle_forces_node)
    ld.add_action(laser_filter2_node)
    ld.add_action(rosbag_cmd)
    ld.add_action(detect_leg_clusters_node)
    #ld.add_action(plot_leg_clusters_node)
    ld.add_action(rviz_cmd) 
    return ld 
 
