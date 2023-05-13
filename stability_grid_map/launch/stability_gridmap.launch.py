#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from os.path import join


def generate_launch_description():
    # Declare launch configuration variables that can access the launch arguments values
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    ld = LaunchDescription()

    # Declare launch arguments
    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    ld.add_action(use_sim_time_cmd)

    # Stability gridmap node
    stability_gridmap_node = Node(
            package="stability_grid_map",
            executable="simple_demo",
            name="stability_gridmap_node",
            parameters= [
                 {'stability_topic':       'user_stability'},
                 {'stability_map_topic':   '/grid_map'},
                 {'is_verbose':            True },
                 {'maps_frame_id':         'map'},
                 {'maps_size_x':           48.0},
                 {'maps_size_y':           40.0},
                 {'maps_resolution':       0.4},
                 {'maps_origin_x':         0.0},
                 {'maps_origin_y':         0.0},
                 {'update_radius':         1.2},
                 {'initial_map_value':     0.0},
                 {'map_fusion_timer_ms':   500},
                 {'map_publish_timer_ms':  1000},
                 {'use_sim_time':          use_sim_time}
            ],
    )
    ld.add_action(stability_gridmap_node)
    

    return ld 
 

