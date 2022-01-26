#!/usr/bin/python3


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Visualize steps:
    visualize_steps_node = Node(
            package="walker_step_detector",
            executable="plot_steps",
            name="plot_steps",
            parameters= [
                {"steps_topic_name" : "/detected_step"},
                {"marker_display_lifetime" : 0.1},
                {"speed_dead_zone" : 0.05}
            ]
    )    
        
    ld.add_action(visualize_steps_node)

    return ld 
 
