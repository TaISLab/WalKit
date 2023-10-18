#!/usr/bin/python3
 
from launch import LaunchDescription 

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

'''
Should be used after realsense output, but seems too cumbersome for the rpi and there is little distortion

'''
    
def generate_launch_description():    
   
    ld = LaunchDescription()

    composable_nodes = [
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer_node',
                namespace='camera',              
                remappings=[
                    ('image_raw', 'color/image_raw')
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_mono_node',
                namespace='camera',
                # Remap subscribers and publishers
                remappings=[
                    ('image', 'image_mono'),
                    ('camera_info', 'color/camera_info'),
                    ('image_rect', 'image_rect')
                ],
                parameters=[
                    {'queue_size': 15}
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                namespace='camera',
                # Remap subscribers and publishers
                remappings=[
                    ('image', 'image_color'),
                    ('camera_info', 'color/camera_info'),
                    ('image_rect', 'image_rect_color')
                ],
                parameters=[
                    {'queue_size': 15}
                ],
            )
    ]

    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )
    ld.add_action(arg_container)

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )
    ld.add_action(image_processing_container)

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )
    ld.add_action(load_composable_nodes)

    return ld
