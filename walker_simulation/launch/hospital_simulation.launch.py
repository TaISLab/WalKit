

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os 

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Default filenames and where to find them
    bringup_dir = get_package_share_directory('walker_bringup')
    sim_package_dir = get_package_share_directory('walker_simulation')
    world_file_package_dir = get_package_share_directory('aws_robomaker_hospital_world')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    walker_state_publisher_launch_file = os.path.join( bringup_dir, 'launch', 'walker_state_publisher.launch.py')

    # Turtlebot3 burger world... without the robot
    world = os.path.join(sim_package_dir, 'worlds', 'test_world.model')
    
    # World from aws
    world_file_name = "hospital.world"
    world = os.path.join(world_file_package_dir, 'worlds', world_file_name)
    
    # custom
    world = os.path.join(sim_package_dir, 'worlds', 'test_hospital.model')
    
    ld = LaunchDescription()

    # Set env vars
    # to print messages to stdout immediately
    log_config = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(log_config)
    
    # to find gazebo models
    gazebo_models_config = SetEnvironmentVariable('GAZEBO_MODEL_PATH', world_file_package_dir + '/models:' + world_file_package_dir + '/fuel_models')
    ld.add_action(gazebo_models_config)
    # solves an odd bug that makes gazebo crash
    gazebo_bug_config = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')
    ld.add_action(gazebo_bug_config)


    # Create the launch configuration variables: 
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_pose_x = LaunchConfiguration('robot_pose_x')
    robot_pose_y = LaunchConfiguration('robot_pose_y')
    robot_pose_z = LaunchConfiguration('robot_pose_z')
    robot_pose_roll = LaunchConfiguration('robot_pose_roll')
    robot_pose_pitch = LaunchConfiguration('robot_pose_pitch')
    robot_pose_yaw = LaunchConfiguration('robot_pose_yaw')

    # Map these variables to Arguments: can be set from the command line or a default will be used    
    log_level_launch_arg = DeclareLaunchArgument( "log_level", default_value=TextSubstitution(text="DEBUG") )
    use_sim_time_launch_arg = DeclareLaunchArgument("use_sim_time",default_value='false', description='Use simulation (Gazebo) clock if true')
    robot_pose_x_arg = DeclareLaunchArgument( name='robot_pose_x', default_value='-4.4' )
    robot_pose_y_arg = DeclareLaunchArgument( name='robot_pose_y', default_value='1.74' )
    robot_pose_z_arg = DeclareLaunchArgument( name='robot_pose_z', default_value='0.0' )
    robot_pose_roll_arg = DeclareLaunchArgument( name='robot_pose_roll', default_value='0.0' )
    robot_pose_pitch_arg = DeclareLaunchArgument( name='robot_pose_pitch', default_value='0.0' )
    robot_pose_yaw_arg = DeclareLaunchArgument( name='robot_pose_yaw', default_value='1.57' )

    # Declare defined launch options 
    ld.add_action(log_level_launch_arg)
    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(robot_pose_x_arg)
    ld.add_action(robot_pose_y_arg)
    ld.add_action(robot_pose_z_arg)
    ld.add_action(robot_pose_roll_arg)
    ld.add_action(robot_pose_pitch_arg)
    ld.add_action(robot_pose_yaw_arg)

    # Spawning the walker  ...........................................................
    # It needs two elements:state publisher and gazebo entity
    
    # 1.- Create state publisher
    robot_state_publisher_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(walker_state_publisher_launch_file),
                                          launch_arguments={'use_sim_time': use_sim_time,
                                                            'log_level': log_level}.items(),
    )

    ld.add_action(robot_state_publisher_launch)

    # 3.- And spawn it.
    robot_spawn_model_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            parameters=[{'/use_sim_time': use_sim_time }],
            arguments=[
                '-entity', 'robot', 
                '-topic', '/walker_description',
                #'-file', urdf_path,
                '-x', robot_pose_x, 
                '-y', robot_pose_y, 
                '-z', robot_pose_z, 
                '-R', robot_pose_roll, 
                '-P', robot_pose_pitch, 
                '-Y', robot_pose_yaw
            ],
            output='screen'
        )
    ld.add_action(robot_spawn_model_node)

    # gazebo: We do this way to make it verbose ....
    # 1. Server
    gz_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
            launch_arguments={'verbose': 'true','world': world}.items()
        )
    ld.add_action(gz_server_launch)
    
    # 2. Client
    gz_client_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py'))
        )
    ld.add_action(gz_client_launch)

    return ld


if __name__ == '__main__':
    generate_launch_description()
