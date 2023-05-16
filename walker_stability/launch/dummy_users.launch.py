from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

'''
ros2 launch stability_grid_map visualize_stability_gridmap.launch.py 
ros2 launch walker_bringup walker_map_server.launch.py 
ros2 launch stability_grid_map stability_gridmap.launch.py 
ros2 launch walker_stability dummy_users.launch.py 

'''
def generate_launch_description():
    walker_stability_folder = get_package_share_directory('walker_stability')


    ld = LaunchDescription()
    config = join(walker_stability_folder,
        'config',
        'users_conf.yaml'
        )
        
    node_1=Node(
        package = 'walker_stability',
        executable = 'dummy_user.py',
        name = 'dummy_user_1',
        parameters = [config]
    )
    ld.add_action(node_1)

    node_2=Node(
        package = 'walker_stability',
        executable = 'dummy_user.py',
        name = 'dummy_user_2',
        parameters = [config]
    )
    ld.add_action(node_2)

    node_3=Node(
        package = 'walker_stability',
        executable = 'dummy_user.py',
        name = 'dummy_user_3',
        parameters = [config]
    )
    ld.add_action(node_3)    

    node_4=Node(
        package = 'walker_stability',
        executable = 'dummy_user.py',
        name = 'dummy_user_4',
        parameters = [config]
    )
    ld.add_action(node_4)

    node_zero=Node(
        package = 'walker_stability',
        executable = 'dummy_user.py',
        name = 'dummy_user_zero',
        parameters = [config]
    )
    ld.add_action(node_zero)
    
    node_one=Node(
        package = 'walker_stability',
        executable = 'dummy_user.py',
        name = 'dummy_user_one',
        parameters = [config]
    )
    ld.add_action(node_one)
    return ld
