from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Initialize a new launch description.
    ld = LaunchDescription()
    
    # Get path to the params.yaml file.
    params = Path(get_package_share_directory("pure_pursuit"), "config", "params.yaml")

    # Create new actions to spin up nodes for the pure pursuit node and path
    # file publisher node.
    # pure_pursuit_node = Node(
    #     executable="pure_pursuit"
    # )
    path_file_publisher_node = Node(
        package="pure_pursuit",
        executable="path_publisher.py",
        parameters=[params],
        remappings="" # TODO: PLACE WHATEVER TOPIC REMAPPING IS NEEDED HERE FOR SIMULATION. REMAP ODOM, ANY OTHERS NEEDED.
    )
    # Add the launch_ros "Node" actions we created.
    # ld.add_action(pure_pursuit_node)
    ld.add_action(path_file_publisher_node)

    # Return the newly created launch description.
    return ld