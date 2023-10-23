from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# TODO: RENAME THIS FILE TO "launch_simulation.py"
# TODO: Make another launch file called "launch_car.py"

def generate_launch_description():
    
    # Initialize a new launch description.
    ld = LaunchDescription()
    
    # Get path to the params.yaml file.
    simulation_params = Path(get_package_share_directory("pure_pursuit"), "config", "simulation_params.yaml")

    # Create new actions to spin up nodes for the pure pursuit node and path
    # file publisher node.
    pure_pursuit_node = Node(
        namespace="ego_racecar",
        package="pure_pursuit",
        executable="pure_pursuit_node.py",
        parameters=[simulation_params],
    )
    pose_publisher_node = Node(
        namespace="ego_racecar",
        package="pure_pursuit",
        executable="pose_publisher_node.py",
        name="car_pose_publisher",
        parameters=[simulation_params]
    )
    path_file_publisher_node = Node(
        namespace="ego_racecar",
        package="pure_pursuit",
        executable="path_publisher_node.py",
        name="path_publisher",
        parameters=[simulation_params],
        remappings="" # TODO: PLACE WHATEVER TOPIC REMAPPING IS NEEDED HERE FOR SIMULATION. REMAP ODOM, ANY OTHERS NEEDED.
    )
    # Add the launch_ros "Node" actions we created.
    ld.add_action(pure_pursuit_node)
    ld.add_action(pose_publisher_node)
    # ld.add_action(path_file_publisher_node)

    # Return the newly created launch description.
    return ld