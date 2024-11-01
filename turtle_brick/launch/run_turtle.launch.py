from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

########## Begin Citation [1] #################
def generate_launch_description():
    # Define the path to the package share
    turtle_brick_share = FindPackageShare('turtle_brick').find('turtle_brick')

    # Start the turtlesim node with holonomic set to true
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        parameters=[{'holonomic': True}]
    )

    # Start the turtle_robot node with parameters loaded from a YAML file
    turtle_robot_node = Node(
        package='turtle_brick',
        executable='turtle_robot',
        name='turtle_robot',
        parameters=[os.path.join(turtle_brick_share, 'turtle.yaml')]
    )

    # Include the show_turtle launch file
    show_turtle_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtle_brick_share, 'show_turtle.launch.py'))
    )

    return LaunchDescription([
        turtlesim_node,
        turtle_robot_node,
        show_turtle_include
    ])
########### End Citation [1] ##############