from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # File paths
    turtle_yaml = PathJoinSubstitution([FindPackageShare("turtle_brick"), "turtle.yaml"])
    turtle_urdf_xacro = PathJoinSubstitution([FindPackageShare("turtle_brick"), "turtle.urdf.xacro"])
    rviz_config = PathJoinSubstitution([FindPackageShare("turtle_brick"), "view_robot.rviz"])

    return LaunchDescription([
        
        # Turtlesim Node
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node",
            parameters=[{"holonomic": True}]
        ),

        # Turtle Robot Node
        Node(
            package="turtle_brick",
            executable="turtle_robot",
            name="turtle_robot",
        ),
        

        # Robot State Publisher Node
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", turtle_urdf_xacro])
            }]
        ),

        # Joint State Publisher GUI Node
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),

        # RViz2 Node
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config]
        ),
    ])
