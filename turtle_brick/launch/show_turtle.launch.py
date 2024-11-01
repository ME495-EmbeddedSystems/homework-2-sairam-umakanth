from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution 
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

########### Begin Citation [1] ##################
def generate_launch_description():
    # Declare the 'use_jsp' argument
    use_jsp_arg = DeclareLaunchArgument(
        'use_jsp',
        default_value='gui',
        description="Controls the joint state publisher: 'gui' for joint_state_publisher_gui, 'jsp' for joint_state_publisher, 'none' for no publisher."
    )

    # Path to turtle_brick package share
    turtle_brick_share = FindPackageShare('turtle_brick')

    # Conditional nodes based on use_jsp argument
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_jsp').perform() == 'gui')
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_jsp').perform() == 'jsp')
    )

############# End Citation [1] ################
    # Robot state publisher node with xacro file for robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                PathJoinSubstitution([FindPackageShare('xacro'), 'xacro']),
                ' ',
                PathJoinSubstitution([turtle_brick_share, 'turtle.urdf.xacro'])
            ])
        }]
    )

    # RViz node with specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([turtle_brick_share, 'view_robot.rviz'])]
    )

    return LaunchDescription([
        use_jsp_arg,
        jsp_gui_node,
        jsp_node,
        robot_state_publisher_node,
        rviz_node,
    ])
