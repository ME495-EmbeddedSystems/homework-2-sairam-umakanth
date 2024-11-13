from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

########### Begin Citation [1] #####################
def generate_launch_description():
    # Paths to package shares and files
    # not declarative stykle, these variable definitions are unnecessary 
    turtle_brick_share = FindPackageShare('turtle_brick')
    run_turtle_launch = PathJoinSubstitution([turtle_brick_share, 'run_turtle.launch.py'])
    turtle_config = PathJoinSubstitution([turtle_brick_share, 'turtle.yaml'])
    rviz_config = PathJoinSubstitution([turtle_brick_share, 'view_robot.rviz'])

    # Include the run_turtle XML launch file
    run_turtle_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(run_turtle_launch)
    )

    # Arena node
    arena_node = Node(
        package='turtle_brick',
        executable='arena',
        output='screen',
        parameters=[turtle_config]
    )

    # Catcher node
    catcher_node = Node(
        package='turtle_brick',
        executable='catcher',
        name='catcher',
        parameters=[turtle_config]
    )

    # RViz node with specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )
############ End Citation [1] ################

    return LaunchDescription([
        run_turtle_launch_include,
        arena_node,
        catcher_node,
        rviz_node,
    ])
