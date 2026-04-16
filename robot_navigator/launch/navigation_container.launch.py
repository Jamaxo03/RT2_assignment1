import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='navigation_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            prefix=['xterm -e'],
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_navigator',
                    plugin='robot_navigator::NavigationServer',
                    name='navigation_server',
                    parameters=[{'use_sim_time': True}] 
                    ),
                    
                ComposableNode(
                    package='robot_navigator',
                    plugin='robot_navigator::NavigationClient',
                    name='navigation_client',
                    parameters=[{'use_sim_time': True}] 
                    )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])