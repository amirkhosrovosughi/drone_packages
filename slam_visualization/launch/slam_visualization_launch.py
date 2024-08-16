from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('slam_visualization'),
        'config',
        'visualization_config.rviz'
    ])

    return LaunchDescription([
        Node(
            package='slam_visualization',
            executable='slam_visualization',
            name='slam_visualization',
            output='screen'
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'map']
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])
