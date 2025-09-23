# drone_launch.launch.py

import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # TF publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0.12', '0.03', '0.242', '0', '0.785 ', '0', 'base_link', 'camera_frame'] # updated
        ),
        
        # Image bridge nodes
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='depth_image_bridge',
            output='screen',
            arguments=['/depth_camera']
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='camera_image_bridge',
            output='screen',
            arguments=['/camera']
        ),

        # PX4 command handler node
        Node(
            package='px4_command_handler',
            executable='px4_command_handler',
            name='px4_command_handler',
            output='screen'
        ),

        # Visual feature extraction node
        Node(
            package='visual_feature_extraction',
            executable='visual_feature_extraction',
            name='visual_feature_extraction',
            output='screen'
        ),

        Node(
            package='camera_info_publisher',
            executable='camera_info_publisher',
            name='camera_info_publisher_node',
            output='screen',
        ),

        Node(
            package='feature_2dto3d_transfer',
            executable='feature_2dto3d_transfer',
            name='feature_2dto3d_transfer',
            output='screen',
        ),

        Node(
            package='slam',
            executable='slam',
            name='slam',
            output='screen',
        ),
    ])

