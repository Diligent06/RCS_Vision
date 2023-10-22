import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
            name='test_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='auto_aim_exec',
                    plugin='mindvision_camera::MVCameraNode',
                    name='camera'
                ),
                ComposableNode(
                    package='auto_aim_exec',
                    plugin='target_detect::TargetDetectNode',
                    name='detect'
                ),
                ComposableNode(
                    package='auto_aim_exec',
                    plugin='post_process::PostProcessNode',
                    name='post_process'
                ),
                ComposableNode(
                    package='auto_aim_exec', 
                    plugin='serial::SerialNode',
                    name='serial'
                )
            ],
            output='screen',
        )
    return LaunchDescription([
        container
    ])
