import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('auto_aim_exec'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://auto_aim_exec/config/mindvision_camera/camera_info.yaml'
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
                )
            ],
            output='screen',
        )
    serial_node = Node(
            package='auto_aim_exec',
            executable='serial_node',
            output='screen'
        )
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),
        container,
        serial_node
    ])
