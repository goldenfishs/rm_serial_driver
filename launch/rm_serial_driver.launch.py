from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rm_serial_driver')
    param_file = os.path.join(pkg_share, 'config', 'serial_config.yaml')

    container = ComposableNodeContainer(
        name='rm_serial_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # 多线程容器
        composable_node_descriptions=[
            ComposableNode(
                package='rm_serial_driver',
                plugin='rm_serial_driver::RMSerialDriver',
                name='rm_serial_driver',
                parameters=[param_file]
            )
        ],
        output='screen'
    )
    return LaunchDescription([container])