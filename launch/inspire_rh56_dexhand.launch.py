import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_hand_control')
    default_config_file = os.path.join(pkg_share, 'config/hand/inspire_rh56.yaml')

    config_file = LaunchConfiguration('config_file')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the joint configuration YAML file'),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the Inspire RH56 hand'),

        Node(
            package='arm_hand_control',
            executable='inspire_rh56_dexhand',
            name='inspire_rh56_dexhand_node',
            parameters=[
                {'config_file': config_file,
                 'serial_port': serial_port}
            ],
            output='screen'
        )
    ])
