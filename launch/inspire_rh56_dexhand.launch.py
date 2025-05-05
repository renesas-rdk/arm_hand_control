import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('arm_hand_control')

    # Default config file path (relative to package)
    default_config = os.path.join('config', 'hand/inspire_rh56.yaml')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to config file for hand parameters'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Inspire RH56 hand'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )

    # Create the node
    inspire_rh56_dexhand_node = Node(
        package='arm_hand_control',
        executable='inspire_rh56_dexhand',
        name='inspire_rh56_dexhand',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'command_threshold': 50
        }]
    )

    # Return the launch description
    return LaunchDescription([
        config_arg,
        serial_port_arg,
        baudrate_arg,
        inspire_rh56_dexhand_node
    ])
