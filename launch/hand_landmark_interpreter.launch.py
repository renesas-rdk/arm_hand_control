import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('arm_hand_control')
    default_config = os.path.join(pkg_dir, 'config/hand/inspire_rh56.yaml')

    # Declare the config file path as a launch argument
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to config file for hand parameters'
    )

    # Create the hand landmark interpreter node
    hand_landmark_interpreter_node = Node(
        package='arm_hand_control',
        executable='hand_landmark_interpreter',
        name='hand_landmark_interpreter',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            {'curl_smooth_factor': 0.5}
        ],
        remappings=[
            ('hand_landmarks', '/hand_landmark_estimation/hand_landmarks')
        ]
    )

    # Return the launch description
    return LaunchDescription([
        config_arg,
        hand_landmark_interpreter_node
    ])
