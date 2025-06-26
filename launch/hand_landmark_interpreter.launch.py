# *********************************************************************************************************************
# Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
#
# The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
# and/or its licensors ("Renesas") and subject to statutory and contractual protections.
#
# Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
# display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
# purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
# SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
# NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
# OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
# be subject to different terms.
# *********************************************************************************************************************

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directories
    inspire_rh56_pkg_dir = get_package_share_directory('inspire_rh56_dexhand')

    # Default config file path (from inspire_rh56_dexhand package)
    default_config = os.path.join(inspire_rh56_pkg_dir, 'config/inspire_rh56.yaml')

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
