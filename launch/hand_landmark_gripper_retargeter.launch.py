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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gripper_command_topic_arg = DeclareLaunchArgument(
        'gripper_command_topic',
        default_value='hand_gripper_command',
        description='Topic to publish control_msgs/GripperCommand on '
                    '(applied via topic remapping of "gripper_command")'
    )
    gripper_max_width_arg = DeclareLaunchArgument(
        'gripper_max_width',
        default_value='0.06',
        description='Upper bound of the gripper width in metres'
    )
    pinch_open_ratio_arg = DeclareLaunchArgument(
        'pinch_open_ratio',
        default_value='1.2',
        description='pinch/palm ratio mapped to gripper_max_width'
    )
    pinch_close_ratio_arg = DeclareLaunchArgument(
        'pinch_close_ratio',
        default_value='0.15',
        description='pinch/palm ratio mapped to 0.0 m'
    )
    gripper_smooth_factor_arg = DeclareLaunchArgument(
        'gripper_smooth_factor',
        default_value='0.7',
        description='EMA smoothing factor for the published width'
    )

    retargeter_node = Node(
        package='arm_hand_control',
        executable='hand_landmark_gripper_retargeter',
        name='hand_landmark_gripper_retargeter',
        output='screen',
        parameters=[{
            'gripper_max_width': LaunchConfiguration('gripper_max_width'),
            'pinch_open_ratio': LaunchConfiguration('pinch_open_ratio'),
            'pinch_close_ratio': LaunchConfiguration('pinch_close_ratio'),
            'gripper_smooth_factor': LaunchConfiguration('gripper_smooth_factor'),
        }],
        remappings=[
            ('hand_landmarks', '/hand_landmark_estimation/hand_landmarks'),
            ('gripper_command', LaunchConfiguration('gripper_command_topic')),
        ],
    )

    return LaunchDescription([
        gripper_command_topic_arg,
        gripper_max_width_arg,
        pinch_open_ratio_arg,
        pinch_close_ratio_arg,
        gripper_smooth_factor_arg,
        retargeter_node,
    ])
