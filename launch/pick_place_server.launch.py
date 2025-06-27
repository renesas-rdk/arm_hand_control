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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import FrontendLaunchDescriptionSource


def generate_launch_description():
    """
    Launch pick-place action server with Agilex Piper arm controller and visualization.
    Pipeline: pick-place action server → arm controller → foxglove visualization

    Topic flow:
    - Pick-place action server publishes to: /arm/pose_command, /arm/gripper_command
      and subscribes to: /arm/current_pose, /arm/joint_states, /arm/status
    - Arm controller subscribes to: /arm/pose_command, /arm/joint_command, /arm/gripper_command
      and publishes: /arm/joint_states, /arm/gripper_joint_states, /arm/current_pose, /arm/status
    - Foxglove bridge allows visualization of all topics in Foxglove Studio

    Action interface:
    - /pick_place (arm_hand_control/action/PickPlace): Execute pick-and-place operations

    Services (from arm controller):
    - /piper/enable_arm: Enable/disable the arm
    - /piper/home: Move to home position
    - /piper/emergency_stop: Emergency stop
    - /piper/set_motion_mode: Set motion mode (true=Joint, false=Cartesian)

    Note: The arm controller starts with motion_mode=0 (Cartesian) for pick-place operations.
    """

    # Declare launch arguments
    can_interface = LaunchConfiguration('can_interface')

    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can2',
        description='CAN interface to use for the arm controller'
    )

    # Create the pick-place action server node
    pick_place_server_node = Node(
        package='arm_hand_control',
        executable='pick_place_action_server',
        name='pick_place_action_server',
        output='screen',
        parameters=[{
            'position_tolerance': 0.005,      # meters
            'orientation_tolerance': 0.05,    # radians (~2.86 degrees)
            'move_timeout': 2.0,              # seconds
            'gripper_timeout': 1.0,           # seconds
            'gripper_settle_time': 0.5,       # seconds
        }],
        remappings=[
            ('/arm/pose_command', '/arm/pose_command'),
            ('/arm/gripper_command', '/arm/gripper_command'),
            ('/arm/current_pose', '/arm/current_pose'),
            ('/arm/joint_states', '/arm/joint_states'),
            ('/arm/status', '/arm/status'),
            ('/arm/set_high_speed', '/piper/set_high_speed'),
        ]
    )

    # Create arm controller node
    # SUBSCRIBES: /arm/pose_command, /arm/joint_command, /arm/gripper_command
    # PUBLISHES: /arm/joint_states, /arm/current_pose, /arm/status
    arm_config_file = os.path.join(
        get_package_share_directory('agilex_piper_arm'), 'config/agilex_piper.yaml')
    arm_node = Node(
        package='agilex_piper_arm',
        executable='agilex_piper_arm_node',
        name='agilex_piper_arm_node',
        output='screen',
        parameters=[{
            'config_file': arm_config_file,
            'can_interface': can_interface,
            'arm_enabled': True,
            'motion_mode': 0,  # 0=Cartesian mode for pick-place operations
            'listen_only': False
        }],
        remappings=[
            # Input command topics
            ('piper/pose_command', '/arm/pose_command'),
            ('piper/joint_command', '/arm/joint_command'),
            ('piper/gripper_command', '/arm/gripper_command'),

            # Output feedback topics
            ('piper/joint_states', '/arm/joint_states'),
            ('piper/gripper_joint_states', '/arm/gripper_joint_states'),
            ('piper/current_pose', '/arm/current_pose'),
            ('piper/status', '/arm/status')
        ]
    )

    # Foxglove bridge for visualization in Foxglove Studio
    # BRIDGES: All relevant topics for visualization in Foxglove Studio
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    return LaunchDescription([
        # Launch arguments
        declare_can_interface,

        # Nodes
        pick_place_server_node,
        arm_node,
        foxglove_bridge_launch
    ])
