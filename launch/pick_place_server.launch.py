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
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch pick-place action server for Agilex Piper arm.

    Topic Remapping:
    - /arm/pose_command → /agilex_piper_cartesian_motion_controller/target_frame
      Commands for Cartesian end-effector pose
    - /arm/current_pose → /agilex_piper_cartesian_motion_controller/current_pose
      Feedback on current end-effector pose
    - /arm/gripper_command → /gripper_command
      Commands for gripper control
    - /arm/speed → /agilex_piper_gpio_controller/commands
      Dynamic speed control commands

    Action Interface:
    - /pick_place (arm_hand_control/action/PickPlace): Execute pick-and-place operations

    Prerequisites:
    - Agilex Piper arm controller must be running (ros2_control with cartesian_motion_controller)
    - robot_state_publisher should be running for the arm URDF
    - Gripper controller should be available

    Parameters:
    - position_tolerance: Position error threshold (meters)
    - orientation_tolerance: Orientation error threshold (radians)
    - move_timeout: Maximum time for each motion (seconds)
    - gripper_settle_time: Time to wait for gripper to stabilize (seconds)
    - use_current_pose_as_home: If true, uses first received pose as home (recommended)
    - home_position: Home position in Cartesian space if not using current pose (x, y, z in meters)
    - home_orientation: Home orientation if not using current pose as quaternion (x, y, z, w)
    - home_gripper_position: Gripper opening at home (meters)
    """

    # Create the pick-place action server node
    pick_place_server = Node(
        package='arm_hand_control',
        executable='pick_place_action_server',
        name='pick_place_action_server',
        output='screen',
        parameters=[{
            'position_tolerance': 0.005,      # 5mm tolerance
            'orientation_tolerance': 0.05,    # ~2.86 degrees tolerance
            'move_timeout': 2.0,              # 2 seconds timeout
            'gripper_settle_time': 0.5,       # 0.5 second settling time
            # Home position strategy
            'use_current_pose_as_home': True, # Use first received pose as home (recommended)
            # Fallback home position parameters (used only if use_current_pose_as_home is False)
            'home_position.x': 0.15,          # meters
            'home_position.y': 0.0,           # meters
            'home_position.z': 0.22,          # meters
            'home_orientation.x': 0.0,        # quaternion x
            'home_orientation.y': 0.68,       # quaternion y (approx 45° pitch)
            'home_orientation.z': 0.0,        # quaternion z
            'home_orientation.w': 0.74,       # quaternion w
            'home_gripper_position': 0.05,    # meters (open position)
        }],
        remappings=[
            # Remap to actual Agilex Piper controller topics
            ('/arm/pose_command', '/agilex_piper_cartesian_motion_controller/target_frame'),
            ('/arm/gripper_command', '/gripper_command'),
            ('/arm/current_pose', '/agilex_piper_cartesian_motion_controller/current_pose'),
            ('/arm/speed', '/agilex_piper_gpio_controller/commands'),
        ]
    )

    return LaunchDescription([
        pick_place_server
    ])
