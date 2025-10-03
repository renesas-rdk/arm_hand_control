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

"""
Launch pick-place action server for Agilex Piper arm.

Launch Arguments:
- use_native_cartesian (bool, default='false'):
  Use native Cartesian control (hardware-level) instead of ROS2 controller-based control.
  When true, uses GPIO controller topics for pose commands and feedback.
  When false, uses Cartesian motion controller topics.

Topic Remapping (when use_native_cartesian=false):
- /arm/pose_command → /agilex_piper_cartesian_motion_controller/target_frame
  Commands for Cartesian end-effector pose
- /arm/current_pose → /agilex_piper_cartesian_motion_controller/current_pose
  Feedback on current end-effector pose

Topic Remapping (when use_native_cartesian=true):
- /arm/pose_command → /agilex_piper_gpio_controller/target_pose
  Commands for Cartesian end-effector pose (native control)
- /arm/current_pose → /agilex_piper_gpio_controller/current_pose
  Feedback on current end-effector pose (native control)

Common Topic Remapping:
- /arm/gripper_command → /gripper_command
  Commands for gripper control
- /arm/speed → /agilex_piper_gpio_controller/commands
  Dynamic speed control commands

Action Interface:
- /pick_place (arm_hand_control/action/PickPlace): Execute pick-and-place operations

Prerequisites:
- Agilex Piper arm controller must be running:
    For native control: ros2_control with gpio_controller
    For ROS2 control: ros2_control with cartesian_motion_controller
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

Usage:
  # With ROS2 Cartesian motion controller (default):
  ros2 launch arm_hand_control pick_place_server.launch.py

  # With native Cartesian control (hardware-level):
  ros2 launch arm_hand_control pick_place_server.launch.py use_native_cartesian:=true

  # Example action call:
  ros2 action send_goal /pick_place arm_hand_control/action/PickPlace \
    "{pick_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.05}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}, \
    place_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.1, z: 0.05}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}, \
    approach_height: 0.05, gripper_open_position: 0.03, gripper_closed_position: 0.01, gripper_force: 1.0, return_to_home: true}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function to evaluate launch configurations at runtime."""
    # Get launch configuration
    use_native_cartesian_value = LaunchConfiguration('use_native_cartesian').perform(context)

    # Determine topic remapping based on control mode
    if use_native_cartesian_value.lower() == 'true':
        # Native Cartesian control mode (hardware-level)
        pose_command_topic = '/agilex_piper_gpio_controller/target_pose'
        current_pose_topic = '/agilex_piper_gpio_controller/current_pose'
    else:
        # ROS2 controller-based Cartesian control mode
        pose_command_topic = '/agilex_piper_cartesian_motion_controller/target_frame'
        current_pose_topic = '/agilex_piper_cartesian_motion_controller/current_pose'

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
            # Remap to actual Agilex Piper controller topics (based on control mode)
            ('/arm/pose_command', pose_command_topic),
            ('/arm/current_pose', current_pose_topic),
            ('/arm/gripper_command', '/gripper_command'),
            ('/arm/speed', '/agilex_piper_gpio_controller/commands'),
        ]
    )

    return [pick_place_server]


def generate_launch_description():
    """Generate launch description for pick-place action server."""
    # Declare launch argument
    use_native_cartesian_arg = DeclareLaunchArgument(
        'use_native_cartesian',
        default_value='false',
        description='Use native Cartesian control (true) or ROS2 controller-based control (false)'
    )

    return LaunchDescription([
        use_native_cartesian_arg,
        OpaqueFunction(function=launch_setup)
    ])
