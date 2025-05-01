import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch teleop arm control.
    Pipeline: teleop controller → arm controller

    Topic flow:
    - Teleop controller subscribes to: /arm/pose/cmd_vel, /arm/joint/cmd_vel, /arm/joint_states, /arm/current_pose
      and publishes: /arm/pose_command, /arm/joint_command
    - Arm controller subscribes to: /arm/pose_command, /arm/joint_command
      and publishes: /arm/joint_states, /arm/current_pose, /arm/status

    Note: Set parameter '/agilex_piper_arm.motion_mode' to 0 for cartesian control and 1 for joint control.
    """
    # Get package share directory
    pkg_dir = get_package_share_directory('arm_hand_control')

    # Set up launch arguments
    arm_config = LaunchConfiguration('arm_config_file')
    can_interface = LaunchConfiguration('can_interface')
    arm_enabled = LaunchConfiguration('arm_enabled')
    motion_mode = LaunchConfiguration('motion_mode')
    listen_only = LaunchConfiguration('listen_only')

    # Declare launch arguments
    declare_arm_config = DeclareLaunchArgument(
        'arm_config_file',
        default_value=TextSubstitution(text=os.path.join(pkg_dir, 'config/arm/agilex_piper.yaml')),
        description='Path to the arm configuration file'
    )

    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use for the arm controller'
    )

    declare_arm_enabled = DeclareLaunchArgument(
        'arm_enabled',
        default_value='true',
        description='Enable the arm on startup'
    )

    declare_motion_mode = DeclareLaunchArgument(
        'motion_mode',
        default_value='1',
        description='Control mode: 0=Cartesian, 1=Joint'
    )

    declare_listen_only = DeclareLaunchArgument(
        'listen_only',
        default_value='false',
        description='If true, commands will be received but not executed'
    )

    # 1. Create teleop controller node
    # SUBSCRIBES: /arm/pose/cmd_vel, /arm/joint/cmd_vel, /arm/joint_states, /arm/current_pose
    # PUBLISHES: /arm/pose_command, /arm/joint_command
    teleop_node = Node(
        package='arm_hand_control',
        executable='teleop_twist_controller',
        name='teleop_twist_controller',
        output='screen',
        parameters=[{
            'linear_scale': 0.01,
            'angular_scale': 0.01,
            'joint_vel_scale': 0.01
        }],
        remappings=[
            # Input topics
            ('pose/cmd_vel', '/arm/pose/cmd_vel'),      # Pose twist commands
            ('joint/cmd_vel', '/arm/joint/cmd_vel'),    # Joint twist commands
            ('joint_states', '/arm/joint_states'),      # Joint state feedback
            ('current_pose', '/arm/current_pose'),      # Current end effector pose

            # Output topics
            ('pose_command', '/arm/pose_command'),      # Pose commands
            ('joint_command', '/arm/joint_command'),    # Joint commands
        ]
    )

    # 2. Create arm controller node
    # SUBSCRIBES: /arm/pose_command, /arm/joint_command
    # PUBLISHES: /arm/joint_states, /arm/current_pose, /arm/status
    arm_node = Node(
        package='arm_hand_control',
        executable='agilex_piper_arm',
        name='agilex_piper_arm',
        output='screen',
        parameters=[{
            'config_file': arm_config,
            'can_interface': can_interface,
            'arm_enabled': arm_enabled,
            'motion_mode': motion_mode,
            'listen_only': listen_only
        }],
        remappings=[
            # Input command topics
            ('piper/pose_command', '/arm/pose_command'),      # Pose commands
            ('piper/joint_command', '/arm/joint_command'),    # Joint commands

            # Output feedback topics
            ('piper/joint_states', '/arm/joint_states'),      # Joint state
            ('piper/current_pose', '/arm/current_pose'),      # Current end effector pose
            ('piper/status', '/arm/status')                   # Arm status
        ]
    )

    # Return all nodes in execution order
    return LaunchDescription([
        # Launch arguments
        declare_arm_config,
        declare_can_interface,
        declare_arm_enabled,
        declare_motion_mode,
        declare_listen_only,

        # Nodes
        teleop_node,                  # Teleop controller
        arm_node                      # Arm controller
    ])
