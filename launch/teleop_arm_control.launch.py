import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('arm_hand_control')

    # Set up launch arguments
    arm_config = LaunchConfiguration('arm_config_file')
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    joint_vel_scale = LaunchConfiguration('joint_vel_scale')
    can_interface = LaunchConfiguration('can_interface')

    # Declare launch arguments
    declare_arm_config = DeclareLaunchArgument(
        'arm_config_file',
        default_value=TextSubstitution(text=os.path.join(pkg_dir, 'config/arm/agilex_piper.yaml')),
        description='Path to the arm configuration file'
    )

    declare_linear_scale = DeclareLaunchArgument(
        'linear_scale',
        default_value='0.01',
        description='Linear scale factor for cartesian position control (m per unit twist)'
    )

    declare_angular_scale = DeclareLaunchArgument(
        'angular_scale',
        default_value='0.01',
        description='Angular scale factor for cartesian orientation control (rad per unit twist)'
    )

    declare_joint_vel_scale = DeclareLaunchArgument(
        'joint_vel_scale',
        default_value='0.01',
        description='Joint velocity scale factor for joint position control (rad per unit twist)'
    )

    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use for the arm controller'
    )

    # Create teleop controller node
    teleop_node = Node(
        package='arm_hand_control',
        executable='teleop_twist_controller',
        name='teleop_twist_controller',
        output='screen',
        parameters=[{
            'linear_scale': linear_scale,
            'angular_scale': angular_scale,
            'joint_vel_scale': joint_vel_scale
        }],
        remappings=[
            # Input topics
            ('pose/cmd_vel', '/arm/pose/cmd_vel'),      # Pose twist commands
            ('joint/cmd_vel', '/arm/joint/cmd_vel'),    # Joint twist commands
            ('joint_states', '/joint_states'),          # Joint state feedback
            ('current_pose', '/arm/current_pose'),      # Current end effector pose

            # Output topics
            ('pose_command', '/arm/pose_command'),      # Pose commands
            ('joint_command', '/arm/joint_command'),    # Joint commands
        ]
    )

    # Create arm controller node
    arm_node = Node(
        package='arm_hand_control',
        executable='agilex_piper_arm',
        name='agilex_piper_arm',
        output='screen',
        parameters=[{
            'config_file': arm_config,
            'can_interface': can_interface
        }],
        remappings=[
            # Input command topics
            ('piper/pose_command', '/arm/pose_command'),      # Pose commands
            ('piper/joint_command', '/arm/joint_command'),    # Joint commands

            # Output feedback topics
            ('joint_states', '/joint_states'),                # Joint state
            ('piper/current_pose', '/arm/current_pose'),      # Current end effector pose
            ('piper/status', '/arm/status')                   # Arm status
        ]
    )

    return LaunchDescription([
        # Launch arguments
        declare_arm_config,
        declare_linear_scale,
        declare_angular_scale,
        declare_joint_vel_scale,
        declare_can_interface,

        # Nodes
        teleop_node,
        arm_node
    ])
