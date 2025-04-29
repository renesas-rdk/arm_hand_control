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
        description='Linear scale for twist commands'
    )

    declare_angular_scale = DeclareLaunchArgument(
        'angular_scale',
        default_value='0.01',
        description='Angular scale for twist commands'
    )

    declare_joint_vel_scale = DeclareLaunchArgument(
        'joint_vel_scale',
        default_value='0.01',
        description='Joint velocity scale for twist commands'
    )

    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use for the arm'
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
            ('pose/cmd_vel', '/arm/pose/cmd_vel'),    # Input pose twist commands
            ('joint/cmd_vel', '/arm/joint/cmd_vel'),  # Input joint twist commands
            ('joint_states', '/joint_states'),        # Input joint state feedback
            ('current_pose', '/arm/current_pose'),    # Input current end effector pose
            ('pose_command', '/arm/pose_command'),    # Output pose commands
            ('joint_command', '/arm/joint_command'),  # Output joint commands
        ]
    )

    # Create arm node
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
            ('piper/pose_command', '/arm/pose_command'),    # Input pose commands
            ('piper/joint_command', '/arm/joint_command'),  # Input joint commands
            ('joint_states', '/joint_states'),              # Output joint state
            ('piper/current_pose', '/arm/current_pose'),    # Output current end effector pose
            ('piper/status', '/arm/status')                 # Output arm status
        ]
    )

    return LaunchDescription([
        declare_arm_config,
        declare_linear_scale,
        declare_angular_scale,
        declare_joint_vel_scale,
        declare_can_interface,
        teleop_node,
        arm_node
    ])
