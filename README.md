# Arm Hand Control

A ROS 2 package for controlling robotic hands through gesture recognition and landmark tracking.

## Overview

This package provides nodes for controlling robotic hands, particularly the Inspire RH56 Dexhand. It supports:
- Hand gesture interpretation (predefined gestures)
- Hand landmark interpretation (from hand tracking algorithms)
- Direct control of the Inspire RH56 Dexhand hardware
- Pick-and-place operations via action server

## Nodes

### 1. Hand Gesture Interpreter (`hand_gesture_interpreter_node`)

Subscribes to string-based gesture commands and converts them into joint positions for the robotic hand.

- **Subscriptions**:
  - `hand_gesture` (std_msgs/String) - Commands like "grasp", "pinch", "point", etc.
- **Publications**:
  - `joint_states` (sensor_msgs/JointState) - Joint positions for the robotic hand

Supported gestures:
- **Basic hand gestures**:
  - grasp - Close all fingers
  - pinch - Pinch gesture with thumb and index finger
  - three_finger_grasp - Grasp with three fingers
  - open_hand - Open all fingers
  - loose_fist - A loosely closed hand
  - grasp_X - Grasp with percentage X (e.g., "grasp_0.5" for 50% closed)

- **Counting gestures**:
  - one - Extend index finger only
  - two - Extend index and middle fingers
  - three - Extend thumb, index, and middle fingers
  - four - Extend all fingers except thumb
  - five - Open all fingers

- **Communication gestures**:
  - point - Extend index finger only
  - thumbs_up - Extend thumb, close other fingers
  - thumbs_down - Position thumb downward
  - ok - Form an "OK" gesture
  - call_me - Extend thumb and pinky
  - peace - Extend index and middle fingers
  - wave - Wave gesture (parameter controlled)

- **Fun/special gestures**:
  - rock - Extend index and pinky fingers
  - fist_bump - Closed fist with thumb positioned alongside
  - gun - Extend index finger with thumb perpendicular
  - spider_man - Extend thumb, index, and pinky
  - hang_loose - Extend thumb, index, and pinky
  - thumbs_middle - Extend middle finger with thumb to the side
  - finger_cross - Cross fingers gesture
  - italian_hand - Traditional Italian hand gesture

- **Demo mode control**:
  - demo_start - Start automated cycling through gestures
  - demo_stop - Stop automated gesture cycling

### 2. Hand Landmark Interpreter (`hand_landmark_interpreter_node`)

Transforms hand landmark positions (e.g., from MediaPipe or other vision systems) into joint positions.

- **Subscriptions**:
  - `hand_landmarks` (geometry_msgs/PoseArray) - 3D positions of hand landmarks
- **Publications**:
  - `joint_states` (sensor_msgs/JointState) - Joint positions for the robotic hand

Processes 21 landmarks corresponding to the MediaPipe hand tracking model.

### 3. Pick-Place Action Server (`pick_place_action_server`)

Provides an action server for executing pick-and-place operations with the robotic arm and gripper.

- **Action Server**:
  - `pick_place` (arm_hand_control/action/PickPlace) - Execute pick-and-place operations

- **Published Topics**:
  - `/arm/pose_command` (geometry_msgs/PoseStamped) - Commands for arm end-effector
  - `/arm/gripper_command` (control_msgs/GripperCommand) - Commands for gripper

- **Subscribed Topics**:
  - `/arm/current_pose` (geometry_msgs/PoseStamped) - Current end-effector pose
  - `/arm/joint_states` (sensor_msgs/JointState) - Current joint states
  - `/arm/status` (std_msgs/UInt8MultiArray) - Arm status information

The action server implements a state machine that sequences through:
1. Open gripper (initial)
2. Approach pick position
3. Descend to pick
4. Close gripper
5. Lift object
6. Approach place position
7. Descend to place
8. Open gripper
9. Retreat from place
10. Return to home position (optional, controlled by `return_to_home` parameter in action goal)

## Installation

```bash
# Create workspace (if not already created)
mkdir -p ~/ws/rz_ros2_ws/src
cd ~/ws/rz_ros2_ws/src

# Clone the repository (assuming it's part of a larger project)
# git clone <repository-url>

# Build the package
cd ~/ws/rz_ros2_ws
colcon build --symlink-install --packages-select arm_hand_control

# Source the workspace
source install/setup.bash
```

## Usage

### Launch Files

The package includes several launch files:

1. Hand Landmark Interpreter:
```bash
ros2 launch arm_hand_control hand_landmark_interpreter.launch.py
```

2. Hand Gesture Interpreter:
```bash
ros2 launch arm_hand_control hand_gesture_interpreter.launch.py
```

3. Pick-Place Action Server:
```bash
ros2 launch arm_hand_control pick_place_server.launch.py
```

### Configuration

Configuration for the robotic hand is stored in YAML files in the `config/hand/` directory.
The default configuration is for the Inspire RH56 Dexhand with the following joints:

- Thumb (yaw and pitch)
- Index finger
- Middle finger
- Ring finger
- Pinky finger

### Examples

Send a gesture command:
```bash
ros2 topic pub /hand_gesture std_msgs/String "data: 'grasp'"
```

Send a gesture with parameter:
```bash
ros2 topic pub /hand_gesture std_msgs/String "data: 'grasp_0.5'"
```

Start demo mode to cycle through all gestures:
```bash
ros2 topic pub /hand_gesture std_msgs/String "data: 'demo_start'"
```

Send a pick-and-place action goal (using command line):
```bash
ros2 action send_goal /pick_place arm_hand_control/action/PickPlace \
  "{pick_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.17}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}, \
   place_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.1, z: 0.17}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}, \
   approach_height: 0.07, gripper_open_position: 0.03, gripper_closed_position: 0.01, gripper_force: 1.0, return_to_home: true}"
```

To execute a pick-and-place without returning to home position:
```bash
ros2 action send_goal /pick_place arm_hand_control/action/PickPlace \
  "{pick_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.17}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}, \
   place_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.1, z: 0.17}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}, \
   approach_height: 0.07, gripper_open_position: 0.03, gripper_closed_position: 0.01, gripper_force: 1.0, return_to_home: false}"
```

## Dependencies

- ROS 2
- sensor_msgs
- std_msgs
- geometry_msgs
- control_msgs
- trajectory_msgs
- tf2
- tf2_geometry_msgs
- yaml-cpp
- ament_index_cpp

## License

This package is licensed under the MIT License.
