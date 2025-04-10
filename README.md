# Arm Hand Control

A ROS 2 package for controlling robotic hands through gesture recognition and landmark tracking.

## Overview

This package provides nodes for controlling robotic hands, particularly the Inspire RH56 Dexhand. It supports:
- Hand gesture interpretation (predefined gestures)
- Hand landmark interpretation (from hand tracking algorithms)
- Direct control of the Inspire RH56 Dexhand hardware

## Nodes

### 1. Hand Gesture Interpreter (`hand_gesture_interpreter_node`)

Subscribes to string-based gesture commands and converts them into joint positions for the robotic hand.

- **Subscriptions**:
  - `hand_gesture` (std_msgs/String) - Commands like "grasp", "pinch", "point", etc.
- **Publications**:
  - `joint_states` (sensor_msgs/JointState) - Joint positions for the robotic hand

Supported gestures:
- grasp - Close all fingers
- pinch - Pinch gesture with thumb and index finger
- point - Extend index finger only
- thumbs_up - Extend thumb, close other fingers
- thumbs_down - Position thumb downward
- ok - Form an "OK" gesture
- rock - Extend index and pinky fingers
- peace - Extend index and middle fingers
- three_finger_grasp - Grasp with three fingers
- call_me - Extend thumb and pinky
- one, two, three, four, five - Numerical counting gestures
- open_hand - Open all fingers
- grasp_X - Grasp with percentage X (e.g., "grasp_0.5" for 50% closed)

### 2. Hand Landmark Interpreter (`hand_landmark_interpreter_node`)

Transforms hand landmark positions (e.g., from MediaPipe or other vision systems) into joint positions.

- **Subscriptions**:
  - `hand_landmarks` (geometry_msgs/PoseArray) - 3D positions of hand landmarks
- **Publications**:
  - `joint_states` (sensor_msgs/JointState) - Joint positions for the robotic hand

Processes 21 landmarks corresponding to the MediaPipe hand tracking model.

### 3. Inspire RH56 Dexhand Controller (`inspire_rh56_dexhand_node`)

Hardware interface for the Inspire RH56 Dexhand via serial communication.

- **Subscriptions**:
  - `joint_states` (sensor_msgs/JointState) - Joint positions to apply
- **Hardware Interface**:
  - Communicates with the hand over a serial connection

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

3. Inspire RH56 Dexhand:
```bash
ros2 launch arm_hand_control inspire_rh56_dexhand.launch.py
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

## Dependencies

- ROS 2
- sensor_msgs
- std_msgs
- geometry_msgs
- yaml-cpp
- ament_index_cpp

## License

This package is licensed under the MIT License.
