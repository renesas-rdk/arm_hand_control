# arm_hand_control

ROS 2 package providing nodes for controlling a robotic arm and dexterous hand
from vision-based hand landmarks, gesture commands, and pick-and-place actions.

## Nodes

| Executable | Description |
|---|---|
| `hand_gesture_interpreter` | Maps string gesture commands (`grasp`, `pinch`, `one`, …) to dexterous-hand joint positions. |
| `hand_landmark_interpreter` | Converts a 21-point MediaPipe hand-landmark `PoseArray` into per-finger joint positions. |
| `hand_landmark_gripper_retargeter` | Retargets thumb / index / middle pinch geometry into a 1-DoF parallel `GripperCommand`. |
| `hand_landmark_pose_publisher` | Publishes hand-landmark poses for visualization / downstream consumers. |
| `teleop_twist_controller` | Converts `Twist` input into joint trajectory commands for the arm. |
| `pick_place_action_server` | Action server executing a pick-and-place sequence with arm pose + gripper commands. |
| `gesture_action_client` | CLI client for the `ExecuteGesture` action. |

## Actions

- `ExecuteGesture.action`
- `PickPlace.action`

## Launch Files

- `hand_gesture_interpreter.launch.py`
- `hand_landmark_interpreter.launch.py`
- `hand_landmark_gripper_retargeter.launch.py`
- `pick_place_server.launch.py`

## Usage

```bash
# Interpret hand landmarks into hand joint commands
ros2 launch arm_hand_control hand_landmark_interpreter.launch.py \
    config_file:=<path/to/hand_config.yaml>

# Retarget hand landmarks into a parallel gripper command
ros2 launch arm_hand_control hand_landmark_gripper_retargeter.launch.py

# Drive the hand with a named gesture
ros2 topic pub /hand_gesture std_msgs/String "data: 'grasp'"

# Run a pick-and-place sequence
ros2 launch arm_hand_control pick_place_server.launch.py
```

## Topics

| Direction | Topic | Type | Used by |
|---|---|---|---|
| sub | `hand_landmarks` | `geometry_msgs/PoseArray` | landmark interpreter, gripper retargeter |
| sub | `hand_gesture` | `std_msgs/String` | gesture interpreter |
| pub | `position_controller_command` | `std_msgs/Float64MultiArray` | landmark interpreter |
| pub | `hand_gripper_command` | `control_msgs/GripperCommand` | gripper retargeter |

## License

Apache License 2.0
