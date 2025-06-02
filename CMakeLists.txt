cmake_minimum_required(VERSION 3.8)
project(arm_hand_control)

# Build settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(agilex_piper_controller REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteGesture.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

# Include directories
include_directories(include)

# Executables
# Hand gesture interpreter
add_executable(hand_gesture_interpreter src/hand_gesture_interpreter_node.cpp)
ament_target_dependencies(hand_gesture_interpreter
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  ament_index_cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(hand_gesture_interpreter
  "${cpp_typesupport_target}"
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(hand_gesture_interpreter PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}
)

# Hand landmark interpreter
add_executable(hand_landmark_interpreter src/hand_landmark_interpreter_node.cpp)
ament_target_dependencies(hand_landmark_interpreter
  rclcpp
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)
target_link_libraries(hand_landmark_interpreter
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(hand_landmark_interpreter PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}
)

# Inspire RH56 DexHand
add_executable(inspire_rh56_dexhand src/inspire_rh56_dexhand_node.cpp)
ament_target_dependencies(inspire_rh56_dexhand
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)
target_link_libraries(inspire_rh56_dexhand
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(inspire_rh56_dexhand PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}
)

# AgileX Piper Arm
add_executable(agilex_piper_arm src/agilex_piper_arm_node.cpp)
ament_target_dependencies(agilex_piper_arm
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  ament_index_cpp
  agilex_piper_controller
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(agilex_piper_arm
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(agilex_piper_arm PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}
)

# Teleop Twist Controller
add_executable(teleop_twist_controller src/teleop_twist_controller_node.cpp)
ament_target_dependencies(teleop_twist_controller
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
)

# Gesture Action Client
add_executable(gesture_action_client src/gesture_action_client.cpp)
ament_target_dependencies(gesture_action_client
  rclcpp
  rclcpp_action
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(gesture_action_client
  "${cpp_typesupport_target}"
)

# hand landmark twist publisher
add_executable(hand_landmark_twist_publisher src/hand_landmark_twist_publisher.cpp)
ament_target_dependencies(hand_landmark_twist_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  rclcpp_action
)
target_link_libraries(hand_landmark_twist_publisher
  "${cpp_typesupport_target}"
)

# hand landmark pose publisher
add_executable(hand_landmark_pose_publisher src/hand_landmark_pose_publisher.cpp)
ament_target_dependencies(hand_landmark_pose_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  rclcpp_action
)
target_link_libraries(hand_landmark_pose_publisher
  "${cpp_typesupport_target}"
)

# Installation
install(TARGETS
  hand_gesture_interpreter
  hand_landmark_interpreter
  inspire_rh56_dexhand
  agilex_piper_arm
  teleop_twist_controller
  gesture_action_client
  hand_landmark_twist_publisher
  hand_landmark_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
