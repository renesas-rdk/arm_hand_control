cmake_minimum_required(VERSION 3.8)
project(arm_hand_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

# Add the hand gesture interpreter executable
add_executable(hand_gesture_interpreter src/hand_gesture_interpreter_node.cpp)
ament_target_dependencies(hand_gesture_interpreter
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  ament_index_cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(hand_gesture_interpreter "${cpp_typesupport_target}")
target_link_libraries(hand_gesture_interpreter ${YAML_CPP_LIBRARIES})
target_include_directories(hand_gesture_interpreter PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# Add the hand landmark interpreter executable
add_executable(hand_landmark_interpreter src/hand_landmark_interpreter_node.cpp)
ament_target_dependencies(hand_landmark_interpreter
  rclcpp
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)
target_link_libraries(hand_landmark_interpreter ${YAML_CPP_LIBRARIES})
target_include_directories(hand_landmark_interpreter PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# Add the inspire rh56 dexhand executable
add_executable(inspire_rh56_dexhand src/inspire_rh56_dexhand_node.cpp)
ament_target_dependencies(inspire_rh56_dexhand
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)
target_link_libraries(inspire_rh56_dexhand ${YAML_CPP_LIBRARIES})
target_include_directories(inspire_rh56_dexhand PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# Add the AgileX Piper arm executable
add_executable(agilex_piper_arm src/agilex_piper_arm_node.cpp)
ament_target_dependencies(agilex_piper_arm
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  ament_index_cpp
  agilex_piper_controller
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(agilex_piper_arm ${YAML_CPP_LIBRARIES})
target_include_directories(agilex_piper_arm PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# Add the Piper teleop node executable
add_executable(piper_teleop src/piper_teleop_node.cpp)
ament_target_dependencies(piper_teleop
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  ament_index_cpp
  tf2
  tf2_geometry_msgs
)
target_link_libraries(piper_teleop ${YAML_CPP_LIBRARIES})
target_include_directories(piper_teleop PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# Add the action client executable
add_executable(gesture_action_client src/gesture_action_client.cpp)
ament_target_dependencies(gesture_action_client
  rclcpp
  rclcpp_action
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(gesture_action_client "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  hand_gesture_interpreter
  hand_landmark_interpreter
  inspire_rh56_dexhand
  agilex_piper_arm
  piper_teleop
  gesture_action_client
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
