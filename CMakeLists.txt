cmake_minimum_required(VERSION 3.8)
project(arm_hand_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(include)

# Executable
add_executable(hand_gesture_interpreter src/hand_gesture_interpreter_node.cpp)
ament_target_dependencies(hand_gesture_interpreter
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)
target_link_libraries(hand_gesture_interpreter ${YAML_CPP_LIBRARIES})
target_include_directories(hand_gesture_interpreter PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# Install executables
install(TARGETS
  hand_gesture_interpreter
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
