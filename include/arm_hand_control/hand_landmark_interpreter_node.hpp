#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>

namespace arm_hand_control
{

// Struct to represent joint configuration
struct JointConfig
{
  std::string name;
  std::string finger;
  std::string role;
  double limit_max;
  double default_position;
};

class HandLandmarkInterpreter : public rclcpp::Node
{
public:
  HandLandmarkInterpreter();

private:
  // Configuration functions
  void load_configuration();

  // Callback functions
  void landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void publish_joint_states();

  // Helper functions
  void set_finger_position(const std::string& finger, const std::string& role, double position);
  void set_finger_positions(const std::string& finger, double percentage);
  void set_all_fingers_except(const std::vector<std::string>& exceptions, double percentage);
  void set_joint_position(const std::string& joint_name, double position);
  void reset_joint_positions();

  // Landmark interpretation functions
  void process_landmarks(const std::vector<geometry_msgs::msg::Pose>& landmarks);
  double calculate_finger_curl(const std::vector<geometry_msgs::msg::Pose>& landmarks, int start_idx, int num_joints);

  // Subscribers, publishers, and timers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Configuration data
  std::string config_file_path_;
  double publish_rate_hz_ = 30.0;

  // Joint data structures
  std::vector<std::string> joint_names_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_limits_;
  std::map<std::string, JointConfig> joint_configs_;
  std::map<std::string, std::map<std::string, std::vector<std::string>>> finger_joints_;

  // Landmark data
  std::vector<geometry_msgs::msg::Pose> last_landmarks_;
  bool landmarks_received_ = false;

  // Constants for landmark indices (MediaPipe hand landmark model)
  const int WRIST_IDX = 0;
  const int THUMB_CMC_IDX = 1;    // Thumb metacarpal
  const int THUMB_MCP_IDX = 2;    // Thumb metacarpophalangeal
  const int THUMB_IP_IDX = 3;     // Thumb interphalangeal
  const int THUMB_TIP_IDX = 4;    // Thumb tip
  const int INDEX_MCP_IDX = 5;    // Index finger metacarpophalangeal
  const int INDEX_PIP_IDX = 6;    // Index finger proximal interphalangeal
  const int INDEX_DIP_IDX = 7;    // Index finger distal interphalangeal
  const int INDEX_TIP_IDX = 8;    // Index finger tip
  const int MIDDLE_MCP_IDX = 9;   // Middle finger metacarpophalangeal
  const int MIDDLE_PIP_IDX = 10;  // Middle finger proximal interphalangeal
  const int MIDDLE_DIP_IDX = 11;  // Middle finger distal interphalangeal
  const int MIDDLE_TIP_IDX = 12;  // Middle finger tip
  const int RING_MCP_IDX = 13;    // Ring finger metacarpophalangeal
  const int RING_PIP_IDX = 14;    // Ring finger proximal interphalangeal
  const int RING_DIP_IDX = 15;    // Ring finger distal interphalangeal
  const int RING_TIP_IDX = 16;    // Ring finger tip
  const int PINKY_MCP_IDX = 17;   // Pinky finger metacarpophalangeal
  const int PINKY_PIP_IDX = 18;   // Pinky finger proximal interphalangeal
  const int PINKY_DIP_IDX = 19;   // Pinky finger distal interphalangeal
  const int PINKY_TIP_IDX = 20;   // Pinky finger tip
};

}  // namespace arm_hand_control
