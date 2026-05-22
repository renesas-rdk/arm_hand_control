// ********************************************************************************************************************
// Copyright [2026] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************
#pragma once

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <tuple>
#include <vector>

namespace arm_hand_control
{

struct JointConfig
{
  std::string name;
  std::string finger;
  std::string role;
  double limit_max;
  double default_position;
};

/**
 * Hand landmark interpreter node
 *
 * This node receives hand landmark positions from vision-based tracking systems
 * (like MediaPipe) and converts them into joint positions for a robotic hand.
 * It calculates finger curl and joint angles based on the relative positions
 * of landmarks, applying smoothing for more stable control.
 *
 * Topics:
 * - Subscriptions:
 *   - hand_landmarks (geometry_msgs/PoseArray):
 *     3D positions of hand landmarks (21 points matching MediaPipe model)
 *
 * - Publications:
 *   - joint_states (sensor_msgs/JointState):
 *     Joint positions for the robotic hand
 *
 * Parameters:
 * - config_file (string): Path to the hand configuration YAML file
 * - curl_smooth_factor (float): Smoothing factor for finger curl calculations (0.0-1.0)
 */
class HandLandmarkInterpreter : public rclcpp::Node
{
public:
  HandLandmarkInterpreter();
  ~HandLandmarkInterpreter();

private:
  void load_configuration();
  std::vector<double> get_ordered_positions() const;
  std_msgs::msg::Float64MultiArray build_position_msg(const std::vector<double> & data);
  void landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void publish_joint_states();

  // Helper functions
  void set_finger_position(const std::string & finger, const std::string & role, double percentage);
  void set_finger_positions(const std::string & finger, double percentage);
  void set_all_fingers_except(const std::vector<std::string> & exceptions, double percentage);
  void set_joint_position(const std::string & joint_name, double percentage);
  void reset_joint_positions();

  // Landmark interpretation functions
  void process_landmarks(const std::vector<geometry_msgs::msg::Pose> & landmarks);
  std::tuple<double, double> calculate_finger_curl(
    const std::vector<geometry_msgs::msg::Pose> & landmarks, const std::string & finger);

  // Subscribers, publishers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;

  // Configuration data
  std::string config_file_path_;
  float curl_smooth_factor_;

  // Joint data structures
  std::vector<std::string> joint_names_;
  std::vector<JointConfig> joint_order_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_limits_;
  std::map<std::string, JointConfig> joint_configs_;
  std::map<std::string, std::map<std::string, std::vector<std::string>>> finger_joints_;

  // EMA smoothing storage for finger curl values
  std::map<std::string, double> prev_finger_curls_;

  // Constants for landmark indices (MediaPipe hand landmark model)
  const int HAND_LANDMARK_COUNT = 21;
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
