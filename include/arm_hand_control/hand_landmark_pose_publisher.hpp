// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
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

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

// Include the generated action
#include "arm_hand_control/action/execute_gesture.hpp"

namespace arm_hand_control
{

/**
 * Hand Landmark Pose Publisher Node
 *
 * This node maps hand landmark positions directly to the arm's end effector pose.
 * It uses configurable mapping ranges for position and allows smooth transitions
 * between poses based on hand movement.
 *
 * Topics:
 * - Subscriptions:
 *   - hand_landmarks (geometry_msgs/PoseArray): Hand landmark positions
 *
 * - Publications:
 *   - target_pose (geometry_msgs/PoseStamped): Target end effector pose
 *
 * Parameters:
 * - smoothing_factor (double): Smoothing factor for pose changes [0.0-1.0]
 * - position_scale (double): Scale factor for position mapping
 * - dead_zone_threshold (double): Dead zone threshold for small movements
 * - camera_width (double): Camera image width for normalization
 * - camera_height (double): Camera image height for normalization
 * - target_frame (string): Target frame ID for published poses
 * - initial_pose_x/y/z (double): Initial end effector position
 * - initial_pose_roll/pitch/yaw (double): Initial end effector orientation
 * - max_pose_x/y/z (double): Maximum end effector position
 * - min_pose_x/y/z (double): Minimum end effector position
 */

class HandLandmarkPosePublisher : public rclcpp::Node
{
public:
  HandLandmarkPosePublisher();
  virtual ~HandLandmarkPosePublisher() = default;

private:
  //===== Action Type Definitions =====
  using ExecuteGesture = arm_hand_control::action::ExecuteGesture;
  using GoalHandleExecuteGesture = rclcpp_action::ClientGoalHandle<ExecuteGesture>;

  //===== MediaPipe Hand Landmark Indices =====
  static constexpr int WRIST_IDX = 0;
  static constexpr int THUMB_TIP_IDX = 4;
  static constexpr int INDEX_MCP_IDX = 5;
  static constexpr int MIDDLE_MCP_IDX = 9;
  static constexpr int PINKY_MCP_IDX = 17;

  //===== ROS Communication =====
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_publisher_;
  rclcpp_action::Client<ExecuteGesture>::SharedPtr gesture_client_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  //===== Configuration Parameters =====
  double smoothing_factor_;
  double position_scale_;
  double dead_zone_threshold_;
  double camera_width_;
  double camera_height_;
  std::string target_frame_;

  //===== Pose Mapping Parameters =====
  double initial_pose_x_, initial_pose_y_, initial_pose_z_;
  double initial_pose_roll_, initial_pose_pitch_, initial_pose_yaw_;
  double max_pose_x_, max_pose_y_, max_pose_z_;
  double min_pose_x_, min_pose_y_, min_pose_z_;

  //===== Gripper Control Parameters =====
  double max_gripper_position_;
  double min_gripper_position_;
  double max_gripper_effort_;
  double gripper_command_threshold_;

  //===== State Management =====
  bool has_reference_;
  std::vector<geometry_msgs::msg::Pose> reference_landmarks_;
  geometry_msgs::msg::Pose reference_x_landmark_;
  geometry_msgs::msg::Pose reference_y_landmark_;
  double reference_z_distance_;
  double last_grasp_percentage_;
  geometry_msgs::msg::PoseStamped previous_pose_;
  std::chrono::time_point<std::chrono::steady_clock> last_detection_time_;
  std::chrono::time_point<std::chrono::steady_clock> continuous_detection_start_;

  //===== Constants =====
  static constexpr int HAND_LANDMARK_COUNT = 21;
  static constexpr auto DETECTION_REQUIRED_DURATION = std::chrono::seconds(2);
  static constexpr auto DETECTION_TIMEOUT_DURATION = std::chrono::seconds(1);

  //===== Callback Methods =====
  void landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void check_detection_timeout();

  //===== Processing Methods =====
  double calculate_x_position_change(
    const std::vector<geometry_msgs::msg::Pose> & landmarks, double current_palm_size_pixels);
  double calculate_y_position_change(
    const std::vector<geometry_msgs::msg::Pose> & landmarks, double current_palm_size_pixels);
  double calculate_z_position_change(const std::vector<geometry_msgs::msg::Pose> & landmarks);
  void process_grasp_gesture(const std::vector<geometry_msgs::msg::Pose> & landmarks);
  double calculate_thumb_index_distance(const std::vector<geometry_msgs::msg::Pose> & landmarks);
  void send_grasp_goal(float percentage);
  void send_gripper_command(double grasp_percentage);

  //===== Utility Methods =====
  double calculate_distance(
    const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2);
  double apply_smoothing(double current, double previous, double factor);
  double apply_dead_zone(double value, double threshold);
  double clamp_value(double value, double min_val, double max_val);
  double map_to_range(
    double normalized_value, double min_range, double max_range, double initial_value);
};

}  // namespace arm_hand_control
