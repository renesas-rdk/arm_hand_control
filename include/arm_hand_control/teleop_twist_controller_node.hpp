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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_hand_control
{

/**
 * TeleopTwistControllerNode handles twist commands for both pose and joint control
 *
 * The node provides two separate control streams:
 *
 * Subscribed topics:
 *   - pose/cmd_vel (geometry_msgs/Twist): Controls end effector position and orientation
 *   - joint/cmd_vel (geometry_msgs/Twist): Controls individual joint positions
 *   - current_pose (geometry_msgs/PoseStamped): Current end effector pose
 *   - joint_states (sensor_msgs/JointState): Current joint states
 *
 * Published topics:
 *   - pose_command (geometry_msgs/PoseStamped): Commanded pose for the arm
 *   - joint_command (trajectory_msgs/JointTrajectory): Commanded joint positions
 *
 * Parameters:
 *   - linear_scale (double): Scale factor for linear velocity (m per unit twist)
 *   - angular_scale (double): Scale factor for angular velocity (rad per unit twist)
 *   - joint_vel_scale (double): Scale factor for joint velocity (rad per unit twist)
 *   - joint_names (string_array): List of joint names for trajectory control
 */
class TeleopTwistControllerNode : public rclcpp::Node
{
public:
  explicit TeleopTwistControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void pose_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joint_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  // Helper methods
  void publish_pose_command(const geometry_msgs::msg::PoseStamped & pose);
  void publish_joint_command(const sensor_msgs::msg::JointState & joint_state);
  sensor_msgs::msg::JointState filter_joint_state(const sensor_msgs::msg::JointState & joint_state);

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pose_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joint_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Parameters
  double linear_scale_;                   // Scale factor for linear velocity (m per unit twist)
  double angular_scale_;                  // Scale factor for angular velocity (rad per unit twist)
  double joint_vel_scale_;                // Scale factor for joint velocity (rad per unit twist)
  std::vector<std::string> joint_names_;  // List of joint names for trajectory control

  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  sensor_msgs::msg::JointState current_joints_;
  bool have_pose_;    // Flag indicating if we have received pose data
  bool have_joints_;  // Flag indicating if we have received joint data
};

}  // namespace arm_hand_control
