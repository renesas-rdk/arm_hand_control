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

#include <atomic>
#include <chrono>
#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_hand_control/action/pick_place.hpp"

namespace arm_hand_control
{

// Pick and Place Action Server
//
// This node provides an action server for executing pick-and-place operations
// using the Agilex Piper arm and gripper. It implements a state machine that
// sequences through the stages of a pick-and-place operation.
//
// Action Server:
// - pick_place (arm_hand_control/action/PickPlace): Execute pick-and-place operations
//
// Published Topics:
// - /arm/pose_command (geometry_msgs/PoseStamped): Commands for arm end-effector
// - /arm/gripper_command (control_msgs/GripperCommand): Commands for gripper
//
// Subscribed Topics:
// - /arm/current_pose (geometry_msgs/PoseStamped): Current end-effector pose
// - /arm/joint_states (sensor_msgs/JointState): Current joint states
// - /arm/status (std_msgs/UInt8MultiArray): Arm status information
//
// Parameters:
// - position_tolerance (double): Position tolerance in meters for pose reached check (default: 0.005)
// - orientation_tolerance (double): Orientation tolerance in radians for pose reached check (default: 0.05)
// - move_timeout (double): Timeout in seconds for each move operation (default: 10.0)
// - gripper_timeout (double): Timeout in seconds for gripper operations (default: 3.0)
// - gripper_settle_time (double): Time in seconds to wait for gripper to settle (default: 1.0)

class PickPlaceActionServer : public rclcpp::Node
{
public:
  using PickPlace = arm_hand_control::action::PickPlace;
  using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

  explicit PickPlaceActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PickPlaceActionServer() = default;

private:
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PickPlace::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  // Main execution method
  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  // State machine stages
  bool approach_pick(
    const geometry_msgs::msg::PoseStamped & pick_pose, double approach_height,
    std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool descend_to_pick(
    const geometry_msgs::msg::PoseStamped & pick_pose,
    std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool close_gripper(
    double closed_position, double force, std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool lift_object(
    const geometry_msgs::msg::PoseStamped & pick_pose, double approach_height,
    std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool approach_place(
    const geometry_msgs::msg::PoseStamped & place_pose, double approach_height,
    std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool descend_to_place(
    const geometry_msgs::msg::PoseStamped & place_pose,
    std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool open_gripper(
    double open_position, double force, std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool retreat_from_place(
    const geometry_msgs::msg::PoseStamped & place_pose, double approach_height,
    std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  // Helper methods
  bool set_arm_high_speed(bool high_speed);
  bool move_to_pose(
    const geometry_msgs::msg::PoseStamped & target_pose, const std::string & stage_name,
    float progress_start, float progress_end, std::shared_ptr<PickPlace::Feedback> feedback,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  bool wait_for_gripper_command(double timeout_seconds);

  bool is_pose_reached(
    const geometry_msgs::msg::PoseStamped & target, const geometry_msgs::msg::PoseStamped & current,
    double position_tolerance, double orientation_tolerance);

  void abort_with_message(
    const std::string & message, std::shared_ptr<PickPlace::Result> result,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle);

  geometry_msgs::msg::PoseStamped create_approach_pose(
    const geometry_msgs::msg::PoseStamped & target_pose, double approach_height);

  // Subscriber callbacks
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void status_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  // ROS2 communication
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr status_sub_;

  // Service client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr high_speed_client_;

  // Action server
  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;

  // State tracking
  mutable std::mutex state_mutex_;
  geometry_msgs::msg::PoseStamped current_pose_;
  sensor_msgs::msg::JointState current_joint_state_;
  std::vector<uint8_t> current_status_;
  std::atomic<bool> gripper_command_sent_{false};
  std::chrono::steady_clock::time_point last_gripper_command_time_;

  // Parameters
  double position_tolerance_;     // meters
  double orientation_tolerance_;  // radians
  double move_timeout_;           // seconds
  double gripper_timeout_;        // seconds
  double gripper_settle_time_;    // seconds
};

}  // namespace arm_hand_control
