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
#include "arm_hand_control/pick_place_action_server.hpp"

#include <tf2/utils.h>

#include <cmath>

namespace arm_hand_control
{

PickPlaceActionServer::PickPlaceActionServer(const rclcpp::NodeOptions & options)
: Node("pick_place_action_server", options)
{
  // Declare control parameters
  this->declare_parameter("position_tolerance", 0.005);    // 5mm
  this->declare_parameter("orientation_tolerance", 0.05);  // ~3 degrees
  this->declare_parameter("move_timeout", 2.0);            // 2 seconds
  this->declare_parameter("gripper_settle_time", 0.5);     // 0.5 second

  // Home position parameters (default values for Piper arm)
  this->declare_parameter("use_current_pose_as_home", true);  // Use first received pose as home
  this->declare_parameter("home_position.x", 0.06);
  this->declare_parameter("home_position.y", 0.0);
  this->declare_parameter("home_position.z", 0.22);
  this->declare_parameter("home_orientation.x", 0.0);
  this->declare_parameter("home_orientation.y", 0.68);
  this->declare_parameter("home_orientation.z", 0.0);
  this->declare_parameter("home_orientation.w", 0.74);
  this->declare_parameter("home_gripper_position", 0.05);

  // Get parameters
  position_tolerance_ = this->get_parameter("position_tolerance").as_double();
  orientation_tolerance_ = this->get_parameter("orientation_tolerance").as_double();
  move_timeout_ = this->get_parameter("move_timeout").as_double();
  gripper_settle_time_ = this->get_parameter("gripper_settle_time").as_double();

  // Create publishers
  arm_cmd_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/arm/pose_command", 10);
  gripper_cmd_pub_ =
    this->create_publisher<control_msgs::msg::GripperCommand>("/arm/gripper_command", 10);

  // Speed publisher with TRANSIENT_LOCAL QoS for reliable delivery
  // This ensures late-joining subscribers receive the last published value
  auto speed_qos = rclcpp::QoS(10)
                     .reliability(rclcpp::ReliabilityPolicy::Reliable)
                     .durability(rclcpp::DurabilityPolicy::TransientLocal);
  speed_pub_ =
    this->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>("/arm/speed", speed_qos);

  // Create subscribers
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/arm/current_pose", 10,
    std::bind(&PickPlaceActionServer::pose_callback, this, std::placeholders::_1));

  // Create action server
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<PickPlace>(
    this, "pick_place", std::bind(&PickPlaceActionServer::handle_goal, this, _1, _2),
    std::bind(&PickPlaceActionServer::handle_cancel, this, _1),
    std::bind(&PickPlaceActionServer::handle_accepted, this, _1));

  // Initialize home pose
  bool use_current_pose_as_home = this->get_parameter("use_current_pose_as_home").as_bool();
  home_gripper_position_ = this->get_parameter("home_gripper_position").as_double();
  home_pose_initialized_ = false;

  if (use_current_pose_as_home) {
    RCLCPP_INFO(this->get_logger(), "Home position will be set from first received pose");
    // Home pose will be set in pose_callback when first message arrives
  } else {
    // Use parameters for home position
    home_pose_.header.frame_id = "base_link";
    home_pose_.pose.position.x = this->get_parameter("home_position.x").as_double();
    home_pose_.pose.position.y = this->get_parameter("home_position.y").as_double();
    home_pose_.pose.position.z = this->get_parameter("home_position.z").as_double();
    home_pose_.pose.orientation.x = this->get_parameter("home_orientation.x").as_double();
    home_pose_.pose.orientation.y = this->get_parameter("home_orientation.y").as_double();
    home_pose_.pose.orientation.z = this->get_parameter("home_orientation.z").as_double();
    home_pose_.pose.orientation.w = this->get_parameter("home_orientation.w").as_double();
    home_pose_initialized_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Home position from parameters: [%.3f, %.3f, %.3f]",
      home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
  }

  RCLCPP_INFO(this->get_logger(), "Pick-Place Action Server initialized");
  RCLCPP_INFO(this->get_logger(), "Position tolerance: %.3f m", position_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Orientation tolerance: %.3f rad", orientation_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Move timeout: %.1f s", move_timeout_);
}

rclcpp_action::GoalResponse PickPlaceActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid [[maybe_unused]],
  std::shared_ptr<const PickPlace::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received pick-place goal request");

  // Validate goal
  if (goal->approach_height <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid approach height: %.3f", goal->approach_height);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->pick_pose.header.frame_id.empty() || goal->place_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Pick or place pose missing frame_id");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->pick_pose.header.frame_id != goal->place_pose.header.frame_id) {
    RCLCPP_ERROR(this->get_logger(), "Pick and place poses must be in the same frame");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickPlaceActionServer::handle_cancel(
  const std::shared_ptr<GoalHandlePickPlace> goal_handle [[maybe_unused]])
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel pick-place goal");

  // Open gripper to safe state
  auto gripper_cmd = control_msgs::msg::GripperCommand();
  gripper_cmd.position = 0.05;  // Default open position
  gripper_cmd.max_effort = 1.0;
  gripper_cmd_pub_->publish(gripper_cmd);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickPlaceActionServer::handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  // Execute in a separate thread to avoid blocking
  std::thread{std::bind(&PickPlaceActionServer::execute, this, goal_handle)}.detach();
}

void PickPlaceActionServer::execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PickPlace::Feedback>();
  auto result = std::make_shared<PickPlace::Result>();

  RCLCPP_INFO(this->get_logger(), "Starting pick-place execution");

  // Set initial speed to high speed (100%)
  set_arm_speed(100.0);

  // Stage 1: Open gripper
  if (!open_gripper(goal->gripper_open_position, goal->gripper_force, feedback, goal_handle)) {
    abort_with_message("Failed to open gripper", result, goal_handle);
    return;
  }

  // Stage 2: Approach pick position
  if (!approach_pick(goal->pick_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to approach pick position", result, goal_handle);
    return;
  }

  // Set low speed for descending (10%)
  set_arm_speed(10.0);

  // Stage 3: Descend to pick position
  if (!descend_to_pick(goal->pick_pose, feedback, goal_handle)) {
    abort_with_message("Failed to descend to pick position", result, goal_handle);
    return;
  }

  // Stage 4: Close gripper
  if (!close_gripper(goal->gripper_closed_position, goal->gripper_force, feedback, goal_handle)) {
    abort_with_message("Failed to close gripper", result, goal_handle);
    return;
  }

  // Stage 5: Lift object
  if (!lift_object(goal->pick_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to lift object", result, goal_handle);
    return;
  }

  // Set high speed for transit (100%)
  set_arm_speed(100.0);

  // Stage 6: Approach place position
  if (!approach_place(goal->place_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to approach place position", result, goal_handle);
    return;
  }

  // Set low speed for descending (10%)
  set_arm_speed(10.0);

  // Stage 7: Descend to place position
  if (!descend_to_place(goal->place_pose, feedback, goal_handle)) {
    abort_with_message("Failed to descend to place position", result, goal_handle);
    return;
  }

  // Stage 8: Open gripper
  if (!open_gripper(goal->gripper_open_position, goal->gripper_force, feedback, goal_handle)) {
    abort_with_message("Failed to open gripper", result, goal_handle);
    return;
  }

  // Stage 9: Retreat from place position
  if (!retreat_from_place(goal->place_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to retreat from place position", result, goal_handle);
    return;
  }

  // Reset to high speed (100%)
  set_arm_speed(100.0);

  // Stage 10: Return to home position (optional based on action request)
  if (goal->return_to_home) {
    feedback->stage = "Returning to home position";
    feedback->progress = 0.95f;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      feedback->current_pose = current_pose_;
    }
    goal_handle->publish_feedback(feedback);

    if (!move_to_home()) {
      RCLCPP_WARN(this->get_logger(), "Failed to return to home position");
      // Don't abort the mission, just warn
    } else {
      feedback->progress = 1.0f;
      goal_handle->publish_feedback(feedback);
    }
  } else {
    // Skip home position, just update progress to complete
    RCLCPP_INFO(this->get_logger(), "Skipping return to home position as requested");
    feedback->stage = "Operation complete";
    feedback->progress = 1.0f;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      feedback->current_pose = current_pose_;
    }
    goal_handle->publish_feedback(feedback);
  }

  // Success!
  result->success = true;
  result->message = "Pick-and-place completed successfully";
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Pick-place execution completed successfully");
}

bool PickPlaceActionServer::approach_pick(
  const geometry_msgs::msg::PoseStamped & pick_pose, double approach_height,
  std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  auto approach_pose = create_approach_pose(pick_pose, approach_height);
  return move_to_pose(
    approach_pose, "Approaching pick position", 0.1f, 0.2f, feedback, goal_handle);
}

bool PickPlaceActionServer::descend_to_pick(
  const geometry_msgs::msg::PoseStamped & pick_pose, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  return move_to_pose(pick_pose, "Descending to pick position", 0.2f, 0.3f, feedback, goal_handle);
}

bool PickPlaceActionServer::close_gripper(
  double closed_position, double force, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  feedback->stage = "Closing gripper";
  feedback->progress = 0.35f;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback->current_pose = current_pose_;
  }
  goal_handle->publish_feedback(feedback);

  // Send gripper command
  auto gripper_cmd = control_msgs::msg::GripperCommand();
  gripper_cmd.position = closed_position;
  gripper_cmd.max_effort = force;
  gripper_cmd_pub_->publish(gripper_cmd);

  // Wait for gripper to settle
  wait_for_gripper_settle(gripper_settle_time_);

  feedback->progress = 0.4f;
  goal_handle->publish_feedback(feedback);
  return true;
}

bool PickPlaceActionServer::lift_object(
  const geometry_msgs::msg::PoseStamped & pick_pose, double approach_height,
  std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  auto lift_pose = create_approach_pose(pick_pose, approach_height);
  return move_to_pose(lift_pose, "Lifting object", 0.4f, 0.5f, feedback, goal_handle);
}

bool PickPlaceActionServer::approach_place(
  const geometry_msgs::msg::PoseStamped & place_pose, double approach_height,
  std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  auto approach_pose = create_approach_pose(place_pose, approach_height);
  return move_to_pose(
    approach_pose, "Approaching place position", 0.5f, 0.6f, feedback, goal_handle);
}

bool PickPlaceActionServer::descend_to_place(
  const geometry_msgs::msg::PoseStamped & place_pose, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  return move_to_pose(
    place_pose, "Descending to place position", 0.6f, 0.7f, feedback, goal_handle);
}

bool PickPlaceActionServer::open_gripper(
  double open_position, double force, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  feedback->stage = "Opening gripper";
  feedback->progress = feedback->progress > 0.5f ? 0.75f : 0.05f;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback->current_pose = current_pose_;
  }
  goal_handle->publish_feedback(feedback);

  // Send gripper command
  auto gripper_cmd = control_msgs::msg::GripperCommand();
  gripper_cmd.position = open_position;
  gripper_cmd.max_effort = force;
  gripper_cmd_pub_->publish(gripper_cmd);

  // Wait for gripper to settle
  wait_for_gripper_settle(gripper_settle_time_);

  feedback->progress = feedback->progress > 0.7f ? 0.8f : 0.1f;
  goal_handle->publish_feedback(feedback);
  return true;
}

bool PickPlaceActionServer::retreat_from_place(
  const geometry_msgs::msg::PoseStamped & place_pose, double approach_height,
  std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  auto retreat_pose = create_approach_pose(place_pose, approach_height);
  return move_to_pose(
    retreat_pose, "Retreating from place position", 0.8f, 0.9f, feedback, goal_handle);
}

bool PickPlaceActionServer::move_to_pose(
  const geometry_msgs::msg::PoseStamped & target_pose, const std::string & stage_name,
  float progress_start, float progress_end, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "%s", stage_name.c_str());

  // Publish the target pose
  arm_cmd_pub_->publish(target_pose);

  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    // Check for cancellation
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal canceled during %s", stage_name.c_str());
      return false;
    }

    // Update feedback
    feedback->stage = stage_name;
    feedback->progress = progress_start + (progress_end - progress_start) * 0.5f;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      feedback->current_pose = current_pose_;
    }
    goal_handle->publish_feedback(feedback);

    // Check if pose reached
    bool pose_reached = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pose_reached =
        is_pose_reached(target_pose, current_pose_, position_tolerance_, orientation_tolerance_);
    }

    if (pose_reached) {
      feedback->progress = progress_end;
      goal_handle->publish_feedback(feedback);
      return true;
    }

    // Check timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration<double>(elapsed).count() > move_timeout_) {
      RCLCPP_ERROR(this->get_logger(), "Timeout during %s", stage_name.c_str());
      return false;
    }

    // Sleep briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;
}

void PickPlaceActionServer::wait_for_gripper_settle(double settle_time_seconds)
{
  // Simple settling delay to allow gripper to complete movement
  // This provides time for the gripper hardware to reach the target position
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<int>(settle_time_seconds * 1000)));
}

bool PickPlaceActionServer::is_pose_reached(
  const geometry_msgs::msg::PoseStamped & target, const geometry_msgs::msg::PoseStamped & current,
  double position_tolerance, double orientation_tolerance)
{
  // Check position
  double dx = target.pose.position.x - current.pose.position.x;
  double dy = target.pose.position.y - current.pose.position.y;
  double dz = target.pose.position.z - current.pose.position.z;
  double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (position_error > position_tolerance) {
    return false;
  }

  // Check orientation
  tf2::Quaternion target_q(
    target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z,
    target.pose.orientation.w);

  tf2::Quaternion current_q(
    current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z,
    current.pose.orientation.w);

  double angle = target_q.angleShortestPath(current_q);

  return angle <= orientation_tolerance;
}

void PickPlaceActionServer::abort_with_message(
  const std::string & message, std::shared_ptr<PickPlace::Result> result,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  result->success = false;
  result->message = message;
  goal_handle->abort(result);
}

geometry_msgs::msg::PoseStamped PickPlaceActionServer::create_approach_pose(
  const geometry_msgs::msg::PoseStamped & target_pose, double approach_height)
{
  auto approach_pose = target_pose;
  approach_pose.pose.position.z += approach_height;
  return approach_pose;
}

void PickPlaceActionServer::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_pose_ = *msg;

  // Set home position from first received pose if not yet initialized
  if (!home_pose_initialized_) {
    home_pose_ = *msg;
    home_pose_initialized_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Home position set from current pose: [%.3f, %.3f, %.3f]",
      home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
  }
}

void PickPlaceActionServer::set_arm_speed(double speed)
{
  // Create DynamicInterfaceGroupValues message following the pattern from launch file
  auto msg = control_msgs::msg::DynamicInterfaceGroupValues();
  msg.interface_groups.push_back("arm_motion_mode");

  control_msgs::msg::InterfaceValue interface_value;
  interface_value.interface_names.push_back("speed");
  interface_value.values.push_back(speed);
  msg.interface_values.push_back(interface_value);

  // Publish multiple times for reliability (especially for late-joining subscribers)
  // The TRANSIENT_LOCAL QoS ensures the last message is retained
  for (int i = 0; i < 3; ++i) {
    speed_pub_->publish(msg);
  }

  // Allow settling time for message propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  RCLCPP_INFO(this->get_logger(), "Set arm speed to %.1f%%", speed);
}

bool PickPlaceActionServer::move_to_home()
{
  RCLCPP_INFO(this->get_logger(), "Moving to home position");

  // Update timestamp
  home_pose_.header.stamp = this->now();

  // Move arm to home position
  arm_cmd_pub_->publish(home_pose_);

  // Open gripper to home position
  auto gripper_cmd = control_msgs::msg::GripperCommand();
  gripper_cmd.position = home_gripper_position_;
  gripper_cmd.max_effort = 1.0;
  gripper_cmd_pub_->publish(gripper_cmd);

  // Wait for arm to reach home position
  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    // Check if pose reached
    bool pose_reached = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pose_reached =
        is_pose_reached(home_pose_, current_pose_, position_tolerance_, orientation_tolerance_);
    }

    if (pose_reached) {
      return true;
    }

    // Check timeout (use longer timeout for home movement)
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration<double>(elapsed).count() > 5.0) {  // 5 second timeout for home
      RCLCPP_ERROR(this->get_logger(), "Timeout moving to home position");
      return false;
    }

    // Sleep briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;
}

}  // namespace arm_hand_control

// main function for standalone executable
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_hand_control::PickPlaceActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
