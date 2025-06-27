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
  // Declare parameters
  this->declare_parameter("position_tolerance", 0.005);    // 5mm
  this->declare_parameter("orientation_tolerance", 0.05);  // ~3 degrees
  this->declare_parameter("move_timeout", 2.0);            // 2 seconds
  this->declare_parameter("gripper_timeout", 1.0);         // 1 second
  this->declare_parameter("gripper_settle_time", 0.5);     // 1 second

  // Get parameters
  position_tolerance_ = this->get_parameter("position_tolerance").as_double();
  orientation_tolerance_ = this->get_parameter("orientation_tolerance").as_double();
  move_timeout_ = this->get_parameter("move_timeout").as_double();
  gripper_timeout_ = this->get_parameter("gripper_timeout").as_double();
  gripper_settle_time_ = this->get_parameter("gripper_settle_time").as_double();

  // Create publishers
  arm_cmd_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/arm/pose_command", 10);
  gripper_cmd_pub_ =
    this->create_publisher<control_msgs::msg::GripperCommand>("/arm/gripper_command", 10);

  // Create service client for speed control
  high_speed_client_ = this->create_client<std_srvs::srv::SetBool>("/arm/set_high_speed");

  // Create subscribers
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/arm/current_pose", 10,
    std::bind(&PickPlaceActionServer::pose_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/arm/joint_states", 10,
    std::bind(&PickPlaceActionServer::joint_state_callback, this, std::placeholders::_1));

  status_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/arm/status", 10,
    std::bind(&PickPlaceActionServer::status_callback, this, std::placeholders::_1));

  // Create action server
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<PickPlace>(
    this, "pick_place", std::bind(&PickPlaceActionServer::handle_goal, this, _1, _2),
    std::bind(&PickPlaceActionServer::handle_cancel, this, _1),
    std::bind(&PickPlaceActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "Pick-Place Action Server initialized");
  RCLCPP_INFO(this->get_logger(), "Position tolerance: %.3f m", position_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Orientation tolerance: %.3f rad", orientation_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Move timeout: %.1f s", move_timeout_);
  RCLCPP_INFO(this->get_logger(), "Gripper timeout: %.1f s", gripper_timeout_);
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
  gripper_cmd.position = 0.06;  // Default open position
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
  if (!set_arm_high_speed(true)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set initial arm speed");
  }

  // Stage 1: Approach pick position
  if (!approach_pick(goal->pick_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to approach pick position", result, goal_handle);
    return;
  }

  // Set low speed for descending (10%)
  if (!set_arm_high_speed(false)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set low speed for descending");
  }

  // Stage 2: Descend to pick position
  if (!descend_to_pick(goal->pick_pose, feedback, goal_handle)) {
    abort_with_message("Failed to descend to pick position", result, goal_handle);
    return;
  }

  // Stage 3: Close gripper
  if (!close_gripper(goal->gripper_closed_position, goal->gripper_force, feedback, goal_handle)) {
    abort_with_message("Failed to close gripper", result, goal_handle);
    return;
  }

  // Stage 4: Lift object
  if (!lift_object(goal->pick_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to lift object", result, goal_handle);
    return;
  }

  // Set high speed for transit (100%)
  if (!set_arm_high_speed(true)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set high speed for transit");
  }

  // Stage 5: Approach place position
  if (!approach_place(goal->place_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to approach place position", result, goal_handle);
    return;
  }

  // Set low speed for descending (10%)
  if (!set_arm_high_speed(false)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set low speed for descending");
  }

  // Stage 6: Descend to place position
  if (!descend_to_place(goal->place_pose, feedback, goal_handle)) {
    abort_with_message("Failed to descend to place position", result, goal_handle);
    return;
  }

  // Stage 7: Open gripper
  if (!open_gripper(goal->gripper_open_position, goal->gripper_force, feedback, goal_handle)) {
    abort_with_message("Failed to open gripper", result, goal_handle);
    return;
  }

  // Stage 8: Retreat from place position
  if (!retreat_from_place(goal->place_pose, goal->approach_height, feedback, goal_handle)) {
    abort_with_message("Failed to retreat from place position", result, goal_handle);
    return;
  }

  // Reset to high speed (100%)
  if (!set_arm_high_speed(true)) {
    RCLCPP_WARN(this->get_logger(), "Failed to reset arm speed");
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
    approach_pose, "Approaching pick position", 0.0f, 0.125f, feedback, goal_handle);
}

bool PickPlaceActionServer::descend_to_pick(
  const geometry_msgs::msg::PoseStamped & pick_pose, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  return move_to_pose(
    pick_pose, "Descending to pick position", 0.125f, 0.25f, feedback, goal_handle);
}

bool PickPlaceActionServer::close_gripper(
  double closed_position, double force, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  feedback->stage = "Closing gripper";
  feedback->progress = 0.3f;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback->current_pose = current_pose_;
  }
  goal_handle->publish_feedback(feedback);

  // Send gripper command
  auto gripper_cmd = control_msgs::msg::GripperCommand();
  gripper_cmd.position = closed_position;
  gripper_cmd.max_effort = force;

  gripper_command_sent_ = true;
  last_gripper_command_time_ = std::chrono::steady_clock::now();
  gripper_cmd_pub_->publish(gripper_cmd);

  // Wait for gripper to settle
  if (!wait_for_gripper_command(gripper_settle_time_)) {
    RCLCPP_ERROR(this->get_logger(), "Gripper close timeout");
    return false;
  }

  feedback->progress = 0.375f;
  goal_handle->publish_feedback(feedback);
  return true;
}

bool PickPlaceActionServer::lift_object(
  const geometry_msgs::msg::PoseStamped & pick_pose, double approach_height,
  std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  auto lift_pose = create_approach_pose(pick_pose, approach_height);
  return move_to_pose(lift_pose, "Lifting object", 0.375f, 0.5f, feedback, goal_handle);
}

bool PickPlaceActionServer::approach_place(
  const geometry_msgs::msg::PoseStamped & place_pose, double approach_height,
  std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  auto approach_pose = create_approach_pose(place_pose, approach_height);
  return move_to_pose(
    approach_pose, "Approaching place position", 0.5f, 0.625f, feedback, goal_handle);
}

bool PickPlaceActionServer::descend_to_place(
  const geometry_msgs::msg::PoseStamped & place_pose, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  return move_to_pose(
    place_pose, "Descending to place position", 0.625f, 0.75f, feedback, goal_handle);
}

bool PickPlaceActionServer::open_gripper(
  double open_position, double force, std::shared_ptr<PickPlace::Feedback> feedback,
  const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
{
  feedback->stage = "Opening gripper";
  feedback->progress = 0.8f;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback->current_pose = current_pose_;
  }
  goal_handle->publish_feedback(feedback);

  // Send gripper command
  auto gripper_cmd = control_msgs::msg::GripperCommand();
  gripper_cmd.position = open_position;
  gripper_cmd.max_effort = force;

  gripper_command_sent_ = true;
  last_gripper_command_time_ = std::chrono::steady_clock::now();
  gripper_cmd_pub_->publish(gripper_cmd);

  // Wait for gripper to settle
  if (!wait_for_gripper_command(gripper_settle_time_)) {
    RCLCPP_ERROR(this->get_logger(), "Gripper open timeout");
    return false;
  }

  feedback->progress = 0.875f;
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
    retreat_pose, "Retreating from place position", 0.875f, 1.0f, feedback, goal_handle);
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

bool PickPlaceActionServer::wait_for_gripper_command(double timeout_seconds)
{
  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration<double>(elapsed).count() >= timeout_seconds) {
      gripper_command_sent_ = false;
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return false;
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
}

void PickPlaceActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_joint_state_ = *msg;
}

void PickPlaceActionServer::status_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_status_ = msg->data;
}

bool PickPlaceActionServer::set_arm_high_speed(bool high_speed)
{
  if (!high_speed_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Speed service not available");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = high_speed;

  // Use async call with callback instead of blocking
  auto result_future = high_speed_client_->async_send_request(request);

  // Wait for the result without spinning
  auto status = result_future.wait_for(std::chrono::seconds(1));

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Speed service call timed out");
    return false;
  }

  auto response = result_future.get();
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Speed service returned failure: %s", response->message.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "Set arm to %s", high_speed ? "high speed (100%)" : "low speed (10%)");
  return true;
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
