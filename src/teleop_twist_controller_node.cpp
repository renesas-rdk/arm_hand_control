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
// Standard includes
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// ROS2 includes
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Project includes
#include "arm_hand_control/teleop_twist_controller_node.hpp"

namespace arm_hand_control
{

TeleopTwistControllerNode::TeleopTwistControllerNode(const rclcpp::NodeOptions & options)
: Node("teleop_twist_controller_node", options), have_pose_(false), have_joints_(false)
{
  // Declare and get parameters
  this->declare_parameter("linear_scale", 0.01);     // m per unit twist
  this->declare_parameter("angular_scale", 0.01);    // rad per unit twist
  this->declare_parameter("joint_vel_scale", 0.01);  // rad per unit twist for joints
  this->declare_parameter(
    "joint_names",
    std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});

  linear_scale_ = this->get_parameter("linear_scale").as_double();
  angular_scale_ = this->get_parameter("angular_scale").as_double();
  joint_vel_scale_ = this->get_parameter("joint_vel_scale").as_double();
  joint_names_ = this->get_parameter("joint_names").as_string_array();

  // Set up parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TeleopTwistControllerNode::parameter_callback, this, std::placeholders::_1));

  // Create subscribers
  pose_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "pose/cmd_vel", 10,
    std::bind(&TeleopTwistControllerNode::pose_twist_callback, this, std::placeholders::_1));

  joint_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "joint/cmd_vel", 10,
    std::bind(&TeleopTwistControllerNode::joint_twist_callback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "current_pose", 10,
    std::bind(&TeleopTwistControllerNode::pose_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&TeleopTwistControllerNode::joint_state_callback, this, std::placeholders::_1));

  // Create publishers
  pose_cmd_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_command", 10);
  joint_cmd_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_command", 10);

  RCLCPP_INFO(this->get_logger(), "Teleop twist controller node initialized");
  RCLCPP_INFO(
    this->get_logger(), "Linear scale: %.2f, Angular scale: %.2f, Joint vel scale: %.2f",
    linear_scale_, angular_scale_, joint_vel_scale_);

  std::string joint_names_str = "";
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_names_str += joint_names_[i];
    if (i < joint_names_.size() - 1) joint_names_str += ", ";
  }
  RCLCPP_INFO(this->get_logger(), "Joint names: [%s]", joint_names_str.c_str());
}

void TeleopTwistControllerNode::pose_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!have_pose_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Cannot process pose twist command: pose feedback not available");
    return;
  }

  // Use current pose as starting point and modify based on twist
  geometry_msgs::msg::PoseStamped new_pose = current_pose_;

  // Apply linear velocities
  new_pose.pose.position.x += msg->linear.x * linear_scale_;
  new_pose.pose.position.y += msg->linear.y * linear_scale_;
  new_pose.pose.position.z += msg->linear.z * linear_scale_;

  // Convert current orientation to RPY
  tf2::Quaternion q(
    current_pose_.pose.orientation.x, current_pose_.pose.orientation.y,
    current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Apply angular velocities
  roll += msg->angular.x * angular_scale_;
  pitch += msg->angular.y * angular_scale_;
  yaw += msg->angular.z * angular_scale_;

  // Convert back to quaternion
  q.setRPY(roll, pitch, yaw);
  new_pose.pose.orientation.x = q.x();
  new_pose.pose.orientation.y = q.y();
  new_pose.pose.orientation.z = q.z();
  new_pose.pose.orientation.w = q.w();

  // Update timestamp and frame_id
  new_pose.header.stamp = this->now();
  new_pose.header.frame_id =
    current_pose_.header.frame_id.empty() ? "base_link" : current_pose_.header.frame_id;

  publish_pose_command(new_pose);
}

void TeleopTwistControllerNode::joint_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!have_joints_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Cannot process joint twist command: joint state feedback not available");
    return;
  }

  // Use twist to directly control joint positions
  sensor_msgs::msg::JointState new_joints = current_joints_;

  // Map twist components to joint changes
  if (new_joints.position.size() >= joint_names_.size()) {
    if (joint_names_.size() >= 1) new_joints.position[0] += msg->angular.z * joint_vel_scale_;
    if (joint_names_.size() >= 2) new_joints.position[1] += msg->linear.z * joint_vel_scale_;
    if (joint_names_.size() >= 3) new_joints.position[2] += msg->linear.y * joint_vel_scale_;
    if (joint_names_.size() >= 4) new_joints.position[3] += msg->linear.x * joint_vel_scale_;
    if (joint_names_.size() >= 5) new_joints.position[4] += msg->angular.y * joint_vel_scale_;
    if (joint_names_.size() >= 6) new_joints.position[5] += msg->angular.x * joint_vel_scale_;

    publish_joint_command(new_joints);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Insufficient joint information available, need at least %zu joints", joint_names_.size());
  }
}

void TeleopTwistControllerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  have_pose_ = true;
}

void TeleopTwistControllerNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joints_ = filter_joint_state(*msg);
  have_joints_ = true;
}

sensor_msgs::msg::JointState TeleopTwistControllerNode::filter_joint_state(
  const sensor_msgs::msg::JointState & joint_state)
{
  sensor_msgs::msg::JointState filtered_state;
  filtered_state.header = joint_state.header;
  filtered_state.name = joint_names_;

  // Initialize vectors with the correct size
  filtered_state.position.resize(joint_names_.size(), 0.0);
  filtered_state.velocity.resize(joint_names_.size(), 0.0);
  filtered_state.effort.resize(joint_names_.size(), 0.0);

  // Map joint data based on joint_names parameter
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string & target_joint = joint_names_[i];

    // Find the joint in the input message
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), target_joint);
    if (it != joint_state.name.end()) {
      size_t source_index = std::distance(joint_state.name.begin(), it);

      // Copy position if available
      if (source_index < joint_state.position.size()) {
        filtered_state.position[i] = joint_state.position[source_index];
      }

      // Copy velocity if available
      if (source_index < joint_state.velocity.size()) {
        filtered_state.velocity[i] = joint_state.velocity[source_index];
      }

      // Copy effort if available
      if (source_index < joint_state.effort.size()) {
        filtered_state.effort[i] = joint_state.effort[source_index];
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Joint '%s' not found in joint state message",
        target_joint.c_str());
    }
  }

  return filtered_state;
}

void TeleopTwistControllerNode::publish_pose_command(const geometry_msgs::msg::PoseStamped & pose)
{
  auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  *msg = pose;

  // Ensure the message has a valid timestamp and frame id
  if (msg->header.stamp == rclcpp::Time(0)) {
    msg->header.stamp = this->now();
  }
  if (msg->header.frame_id.empty()) {
    msg->header.frame_id = "base_link";
  }

  pose_cmd_pub_->publish(std::move(msg));
}

void TeleopTwistControllerNode::publish_joint_command(
  const sensor_msgs::msg::JointState & joint_state)
{
  auto msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
  msg->header.stamp = this->now();
  msg->joint_names = joint_names_;  // Use parameter-defined joint names

  // Create a single trajectory point for the target position
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = joint_state.position;

  // Set velocities to zero if not specified
  if (joint_state.velocity.empty()) {
    point.velocities.resize(joint_state.position.size(), 0.0);
  } else {
    point.velocities = joint_state.velocity;
  }

  // Add time from start and make the response immediate (5ms)
  point.time_from_start = rclcpp::Duration::from_seconds(0.005);

  // Add the point to the trajectory
  msg->points.push_back(point);

  joint_cmd_pub_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult TeleopTwistControllerNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters) {
    if (param.get_name() == "linear_scale") {
      linear_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated linear scale to: %.2f", linear_scale_);
    } else if (param.get_name() == "angular_scale") {
      angular_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated angular scale to: %.2f", angular_scale_);
    } else if (param.get_name() == "joint_vel_scale") {
      joint_vel_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated joint velocity scale to: %.2f", joint_vel_scale_);
    } else if (param.get_name() == "joint_names") {
      joint_names_ = param.as_string_array();

      std::string joint_names_str = "";
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        joint_names_str += joint_names_[i];
        if (i < joint_names_.size() - 1) joint_names_str += ", ";
      }
      RCLCPP_INFO(this->get_logger(), "Updated joint names to: [%s]", joint_names_str.c_str());
    }
  }

  return result;
}

}  // namespace arm_hand_control

// Main entry point for the ROS node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_hand_control::TeleopTwistControllerNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
