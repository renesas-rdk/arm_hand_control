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
// Standard includes
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// ROS2 includes
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// Project includes
#include "arm_hand_control/teleop_twist_controller_node.hpp"

namespace arm_hand_control
{

TeleopTwistControllerNode::TeleopTwistControllerNode(const rclcpp::NodeOptions & options)
: Node("teleop_twist_controller_node", options), have_pose_(false), have_joints_(false)
{
  // Declare and get parameters
  this->declare_parameter("linear_scale", 0.01);
  this->declare_parameter("angular_scale", 0.01);
  this->declare_parameter("joint_vel_scale", 0.01);
  this->declare_parameter(
    "joint_names",
    std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});

  // Get parameter values
  linear_scale_ = this->get_parameter("linear_scale").as_double();
  angular_scale_ = this->get_parameter("angular_scale").as_double();
  joint_vel_scale_ = this->get_parameter("joint_vel_scale").as_double();
  joint_names_ = this->get_parameter("joint_names").as_string_array();

  // Set up parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TeleopTwistControllerNode::parameter_callback, this, std::placeholders::_1));

  // Create subscribers with appropriate QoS settings
  auto default_qos = rclcpp::QoS(10);
  auto sensor_qos = rclcpp::SensorDataQoS();

  pose_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "pose/cmd_vel", default_qos,
    std::bind(&TeleopTwistControllerNode::pose_twist_callback, this, std::placeholders::_1));

  joint_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "joint/cmd_vel", default_qos,
    std::bind(&TeleopTwistControllerNode::joint_twist_callback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "current_pose", default_qos,
    std::bind(&TeleopTwistControllerNode::pose_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", sensor_qos,
    std::bind(&TeleopTwistControllerNode::joint_state_callback, this, std::placeholders::_1));

  // Create publishers with appropriate QoS settings
  pose_cmd_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_command", default_qos);
  joint_cmd_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_command", default_qos);

  // Log initialization information
  RCLCPP_INFO(this->get_logger(), "Teleop twist controller initialized");
  RCLCPP_INFO(
    this->get_logger(), "Scales - Linear: %.3f, Angular: %.3f, Joint: %.3f", linear_scale_,
    angular_scale_, joint_vel_scale_);
  RCLCPP_INFO(this->get_logger(), "Configured for %zu joints", joint_names_.size());
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
  auto new_pose = current_pose_;

  // Apply linear velocities with scaling
  new_pose.pose.position.x += msg->linear.x * linear_scale_;
  new_pose.pose.position.y += msg->linear.y * linear_scale_;
  new_pose.pose.position.z += msg->linear.z * linear_scale_;

  // Convert current orientation to RPY for angular updates
  tf2::Quaternion q(
    current_pose_.pose.orientation.x, current_pose_.pose.orientation.y,
    current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Apply angular velocities with scaling
  roll += msg->angular.x * angular_scale_;
  pitch += msg->angular.y * angular_scale_;
  yaw += msg->angular.z * angular_scale_;

  // Convert back to quaternion
  q.setRPY(roll, pitch, yaw);
  new_pose.pose.orientation.x = q.x();
  new_pose.pose.orientation.y = q.y();
  new_pose.pose.orientation.z = q.z();
  new_pose.pose.orientation.w = q.w();

  // Update timestamp and ensure valid frame_id
  new_pose.header.stamp = this->now();
  if (new_pose.header.frame_id.empty()) {
    new_pose.header.frame_id = "base_link";
  }

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

  // Check if we have sufficient joint information
  if (current_joints_.position.size() < joint_names_.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Insufficient joint information available, need at least %zu joints but have %zu",
      joint_names_.size(), current_joints_.position.size());
    return;
  }

  // Create modified joint state
  auto new_joints = current_joints_;

  // Map twist components to joint changes with boundary checks
  const double twist_values[] = {
    msg->angular.z,  // Joint 0: angular.z
    msg->linear.z,   // Joint 1: linear.z
    msg->linear.y,   // Joint 2: linear.y
    msg->linear.x,   // Joint 3: linear.x
    msg->angular.y,  // Joint 4: angular.y
    msg->angular.x   // Joint 5: angular.x
  };

  const size_t num_joints = std::min(joint_names_.size(), new_joints.position.size());
  const size_t max_twist_values = sizeof(twist_values) / sizeof(twist_values[0]);

  for (size_t i = 0; i < num_joints && i < max_twist_values; ++i) {
    new_joints.position[i] += twist_values[i] * joint_vel_scale_;
  }

  publish_joint_command(new_joints);
}

void TeleopTwistControllerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  if (!have_pose_) {
    have_pose_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received first pose feedback");
  }
}

void TeleopTwistControllerNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joints_ = filter_joint_state(*msg);
  if (!have_joints_) {
    have_joints_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received first joint state feedback");
  }
}

sensor_msgs::msg::JointState TeleopTwistControllerNode::filter_joint_state(
  const sensor_msgs::msg::JointState & joint_state) const
{
  sensor_msgs::msg::JointState filtered_state;
  filtered_state.header = joint_state.header;
  filtered_state.name = joint_names_;

  // Pre-allocate vectors with the correct size
  const size_t num_joints = joint_names_.size();
  filtered_state.position.reserve(num_joints);
  filtered_state.velocity.reserve(num_joints);
  filtered_state.effort.reserve(num_joints);

  filtered_state.position.resize(num_joints, 0.0);
  filtered_state.velocity.resize(num_joints, 0.0);
  filtered_state.effort.resize(num_joints, 0.0);

  // Create a map for faster lookup of joint indices
  std::unordered_map<std::string, size_t> joint_index_map;
  joint_index_map.reserve(joint_state.name.size());
  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    joint_index_map[joint_state.name[i]] = i;
  }

  // Map joint data based on joint_names parameter
  for (size_t i = 0; i < num_joints; ++i) {
    const auto & target_joint = joint_names_[i];

    auto it = joint_index_map.find(target_joint);
    if (it != joint_index_map.end()) {
      const size_t source_index = it->second;

      // Copy data if available with bounds checking
      if (source_index < joint_state.position.size()) {
        filtered_state.position[i] = joint_state.position[source_index];
      }
      if (source_index < joint_state.velocity.size()) {
        filtered_state.velocity[i] = joint_state.velocity[source_index];
      }
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
  // Create message directly without unnecessary copy
  geometry_msgs::msg::PoseStamped msg = pose;

  // Ensure the message has a valid timestamp and frame id
  if (msg.header.stamp == rclcpp::Time(0)) {
    msg.header.stamp = this->now();
  }
  if (msg.header.frame_id.empty()) {
    msg.header.frame_id = "base_link";
  }

  pose_cmd_pub_->publish(msg);
}

void TeleopTwistControllerNode::publish_joint_command(
  const sensor_msgs::msg::JointState & joint_state)
{
  // Validate input data
  if (joint_state.position.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Cannot publish joint command: joint positions are empty");
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = joint_state.position;

  joint_cmd_pub_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult TeleopTwistControllerNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();

    if (name == "linear_scale") {
      double value = param.as_double();
      if (value > 0.0 && value <= 1.0) {
        linear_scale_ = value;
        RCLCPP_INFO(this->get_logger(), "Updated linear scale: %.3f", value);
      } else {
        result.successful = false;
        result.reason = "linear_scale must be in range (0.0, 1.0]";
        return result;
      }
    } else if (name == "angular_scale") {
      double value = param.as_double();
      if (value > 0.0 && value <= 1.0) {
        angular_scale_ = value;
        RCLCPP_INFO(this->get_logger(), "Updated angular scale: %.3f", value);
      } else {
        result.successful = false;
        result.reason = "angular_scale must be in range (0.0, 1.0]";
        return result;
      }
    } else if (name == "joint_vel_scale") {
      double value = param.as_double();
      if (value > 0.0 && value <= 1.0) {
        joint_vel_scale_ = value;
        RCLCPP_INFO(this->get_logger(), "Updated joint velocity scale: %.3f", value);
      } else {
        result.successful = false;
        result.reason = "joint_vel_scale must be in range (0.0, 1.0]";
        return result;
      }
    } else if (name == "joint_names") {
      auto value = param.as_string_array();
      if (!value.empty()) {
        joint_names_ = value;
        have_joints_ = false;  // Reset since joint configuration changed
        RCLCPP_INFO(this->get_logger(), "Updated joint names (%zu joints)", value.size());
      } else {
        result.successful = false;
        result.reason = "joint_names cannot be empty";
        return result;
      }
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
