#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>

namespace arm_hand_control
{

class PiperTeleopNode : public rclcpp::Node
{
public:
  // Control mode constants
  static constexpr const char* CONTROL_MODE_JOINT = "joint_mode";
  static constexpr const char* CONTROL_MODE_CARTESIAN = "cartesian_mode";
  static constexpr const char* CONTROL_MODE_ENABLE = "enable";
  static constexpr const char* CONTROL_MODE_DISABLE = "disable";

  PiperTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PiperTeleopNode() = default;

private:
  // Parameters
  std::string control_mode_;
  double linear_scale_;
  double angular_scale_;
  double joint_vel_scale_;

  // Current state
  geometry_msgs::msg::Pose current_pose_;
  sensor_msgs::msg::JointState current_joints_;
  bool have_pose_;
  bool have_joints_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_pub_;

  // Callbacks
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Helper methods
  void set_control_mode(const std::string& mode);
  void publish_pose_command(const geometry_msgs::msg::Pose& pose);
  void publish_joint_command(const sensor_msgs::msg::JointState& joint_state);
};

}  // namespace arm_hand_control
