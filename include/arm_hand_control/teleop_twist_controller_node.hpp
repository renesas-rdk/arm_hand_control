#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>

namespace arm_hand_control
{

// Control mode constants
static const std::string CONTROL_MODE_CARTESIAN = "cartesian_mode";
static const std::string CONTROL_MODE_JOINT = "joint_mode";

class TeleopTwistControllerNode : public rclcpp::Node
{
public:
  explicit TeleopTwistControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Callbacks
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter>& parameters);

  // Helper methods
  void set_control_mode(const std::string& mode);
  void publish_pose_command(const geometry_msgs::msg::PoseStamped& pose);
  void publish_joint_command(const sensor_msgs::msg::JointState& joint_state);
  void load_joint_config(const std::string& config_file_name);

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_pub_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // State
  std::string control_mode_;
  double linear_scale_;
  double angular_scale_;
  double joint_vel_scale_;
  geometry_msgs::msg::PoseStamped current_pose_;
  sensor_msgs::msg::JointState current_joints_;
  bool have_pose_;
  bool have_joints_;
};

}  // namespace arm_hand_control
