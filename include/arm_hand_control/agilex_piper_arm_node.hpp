#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>

#include "agilex_piper_controller/piper_controller.hpp"

namespace arm_hand_control
{

/**
 * AgilexPiperArmNode provides ROS 2 interface for the AgileX Piper robotic arm
 *
 * This node creates an interface between ROS 2 and the AgileX Piper robotic arm,
 * allowing for control via joint positions or Cartesian poses.
 *
 * Subscribed topics:
 *   - piper/joint_command (trajectory_msgs/JointTrajectory): Joint position commands
 *   - piper/pose_command (geometry_msgs/PoseStamped): End effector pose commands
 *
 * Published topics:
 *   - joint_states (sensor_msgs/JointState): Current joint positions
 *   - piper/current_pose (geometry_msgs/PoseStamped): Current end effector pose
 *   - piper/status (std_msgs/String): Current arm status
 *
 * Parameters:
 *   - can_interface (string): CAN interface to use (default: "can0")
 *   - update_frequency (double): Update frequency in Hz (default: 50.0)
 *   - config_file (string): Path to configuration file
 *   - arm_enabled (bool): Whether the arm is enabled (default: false)
 *   - control_mode (int): Control mode (0=Cartesian, 1=Joint, default: 1)
 *   - listen_only (bool): When true, commands are received but not executed (default: false)
 */
class AgilexPiperArmNode : public rclcpp::Node
{
public:
  AgilexPiperArmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AgilexPiperArmNode();

private:
  // Joint configuration struct
  struct JointConfig
  {
    std::string name;
    std::string role;
    double limit_min;
    double limit_max;
    double default_position;
  };

  // ROS parameters
  std::string can_interface_;
  double update_frequency_;

  // Control mode parameters
  bool arm_enabled_;
  int control_mode_;  // 0=Cartesian, 1=Joint
  bool listen_only_;  // If true, no commands will be executed

  // OnSetParametersCallbackHandle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Joint configuration
  std::map<std::string, JointConfig> joint_config_;

  // Controller instance
  std::unique_ptr<agilex::piper::PiperController> controller_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Subscribers
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Joint names and positions
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;

  // Callback methods
  void update_callback();
  void joint_command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters);

  // Configuration and utility methods
  std::string resolve_config_file_path(const std::string& config_file);
  bool load_joint_config(const std::string& config_file);
  void apply_joint_limits_to_sdk();
  void apply_control_mode();

  // Publishing methods
  void publish_joint_states();
  void publish_end_pose();
  void publish_arm_status();
};

}  // namespace arm_hand_control