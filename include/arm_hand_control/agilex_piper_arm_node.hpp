#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/msg/gripper_command.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "agilex_piper_controller/piper_controller.hpp"

namespace arm_hand_control
{

// ROS 2 interface node for the AgileX Piper robotic arm
//
// This node provides a comprehensive ROS 2 interface for controlling the AgileX Piper
// robotic arm and gripper, supporting both joint space and Cartesian space control.
//
// Subscribed Topics:
// - piper/joint_command (trajectory_msgs/JointTrajectory): Joint position commands
// - piper/pose_command (geometry_msgs/PoseStamped): End effector pose commands
// - piper/gripper_command (control_msgs/GripperCommand): Gripper position commands
//
// Published Topics:
// - piper/joint_states (sensor_msgs/JointState): Current joint positions
// - piper/gripper_joint_states (sensor_msgs/JointState): Current gripper joint states
// - piper/current_pose (geometry_msgs/PoseStamped): Current end effector pose
// - piper/status (std_msgs/UInt8MultiArray): Current arm status
//
// Parameters:
// - can_interface (string): CAN interface to use (default: "can0")
// - update_frequency (double): Update frequency in Hz (default: 50.0)
// - config_file (string): Path to configuration file
// - arm_enabled (bool): Whether the arm is enabled (default: false)
// - motion_mode (int): Motion mode (0=Cartesian, 1=Joint, default: 1)
// - listen_only (bool): When true, commands are received but not executed (default: false)
class AgilexPiperArmNode : public rclcpp::Node
{
public:
  explicit AgilexPiperArmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AgilexPiperArmNode();

  AgilexPiperArmNode(const AgilexPiperArmNode&) = delete;
  AgilexPiperArmNode& operator=(const AgilexPiperArmNode&) = delete;

private:
  // Constants
  static constexpr size_t NUM_JOINTS = 6;
  static constexpr uint16_t DEFAULT_GRIPPER_EFFORT = 1000; // in 0.001 newtons
  static constexpr uint8_t GRIPPER_ENABLE = 0x01;
  static constexpr uint8_t GRIPPER_DISABLE_CLEAR = 0x02;

  // Joint configuration structure
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
  bool arm_enabled_;
  int motion_mode_;   // 0=Cartesian, 1=Joint
  bool listen_only_;  // If true, no commands will be executed

  // Configuration
  std::map<std::string, JointConfig> joint_config_;

  // Controller instance
  std::unique_ptr<agilex::piper::PiperController> controller_;

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr status_pub_;

  // ROS subscribers
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<control_msgs::msg::GripperCommand>::SharedPtr gripper_cmd_sub_;

  // Timers and callbacks
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Joint state data
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;

  // Callback methods
  void update_callback();
  void joint_command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gripper_command_callback(const control_msgs::msg::GripperCommand::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
      const std::vector<rclcpp::Parameter>& parameters);

  // Configuration and utility methods
  std::string resolve_config_file_path(const std::string& config_file);
  bool load_joint_config(const std::string& config_file);
  void apply_joint_limits_to_sdk();
  void apply_motion_mode();
  bool setup_controller_connection();
  void enable_arm_and_gripper();
  void disable_arm_and_gripper();

  // Publishing methods
  void publish_joint_states();
  void publish_gripper_joint_states();
  void publish_end_pose();
  void publish_arm_status();

  // Validation methods
  bool validate_joint_command(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) const;
  bool is_controller_ready() const;
};

}  // namespace arm_hand_control