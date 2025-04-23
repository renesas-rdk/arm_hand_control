#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>

#include "agilex_piper_controller/piper_controller.hpp"

namespace arm_hand_control
{

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

  // Joint configuration
  std::map<std::string, JointConfig> joint_config_;

  // Controller instance
  std::unique_ptr<agilex::piper::PiperController> controller_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Joint names
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;

  // Callbacks
  void update_callback();
  void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void pose_command_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void control_mode_callback(const std_msgs::msg::String::SharedPtr msg);

  // Configuration methods
  bool load_joint_config(const std::string& config_file);
  void apply_joint_limits_to_sdk();

  // Utility methods
  void publish_joint_states();
  void publish_end_pose();
  void publish_arm_status();

  // Helper methods for better organization
  void declare_and_get_parameters();
  std::string resolve_config_file_path(const std::string& config_file);
  void initialize_joint_data();
  void setup_publishers_and_subscribers();
  void initialize_controller();
  void load_and_apply_configuration(const std::string& config_file_path);
};

}  // namespace arm_hand_control