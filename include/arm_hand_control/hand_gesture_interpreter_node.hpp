#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>

namespace arm_hand_control
{

struct JointConfig
{
  std::string name;
  std::string finger;  // which finger this joint belongs to
  std::string role;    // joint's role (flex, yaw, pitch, etc.)
  double limit_max;
  double default_position;
};

class HandGestureInterpreter : public rclcpp::Node
{
public:
  HandGestureInterpreter();
  virtual ~HandGestureInterpreter() = default;

private:
  // ROS publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Joint state data
  std::vector<std::string> joint_names_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_limits_;

  // Joint classifications
  std::map<std::string, std::map<std::string, std::vector<std::string>>> finger_joints_;  // finger -> role -> joint
                                                                                          // names
  std::map<std::string, JointConfig> joint_configs_;

  // Configuration
  std::string config_file_path_;
  double publish_rate_hz_;

  // Private methods
  void load_configuration();
  void publish_joint_states();
  void gesture_callback(const std_msgs::msg::String::SharedPtr msg);

  // Helper methods
  void set_finger_position(const std::string& finger, const std::string& role, double position);
  void set_finger_positions(const std::string& finger, double position);
  void set_all_fingers_except(const std::vector<std::string>& exceptions, double position);

  // Gesture methods
  void set_joint_position(const std::string& joint_name, double position);
  void reset_joint_positions();
  void grasp(double percentage = 1.0);
  void pinch(double percentage = 1.0);
  void point();
  void thumbs_up();
  void wave(double position);
  void three_finger_grasp(double percentage = 1.0);
  void ok();
  void call_me();
  void peace();
  void rock();
  void count_one();
  void count_two();
  void count_three();
  void count_four();
  void count_five();
  void thumbs_down();
  void fist_bump();
};

}  // namespace arm_hand_control
