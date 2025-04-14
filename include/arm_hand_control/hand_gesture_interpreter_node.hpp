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
  std::string name;         // Joint name used in URDF/joint_states
  std::string finger;       // Which finger this joint belongs to (thumb, index, etc.)
  std::string role;         // Joint's role (flex, yaw, pitch, etc.)
  double limit_max;         // Maximum joint limit (radians)
  double default_position;  // Default/rest position (radians)
};

class HandGestureInterpreter : public rclcpp::Node
{
public:
  HandGestureInterpreter();
  virtual ~HandGestureInterpreter() = default;

private:
  //===== ROS Communication =====
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_subscriber_;

  //===== Joint State Data =====
  std::vector<std::string> joint_names_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_limits_;

  //===== Joint Classification =====
  std::map<std::string, std::map<std::string, std::vector<std::string>>> finger_joints_;  // Maps finger->role->joints
  std::map<std::string, JointConfig> joint_configs_;

  //===== Configuration Parameters =====
  std::string config_file_path_;  // Path to the YAML configuration file
  bool auto_demo_enabled_;        // Whether to auto loop through gestures
  double gesture_duration_;       // How long to hold each gesture in seconds

  //===== Demo Mode Related =====
  rclcpp::TimerBase::SharedPtr demo_timer_;
  size_t current_gesture_idx_;
  bool demo_gesture_in_progress_;

  //===== Core Methods =====
  void load_configuration();
  void publish_joint_states();
  void gesture_callback(const std_msgs::msg::String::SharedPtr msg);

  //===== Demo Mode Methods =====
  void start_demo_mode();
  void stop_demo_mode();
  void demo_timer_callback();
  std::vector<std::string> get_all_available_gestures();

  //===== Joint Control Methods =====
  void set_joint_position(const std::string& joint_name, double position);
  void reset_joint_positions();

  //===== Finger Abstraction Methods =====
  void set_finger_position(const std::string& finger, const std::string& role, double position);
  void set_finger_positions(const std::string& finger, double position);
  void set_all_fingers_except(const std::vector<std::string>& exceptions, double position);

  //===== Gesture Implementation Methods =====
  // Basic hand gestures
  void grasp(double percentage = 1.0);
  void pinch(double percentage = 1.0);
  void three_finger_grasp(double percentage = 1.0);

  // Counting gestures
  void count_one();
  void count_two();
  void count_three();
  void count_four();
  void count_five();

  // Communication gestures
  void point();
  void thumbs_up();
  void thumbs_down();
  void ok();
  void call_me();
  void peace();
  void wave(double position);

  // Fun/special gestures
  void rock();
  void fist_bump();
  void gun();
  void spider_man();
  void hang_loose();
  void finger_cross();
  void italian_hand();
};

}  // namespace arm_hand_control
