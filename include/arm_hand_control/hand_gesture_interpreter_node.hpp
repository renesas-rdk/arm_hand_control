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
#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

// Include the generated action
#include "arm_hand_control/action/execute_gesture.hpp"

namespace arm_hand_control
{

struct JointConfig
{
  std::string name;         // Joint name used in URDF/joint_states
  std::string finger;       // Which finger this joint belongs to (thumb, index, etc.)
  std::string role;         // Joint's role (flex, yaw, pitch, etc.)
  double limit_max;         // Maximum joint limit (radians)
  double velocity_limit;   // Maximum joint velocity limits
  double default_position;  // Default/rest position (radians)
};

/**
 * Hand gesture interpreter node
 *
 * This node interprets gesture commands and translates them into joint positions
 * for controlling a robotic hand. It supports predefined gestures, transitions
 * between gestures, and a demo mode that cycles through available gestures.
 *
 * Topics:
 * - Subscriptions:
 *   - hand_gesture (std_msgs/String):
 *     Receives gesture command strings (e.g., "grasp", "pinch", "point")
 *   - hand_landmarks (geometry_msgs/PoseArray):
 *     Receives hand landmark positions to detect user activity
 *
 * - Publications:
 *   - joint_states (sensor_msgs/JointState):
 *     Publishes joint positions for the robotic hand
 *
 * - Actions:
 *   - execute_gesture (arm_hand_control/action/ExecuteGesture):
 *     Provides smooth transitions between gestures with progress feedback
 *
 * Parameters:
 * - config_file (string): Path to the hand configuration YAML file
 * - auto_demo_enabled (bool): Whether to automatically cycle through gestures
 * - gesture_duration (double): Duration to hold each gesture in demo mode
 * - transition_duration (double): Duration for transitions between gestures
 */
class HandGestureInterpreter : public rclcpp::Node
{
public:
  HandGestureInterpreter();
 ~HandGestureInterpreter();

private:
  //===== Action Server Type Definitions =====
  using ExecuteGesture = arm_hand_control::action::ExecuteGesture;
  using GoalHandleExecuteGesture = rclcpp_action::ServerGoalHandle<ExecuteGesture>;

  //===== ROS Communication =====
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmarks_subscriber_;
  rclcpp::Subscription<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_subscriber_;
  rclcpp_action::Server<ExecuteGesture>::SharedPtr action_server_;

  //===== Joint State Data =====
  std::vector<JointConfig> joint_order_;
  std::vector<std::string> joint_names_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_limits_;
  std::map<std::string, double> joint_velocity_limits_;

  //===== Gesture Transition Data =====
  std::map<std::string, double> target_positions_;
  std::map<std::string, double> start_positions_;
  bool transition_in_progress_ = false;
  rclcpp::TimerBase::SharedPtr transition_timer_;
  double transition_progress_ = 0.0;
  double transition_duration_ = 1.0;
  std::shared_ptr<GoalHandleExecuteGesture> current_goal_handle_;

  //===== Joint Classification =====
  std::map<std::string, std::map<std::string, std::vector<std::string>>>
    finger_joints_;  // Maps finger->role->joints
  std::map<std::string, JointConfig> joint_configs_;

  //===== Configuration Parameters =====
  std::string config_file_path_;  // Path to the YAML configuration file
  bool auto_demo_enabled_;        // Whether to auto loop through gestures
  double gesture_duration_;       // How long to hold each gesture in seconds
  double gripper_max_range_;      // Maximum gripper opening range in meters
  double hand_speed_;

  //===== Demo Mode Related =====
  rclcpp::TimerBase::SharedPtr demo_timer_;
  size_t current_gesture_idx_ = 0;
  bool demo_gesture_in_progress_ = false;

  //===== Hand Landmarks Tracking =====
  rclcpp::TimerBase::SharedPtr landmarks_activity_timer_;
  bool hand_landmarks_received_ = false;
  std::chrono::time_point<std::chrono::steady_clock> last_landmarks_time_;
  const std::chrono::seconds landmarks_timeout_{5};  // 5 seconds timeout
  bool landmarks_demo_mode_stopped_ = false;

  //===== Action Server Methods =====
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteGesture::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteGesture> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExecuteGesture> goal_handle);
  void execute_gesture_action(const std::shared_ptr<GoalHandleExecuteGesture> goal_handle);

  //===== Core Methods =====
  void load_configuration();
  std::vector<double> get_ordered_positions() const;
  std_msgs::msg::Float64MultiArray build_position_msg(const std::vector<double> & data);
  void publish_joint_states();
  void gesture_callback(const std_msgs::msg::String::SharedPtr msg);
  void gripper_command_callback(const control_msgs::msg::GripperCommand::SharedPtr msg);
  void execute_gesture(
    const std::string & gesture, double duration = -1.0, double percentage = -1.0);
  void prepare_gesture_transition(const std::string & gesture, double percentage = -1.0);
  double calculate_estimated_duration();

  //===== Demo Mode Methods =====
  void start_demo_mode();
  void stop_demo_mode();
  void demo_timer_callback();
  std::vector<std::string> get_all_available_gestures();

  //===== Hand Landmarks Methods =====
  void landmarks_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void check_landmarks_activity();

  //===== Joint Control Methods =====
  void set_joint_position(const std::string & joint_name, double position);
  void reset_joint_positions();

  //===== Finger Abstraction Methods =====
  void set_finger_position(const std::string & finger, const std::string & role, double percentage);
  void set_finger_positions(const std::string & finger, double percentage);
  void set_all_fingers_except(const std::vector<std::string> & exceptions, double percentage);

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
  void finger_cross();
  void italian_hand();

  // Debug gestures
  void debug_finger(const std::string & finger, const std::string & role, double percentage);
  void parse_and_execute_debug_gesture(const std::string & gesture_command);
};

}  // namespace arm_hand_control
