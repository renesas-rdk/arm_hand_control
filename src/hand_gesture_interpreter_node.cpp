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
#include "arm_hand_control/hand_gesture_interpreter_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

namespace arm_hand_control
{

HandGestureInterpreter::HandGestureInterpreter() : Node("hand_gesture_interpreter")
{
  // Declare parameters
  this->declare_parameter("config_file", "");
  this->declare_parameter("auto_demo_enabled", true);
  this->declare_parameter("gesture_duration", 1.0);
  this->declare_parameter("transition_duration", 0.8);  // Default transition time for next gesture
  this->declare_parameter("gripper_max_range", 0.04);   // Default max gripper range in meters
  this->declare_parameter("hand_speed", 1500.0);        // Default 1500 mstroke/s

  // Get parameters
  config_file_path_ = this->get_parameter("config_file").as_string();
  auto_demo_enabled_ = this->get_parameter("auto_demo_enabled").as_bool();
  gesture_duration_ = this->get_parameter("gesture_duration").as_double();
  transition_duration_ = this->get_parameter("transition_duration").as_double();
  gripper_max_range_ = this->get_parameter("gripper_max_range").as_double();
  hand_speed_ = this->get_parameter("hand_speed").as_double() * 0.001;

  // Make the path absolute if it's relative
  if (!std::filesystem::path(config_file_path_).is_absolute()) {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_hand_control");
    config_file_path_ = pkg_path + "/" + config_file_path_;
  }

  // Load configuration
  load_configuration();

  // Create publisher for position commands
  auto qos = rclcpp::QoS(1).reliable().durability_volatile();
  position_command_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controller_command", 10);

  // Create subscriber (keep for backward compatibility)
  gesture_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "hand_gesture", qos,
    std::bind(&HandGestureInterpreter::gesture_callback, this, std::placeholders::_1));

  // Create subscriber for gripper commands
  gripper_command_subscriber_ = this->create_subscription<control_msgs::msg::GripperCommand>(
    "gripper_command", qos,
    std::bind(&HandGestureInterpreter::gripper_command_callback, this, std::placeholders::_1));

  // Create subscriber for hand landmarks
  auto landmarks_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  landmarks_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "hand_landmarks", landmarks_qos,
    std::bind(&HandGestureInterpreter::landmarks_callback, this, std::placeholders::_1));

  // Create timer to check hand landmarks activity
  landmarks_activity_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&HandGestureInterpreter::check_landmarks_activity, this));

  // Initialize last landmarks time to now
  last_landmarks_time_ = std::chrono::steady_clock::now();

  // Create action server
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<ExecuteGesture>(
    this, "execute_gesture", std::bind(&HandGestureInterpreter::handle_goal, this, _1, _2),
    std::bind(&HandGestureInterpreter::handle_cancel, this, _1),
    std::bind(&HandGestureInterpreter::handle_accepted, this, _1));

  // Publish initial joint states
  publish_joint_states();

  RCLCPP_INFO(this->get_logger(), "Hand gesture interpreter started");
  RCLCPP_INFO(
    this->get_logger(), "Listening for gestures on topic: %s",
    gesture_subscriber_->get_topic_name());
  RCLCPP_INFO(
    this->get_logger(), "Listening for gripper commands on topic: %s",
    gripper_command_subscriber_->get_topic_name());
  RCLCPP_INFO(
    this->get_logger(), "Listening for hand landmarks on topic: %s",
    landmarks_subscriber_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Gesture action server started: execute_gesture");

  RCLCPP_INFO(this->get_logger(), "Using configuration file: %s", config_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Auto demo enabled: %s", auto_demo_enabled_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Gesture duration: %.2f seconds", gesture_duration_);
  RCLCPP_INFO(this->get_logger(), "Transition duration: %.2f seconds", transition_duration_);
  RCLCPP_INFO(this->get_logger(), "Gripper max range: %.3f meters", gripper_max_range_);
  RCLCPP_INFO(this->get_logger(), "Available gestures: %zu", get_all_available_gestures().size());
  RCLCPP_INFO(
    this->get_logger(), "Will restart demo mode after %ld seconds of no landmarks",
    landmarks_timeout_.count());
  for (const auto & gesture : get_all_available_gestures()) {
    RCLCPP_INFO(this->get_logger(), "  - %s", gesture.c_str());
  }

  // Start demo mode if enabled
  if (auto_demo_enabled_) {
    RCLCPP_INFO(this->get_logger(), "Auto demo mode enabled");
    start_demo_mode();
  }
  else {
    landmarks_demo_mode_stopped_ = true;
    RCLCPP_INFO(this->get_logger(), "Auto demo mode disabled");
  }
}

HandGestureInterpreter::~HandGestureInterpreter()
{
  RCLCPP_INFO(this->get_logger(), "Hand gesture interpreter shutting down");
  landmarks_subscriber_.reset();
  gesture_subscriber_.reset();
  gripper_command_subscriber_.reset();
  position_command_publisher_.reset();
}

//===== ACTION SERVER METHODS =====

rclcpp_action::GoalResponse HandGestureInterpreter::handle_goal(
  const rclcpp_action::GoalUUID & uuid [[maybe_unused]],
  std::shared_ptr<const ExecuteGesture::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(), "Received goal request for gesture: %s", goal->gesture_name.c_str());

  // Check if the requested gesture is valid
  auto available_gestures = get_all_available_gestures();
  if (
    std::find(available_gestures.begin(), available_gestures.end(), goal->gesture_name) ==
    available_gestures.end()) {
    RCLCPP_WARN(this->get_logger(), "Unknown gesture: %s", goal->gesture_name.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Always accept valid gesture goals
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HandGestureInterpreter::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteGesture> goal_handle [[maybe_unused]])
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel gesture");

  // Cancel the transition if one is in progress
  if (transition_timer_) {
    transition_timer_->cancel();
    transition_timer_.reset();
    transition_in_progress_ = false;
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void HandGestureInterpreter::handle_accepted(
  const std::shared_ptr<GoalHandleExecuteGesture> goal_handle)
{
  // Stop any running demo when accepting an action goal
  if (demo_timer_) {
    stop_demo_mode();
  }

  // Execute the goal in a separate thread to avoid blocking the ROS executor
  std::thread{std::bind(&HandGestureInterpreter::execute_gesture_action, this, goal_handle)}
    .detach();
}

void HandGestureInterpreter::execute_gesture_action(
  const std::shared_ptr<GoalHandleExecuteGesture> goal_handle)
{
  // Get the goal
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExecuteGesture::Feedback>();
  auto result = std::make_shared<ExecuteGesture::Result>();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 100,
    "Executing gesture: %s, duration: %.2f seconds, percentage: %.2f", goal->gesture_name.c_str(),
    goal->duration, goal->percentage);

  // Store the goal handle for use in transition callbacks
  current_goal_handle_ = goal_handle;

  // Calculate the estimated duration based on joint movements
  double estimated_duration = calculate_estimated_duration();

  // Determine transition duration
  double duration = goal->duration > 0.0 ? goal->duration : transition_duration_;

  duration = (duration > estimated_duration) ? duration : estimated_duration;

  // Execute the gesture with the requested parameters
  execute_gesture(goal->gesture_name, duration, goal->percentage);

  // Wait for the transition to complete
  while (transition_in_progress_ && rclcpp::ok() && !goal_handle->is_canceling()) {
    // Update feedback
    feedback->percentage_complete = transition_progress_;
    goal_handle->publish_feedback(feedback);

    // Sleep briefly to avoid busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Check if the goal was canceled
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Gesture execution was canceled";
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Gesture execution canceled");
    return;
  }

  // Set the final result
  result->success = true;
  result->message = "Gesture executed successfully";
  goal_handle->succeed(result);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 100, "Gesture execution completed successfully");
}

//===== DEMO MODE METHODS =====

void HandGestureInterpreter::start_demo_mode()
{
  current_gesture_idx_ = 0;
  // Create timer for cycling through gestures
  demo_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(gesture_duration_ * 1000)),
    std::bind(&HandGestureInterpreter::demo_timer_callback, this));

  // Execute first gesture immediately
  demo_timer_callback();
}

void HandGestureInterpreter::stop_demo_mode()
{
  if (demo_timer_) {
    demo_timer_->cancel();
    demo_timer_.reset();
    RCLCPP_INFO(this->get_logger(), "Auto demo mode stopped");
  }
}

std::vector<std::string> HandGestureInterpreter::get_all_available_gestures()
{
  return {
    // Basic hand gestures
    "open_hand", "pinch", "loose_fist", "three_finger_grasp",

    // Counting gestures
    "one", "two", "three", "four", "five",

    // Communication gestures
    "point", "thumbs_up", "ok", "call_me", "peace",

    // Fun/special gestures
    "rock", "fist_bump", "gun", "spider_man", "finger_cross"};
}

void HandGestureInterpreter::demo_timer_callback()
{
  auto gestures = get_all_available_gestures();
  if (gestures.empty()) return;

  std::string current_gesture = gestures[current_gesture_idx_];
  RCLCPP_INFO(this->get_logger(), "Auto demo showing gesture: %s", current_gesture.c_str());

  // Execute the gesture with the default transition duration
  execute_gesture(current_gesture, transition_duration_);

  // Move to next gesture
  current_gesture_idx_ = (current_gesture_idx_ + 1) % gestures.size();
}

//===== CONFIGURATION METHODS =====

void HandGestureInterpreter::load_configuration()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Loading configuration from: %s", config_file_path_.c_str());
    YAML::Node config = YAML::LoadFile(config_file_path_);

    // Load joint configurations
    YAML::Node joints = config["hand_config"]["joints"];
    if (joints.IsSequence()) {
      for (size_t i = 0; i < joints.size(); i++) {
        JointConfig joint_config;
        joint_config.name = joints[i]["name"].as<std::string>();
        joint_config.finger = joints[i]["finger"].as<std::string>();
        joint_config.role = joints[i]["role"].as<std::string>();
        joint_config.limit_max = joints[i]["limit_max"].as<double>();
        joint_config.velocity_limit = joint_config.limit_max * hand_speed_;
        joint_config.default_position = joints[i]["default_position"].as<double>();

        // Store joint info
        joint_order_.push_back(joint_config);
        joint_names_.push_back(joint_config.name);
        joint_positions_[joint_config.name] = joint_config.default_position;
        joint_limits_[joint_config.name] = joint_config.limit_max;
        joint_velocity_limits_[joint_config.name] = joint_config.velocity_limit;
        joint_configs_[joint_config.name] = joint_config;

        // Create finger-to-role-to-joint mappings
        finger_joints_[joint_config.finger][joint_config.role].push_back(joint_config.name);
      }
    }

    RCLCPP_INFO(
      this->get_logger(), "Configuration loaded successfully. Controlling %zu joints.",
      joint_names_.size());

    // Log the finger types found
    RCLCPP_INFO(this->get_logger(), "Fingers detected:");
    for (const auto & finger_entry : finger_joints_) {
      RCLCPP_INFO(this->get_logger(), "  Finger: %s", finger_entry.first.c_str());
      for (const auto & role_entry : finger_entry.second) {
        RCLCPP_INFO(
          this->get_logger(), "    Role: %s, Joints: %zu", role_entry.first.c_str(),
          role_entry.second.size());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading configuration: %s", e.what());
    RCLCPP_INFO(this->get_logger(), "Using default configuration");
    // Define default joints with finger and role classifications
    joint_order_ = {
      {"thumb_proximal_yaw_joint", "thumb", "yaw", 1.308, 0.0, 0.0},
      {"thumb_proximal_pitch_joint", "thumb", "pitch", 0.6, 0.0, 0.0},
      {"index_proximal_joint", "index", "flex", 1.47, 0.0, 0.0},
      {"middle_proximal_joint", "middle", "flex", 1.47, 0.0, 0.0},
      {"ring_proximal_joint", "ring", "flex", 1.47, 0.0, 0.0},
      {"pinky_proximal_joint", "pinky", "flex", 1.47, 0.0, 0.0}};

    for (const auto & joint : joint_order_) {
      joint_names_.push_back(joint.name);
      joint_positions_[joint.name] = joint.default_position;
      joint_limits_[joint.name] = joint.limit_max;
      joint_velocity_limits_[joint.name] = joint.limit_max * hand_speed_;
      joint_configs_[joint.name] = joint;
      finger_joints_[joint.finger][joint.role].push_back(joint.name);
    }
  }
}

std::vector<double> HandGestureInterpreter::get_ordered_positions() const
{
  std::vector<double> result;

  for (const auto & joint : joint_order_) {
    auto it = joint_positions_.find(joint.name);

    if (it == joint_positions_.end()) {
      RCLCPP_WARN(this->get_logger(), "Missing joint: %s", joint.name.c_str());
      result.push_back(joint.default_position);  // safer
    } else {
      result.push_back(it->second);
    }
  }

  return result;
}

std_msgs::msg::Float64MultiArray HandGestureInterpreter::build_position_msg(
  const std::vector<double> & data)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = data;
  return msg;
}

//===== GESTURE TRANSITION METHODS =====

void HandGestureInterpreter::execute_gesture(
  const std::string & gesture, double duration, double percentage)
{
  prepare_gesture_transition(gesture, percentage);
  joint_positions_ = target_positions_;
  publish_joint_states();
  double estimated_duration = calculate_estimated_duration();

  duration = (duration > estimated_duration) ? duration : estimated_duration;
  RCLCPP_INFO(this->get_logger(), "duration : %2f", duration);

  // Simulation transition running
  transition_in_progress_ = true;
  transition_progress_ = 0.0;

  // Timer for tracking progress
  double update_freq = 20.0;
  double dt = 1.0 / update_freq;

  transition_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_freq)), [this, dt, duration]() {
      transition_progress_ += dt / duration;
      if (transition_progress_ >= 1.0) {
        transition_progress_ = 1.0;
        transition_in_progress_ = false;
        transition_timer_->cancel();
        transition_timer_.reset();
        RCLCPP_INFO(this->get_logger(), "Gesture transition completed");
      }
    });
}

double HandGestureInterpreter::calculate_estimated_duration()
{
  double max_time = 0.0;
  for (const auto & joint : joint_names_) {
    double delta = std::abs(target_positions_[joint] - start_positions_[joint]);
    double time = delta / joint_velocity_limits_[joint];
    if (time > max_time) max_time = time;
  }
  return std::max(max_time, 0.1);  // min 100ms
}

void HandGestureInterpreter::prepare_gesture_transition(
  const std::string & gesture, double percentage)
{
  // Store current positions as starting points
  start_positions_ = joint_positions_;

  // Create a copy of current positions for target
  target_positions_ = joint_positions_;

  // Reset progress
  transition_progress_ = 0.0;

  // Compute target positions based on the requested gesture
  // First reset positions to ensure clean state for gesture calculation
  if (gesture.find("debug_finger_") != 0) {
    reset_joint_positions();
  }

  // Special parameter gestures
  if (gesture.find("grasp_") == 0) {
    try {
      size_t pos = gesture.find_last_of('_');
      if (pos != std::string::npos) {
        double grasp_percentage = std::stod(gesture.substr(pos + 1));
        grasp(grasp_percentage);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse grasp percentage: %s", e.what());
    }
  }
  // Debug finger gesture
  else if (gesture.find("debug_finger_") == 0) {
    parse_and_execute_debug_gesture(gesture);
  }
  // Basic hand gestures
  else if (gesture == "grasp") {
    grasp(percentage > 0.0 ? percentage : 1.0);
  } else if (gesture == "pinch") {
    pinch(percentage > 0.0 ? percentage : 1.0);
  } else if (gesture == "three_finger_grasp") {
    three_finger_grasp(percentage < 0.0 ? 0.5 : percentage);
  } else if (gesture == "open_hand") {
    grasp(0.0);
  }
  // Counting gestures
  else if (gesture == "one") {
    count_one();
  } else if (gesture == "two") {
    count_two();
  } else if (gesture == "three") {
    count_three();
  } else if (gesture == "four") {
    count_four();
  } else if (gesture == "five") {
    count_five();
  } else if (gesture == "loose_fist") {
    grasp(0.6);
  }
  // Communication gestures
  else if (gesture == "point") {
    point();
  } else if (gesture == "thumbs_up") {
    thumbs_up();
  } else if (gesture == "ok") {
    ok();
  } else if (gesture == "peace") {
    peace();
  } else if (gesture == "call_me") {
    call_me();
  }
  // Fun/special gestures
  else if (gesture == "rock") {
    rock();
  } else if (gesture == "fist_bump") {
    fist_bump();
  } else if (gesture == "gun") {
    gun();
  } else if (gesture == "spider_man") {
    spider_man();
  } else if (gesture == "finger_cross") {
    finger_cross();
  } else if (gesture == "italian_hand") {
    italian_hand();
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown gesture: %s", gesture.c_str());
  }

  // Save the calculated positions as target
  target_positions_ = joint_positions_;

  // Restore current positions for smooth transition
  joint_positions_ = start_positions_;
}

//===== FINGER ABSTRACTION METHODS =====

void HandGestureInterpreter::set_finger_position(
  const std::string & finger, const std::string & role, double percentage)
{
  auto finger_it = finger_joints_.find(finger);
  if (finger_it != finger_joints_.end()) {
    auto role_it = finger_it->second.find(role);
    if (role_it != finger_it->second.end()) {
      for (const auto & joint_name : role_it->second) {
        joint_positions_[joint_name] = joint_limits_[joint_name] * percentage;
      }
    }
  }
}

void HandGestureInterpreter::set_finger_positions(const std::string & finger, double percentage)
{
  auto finger_it = finger_joints_.find(finger);
  if (finger_it != finger_joints_.end()) {
    for (const auto & role_entry : finger_it->second) {
      for (const auto & joint_name : role_entry.second) {
        joint_positions_[joint_name] = joint_limits_[joint_name] * percentage;
      }
    }
  }
}

void HandGestureInterpreter::set_all_fingers_except(
  const std::vector<std::string> & exceptions, double percentage)
{
  for (const auto & finger_entry : finger_joints_) {
    if (std::find(exceptions.begin(), exceptions.end(), finger_entry.first) == exceptions.end()) {
      set_finger_positions(finger_entry.first, percentage);
    }
  }
}

//===== CORE NODE METHODS =====

void HandGestureInterpreter::publish_joint_states()
{
  auto data = get_ordered_positions();

  for (size_t i = 0; i < joint_order_.size(); ++i) {
    RCLCPP_DEBUG(this->get_logger(), "joint[%s]: %f", joint_order_[i].name.c_str(), data[i]);
  }
  auto position_msg = build_position_msg(data);
  position_command_publisher_->publish(position_msg);
}

void HandGestureInterpreter::gesture_callback(const std_msgs::msg::String::SharedPtr msg)
{
  // For backward compatibility, create an action goal
  RCLCPP_INFO(this->get_logger(), "Received gesture command via topic: %s", msg->data.c_str());
  RCLCPP_INFO(this->get_logger(), "Converting to action for smooth transition");

  // Check for demo control commands
  if (msg->data == "demo_start") {
    start_demo_mode();
    return;
  } else if (msg->data == "demo_stop") {
    stop_demo_mode();
    return;
  }

  // Execute the gesture with default transition duration
  execute_gesture(msg->data, transition_duration_);
}

void HandGestureInterpreter::gripper_command_callback(
  const control_msgs::msg::GripperCommand::SharedPtr msg)
{
  // Calculate the percentage based on the gripper position
  // msg->position is the desired gap between fingers in meters
  // 0.0 meters = fully closed (percentage = 1.0)
  // gripper_max_range_ meters = fully open (percentage = 0.0)
  double clamped_position = std::max(0.0, std::min(gripper_max_range_, msg->position));
  double grasp_percentage = 1.0 - (clamped_position / gripper_max_range_);

  // Combined log message with all relevant information
  RCLCPP_INFO(
    this->get_logger(),
    "Gripper command received: position=%.3f m, effort=%.3f, mapped to %.1f%% closed grasp",
    msg->position, msg->max_effort, grasp_percentage * 100.0);

  // Stop demo mode if it's running
  if (demo_timer_) {
    stop_demo_mode();
  }

  // Use three_finger_grasp gesture with smooth transition
  if (grasp_percentage > 0.2) {
    execute_gesture("three_finger_grasp", transition_duration_, grasp_percentage);
  } else {
    execute_gesture("open_hand", transition_duration_);
  }
}

// Add landmarks callback implementation
void HandGestureInterpreter::landmarks_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg [[maybe_unused]])
{
  // Update last received time
  last_landmarks_time_ = std::chrono::steady_clock::now();

  // If this is the first landmark message or we haven't already stopped the demo
  if (!hand_landmarks_received_ || !landmarks_demo_mode_stopped_) {
    RCLCPP_INFO(this->get_logger(), "Hand landmarks detected, stopping demo mode if running");

    // Stop demo mode if it's running
    if (demo_timer_) {
      stop_demo_mode();
      landmarks_demo_mode_stopped_ = true;
    }
  }

  // Set flag to indicate we've received landmarks
  hand_landmarks_received_ = true;
}

// Add activity checking method
void HandGestureInterpreter::check_landmarks_activity()
{
  // If we've never received landmarks, don't do anything
  if (!hand_landmarks_received_) {
    return;
  }

  // Calculate time since last landmark
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_landmarks_time_);

  // If landmarks have timed out and demo mode isn't running and auto demo is enabled
  if (
    elapsed > landmarks_timeout_ && !demo_timer_ && auto_demo_enabled_ &&
    landmarks_demo_mode_stopped_) {
    RCLCPP_INFO(
      this->get_logger(), "No hand landmarks received for %ld seconds, restarting demo mode",
      elapsed.count());

    // Restart demo mode
    start_demo_mode();

    // Reset flag since we're back in demo mode
    landmarks_demo_mode_stopped_ = false;
  }
}

//===== JOINT CONTROL METHODS =====

void HandGestureInterpreter::set_joint_position(const std::string & joint_name, double position)
{
  auto it = joint_positions_.find(joint_name);
  if (it != joint_positions_.end()) {
    it->second = position;
  }
}

void HandGestureInterpreter::reset_joint_positions()
{
  for (const auto & name : joint_names_) {
    joint_positions_[name] = 0.0;
  }
}

//===== GESTURE IMPLEMENTATION METHODS =====

// Basic hand gestures
void HandGestureInterpreter::grasp(double percentage)
{
  // Set all fingers to appropriate percentage of their max limit
  for (const auto & finger_entry : finger_joints_) {
    set_finger_positions(finger_entry.first, percentage);
  }
}

void HandGestureInterpreter::pinch(double percentage)
{
  // Reset all positions first
  reset_joint_positions();

  // Position the thumb with more control over its joints
  if (finger_joints_.count("thumb") > 0) {
    // For the yaw component of the thumb, use a reduced percentage to avoid
    // thumb moving too close to the palm
    if (finger_joints_["thumb"].count("yaw") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["yaw"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.9 * percentage;
      }
    }

    // For pitch joints, use a higher percentage to bring thumb tip toward index
    if (finger_joints_["thumb"].count("pitch") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["pitch"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.4 * percentage;
      }
    }

    // For other thumb joints (like flex/curl), use a moderate value
    for (const auto & role_entry : finger_joints_["thumb"]) {
      if (role_entry.first != "yaw" && role_entry.first != "pitch") {
        for (const auto & joint_name : role_entry.second) {
          joint_positions_[joint_name] = joint_limits_[joint_name] * 0.6 * percentage;
        }
      }
    }
  }

  // Position index finger - slightly less closed to meet the thumb
  set_finger_positions("index", percentage * 0.6);

  // Close other fingers fully or not at all depending on percentage
  double other_percentage = percentage > 0.3 ? 1.0 : 0.0;
  std::vector<std::string> exceptions = {"thumb", "index"};
  set_all_fingers_except(exceptions, other_percentage);

  RCLCPP_DEBUG(this->get_logger(), "Pinch gesture set with percentage: %f", percentage);
}

void HandGestureInterpreter::three_finger_grasp(double percentage)
{
  // Fully closed state for the three finger grasp gesture
  double index_max_range = 0.75;  // FIXME: temporary value for the broken finger
  double middle_max_range = 0.55;
  double thumb_max_range = 0.50;

  // Reset all positions first
  reset_joint_positions();

  // Set thumb
  set_finger_position("thumb", "yaw", 1.0);
  set_finger_position("thumb", "pitch", percentage * thumb_max_range);

  // Set index and middle fingers
  set_finger_positions("index", percentage * index_max_range);
  set_finger_positions("middle", percentage * middle_max_range);

  // Set ring and pinky fully closed
  set_finger_positions("ring", 1.0);
  set_finger_positions("pinky", 1.0);
}

// Communication gestures
void HandGestureInterpreter::point()
{
  // Close all fingers
  grasp(1.0);

  // Extend index finger
  set_finger_positions("index", 0.0);
}

void HandGestureInterpreter::thumbs_up()
{
  // Close all fingers except thumb
  std::vector<std::string> exceptions = {"thumb"};
  set_all_fingers_except(exceptions, 1.0);

  // Open thumb completely
  set_finger_positions("thumb", 0.0);
}

void HandGestureInterpreter::thumbs_down()
{
  // Without wrist joint, thumbs down is the same as thumbs up
  // Redirect to thumbs_up implementation
  RCLCPP_WARN(
    this->get_logger(), "thumbs_down gesture requires wrist joint, using thumbs_up instead");
  thumbs_up();
}

void HandGestureInterpreter::wave(double position)
{
  // Move all joints based on the position parameter
  for (const auto & finger_entry : finger_joints_) {
    set_finger_positions(finger_entry.first, position);
  }
}

void HandGestureInterpreter::ok()
{
  // Reset positions first
  reset_joint_positions();

  // Position thumb to connect with index finger
  if (finger_joints_.count("thumb") > 0) {
    if (finger_joints_["thumb"].count("yaw") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["yaw"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.9;
      }
    }
    if (finger_joints_["thumb"].count("pitch") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["pitch"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.5;
      }
    }
  }

  // Curl index finger to meet thumb
  set_finger_positions("index", 0.6);

  // Other fingers slightly flexed
  set_finger_positions("middle", 0.08);
  set_finger_positions("ring", 0.05);
  set_finger_positions("pinky", 0.0);
}

void HandGestureInterpreter::call_me()
{
  // Close all fingers first
  grasp(1.0);

  // Extend thumb and pinky
  set_finger_positions("thumb", 0.0);
  set_finger_positions("pinky", 0.0);
}

void HandGestureInterpreter::peace()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index and middle fingers
  set_finger_positions("index", 0.0);
  set_finger_positions("middle", 0.0);
}

// Counting gestures
void HandGestureInterpreter::count_one()
{
  // Same as pointing
  point();
}

void HandGestureInterpreter::count_two()
{
  // Same as peace
  peace();
}

void HandGestureInterpreter::count_three()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index, middle and thumb
  set_finger_positions("index", 0.0);
  set_finger_positions("middle", 0.0);
  set_finger_positions("ring", 0.0);
}

void HandGestureInterpreter::count_four()
{
  // Close all fingers first
  grasp(1.0);

  // Keep thumb closed, extend all others
  set_finger_positions("index", 0.0);
  set_finger_positions("middle", 0.0);
  set_finger_positions("ring", 0.0);
  set_finger_positions("pinky", 0.0);
}

void HandGestureInterpreter::count_five()
{
  // Just open the hand
  grasp(0.0);
}

// Fun/special gestures
void HandGestureInterpreter::rock()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index and pinky
  set_finger_positions("index", 0.0);
  set_finger_positions("pinky", 0.0);
}

void HandGestureInterpreter::fist_bump()
{
  // Close all fingers
  grasp(1.0);

  // Adjust thumb position to be alongside rather than across palm
  if (finger_joints_.count("thumb") > 0) {
    if (finger_joints_["thumb"].count("yaw") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["yaw"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.5;  // Half of max limit
      }
    }
    if (finger_joints_["thumb"].count("pitch") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["pitch"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.7;  // 70% of max limit
      }
    }
  }
}

void HandGestureInterpreter::gun()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index finger (like pointing)
  set_finger_positions("index", 0.0);

  // Extend thumb perpendicular to index (representing the hammer)
  set_finger_positions("thumb", 0.5);
}

void HandGestureInterpreter::spider_man()
{
  // Close ring and middle fingers
  set_finger_positions("ring", 1.0);
  set_finger_positions("middle", 1.0);

  // Extend thumb, index, and pinky
  set_finger_positions("thumb", 0.0);
  set_finger_positions("index", 0.0);
  set_finger_positions("pinky", 0.0);
}

void HandGestureInterpreter::finger_cross()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index and middle fingers
  set_finger_positions("index", 0.0);
  set_finger_positions("middle", 0.0);

  // For a crossed finger effect, we would ideally need more joints to cross them,
  // but we can simulate by partial bending of the index
  if (finger_joints_.count("index") > 0) {
    for (const auto & role_entry : finger_joints_["index"]) {
      for (const auto & joint_name : role_entry.second) {
        joint_positions_[joint_name] = joint_limits_[joint_name] * 0.3;
      }
    }
  }
}

void HandGestureInterpreter::italian_hand()
{
  // Position all fingers to form a pinched appearance
  // All slightly bent but not fully closed
  set_finger_positions("thumb", 0.4);
  set_finger_positions("index", 0.4);
  set_finger_positions("middle", 0.4);
  set_finger_positions("ring", 0.4);
  set_finger_positions("pinky", 0.4);

  // Adjust thumb to meet other fingers
  if (finger_joints_.count("thumb") > 0) {
    if (finger_joints_["thumb"].count("yaw") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["yaw"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.6;
      }
    }

    if (finger_joints_["thumb"].count("pitch") > 0) {
      for (const auto & joint : finger_joints_["thumb"]["pitch"]) {
        joint_positions_[joint] = joint_limits_[joint] * 0.6;
      }
    }
  }
}

// Debug gestures
void HandGestureInterpreter::debug_finger(
  const std::string & finger, const std::string & role, double percentage)
{
  RCLCPP_INFO(
    this->get_logger(), "Debug gesture: setting finger '%s', role '%s' to %.1f%%", finger.c_str(),
    role.c_str(), percentage * 100.0);

  // Check if finger exists
  auto finger_it = finger_joints_.find(finger);
  if (finger_it == finger_joints_.end()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Debug gesture: finger '%s' not found. Available fingers:", finger.c_str());
    for (const auto & f : finger_joints_) {
      RCLCPP_ERROR(this->get_logger(), "  - %s", f.first.c_str());
    }
    return;
  }

  // Check if role exists for this finger
  auto role_it = finger_it->second.find(role);
  if (role_it == finger_it->second.end()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Debug gesture: role '%s' not found for finger '%s'. Available roles:", role.c_str(),
      finger.c_str());
    for (const auto & r : finger_it->second) {
      RCLCPP_ERROR(this->get_logger(), "  - %s", r.first.c_str());
    }
    return;
  }

  // Apply the percentage to the specified finger and role
  set_finger_position(finger, role, percentage);

  // Log the joints that were affected
  RCLCPP_INFO(this->get_logger(), "Debug gesture: affected joints:");
  for (const auto & joint_name : role_it->second) {
    double position = joint_positions_[joint_name];
    double limit = joint_limits_[joint_name];
    RCLCPP_INFO(
      this->get_logger(), "  - %s: %.3f rad (%.1f%% of limit %.3f)", joint_name.c_str(), position,
      (position / limit) * 100.0, limit);
  }
}

void HandGestureInterpreter::parse_and_execute_debug_gesture(const std::string & gesture_command)
{
  // Expected format: debug_finger_<finger>_<role>_<percentage>
  // Example: debug_finger_thumb_yaw_50

  std::string prefix = "debug_finger_";
  if (gesture_command.find(prefix) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid debug gesture format: %s", gesture_command.c_str());
    return;
  }

  std::string params = gesture_command.substr(prefix.length());

  // Split by underscores
  std::vector<std::string> parts;
  std::stringstream ss(params);
  std::string part;

  while (std::getline(ss, part, '_')) {
    if (!part.empty()) {
      parts.push_back(part);
    }
  }

  if (parts.size() != 3) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Debug gesture requires format: debug_finger_<finger>_<role>_<percentage>");
    RCLCPP_ERROR(this->get_logger(), "Example: debug_finger_thumb_yaw_50");
    RCLCPP_ERROR(this->get_logger(), "Available fingers:");
    for (const auto & f : finger_joints_) {
      RCLCPP_ERROR(this->get_logger(), "  - %s (roles: ", f.first.c_str());
      for (const auto & r : f.second) {
        RCLCPP_ERROR(this->get_logger(), "%s ", r.first.c_str());
      }
      RCLCPP_ERROR(this->get_logger(), ")");
    }
    return;
  }

  std::string finger = parts[0];
  std::string role = parts[1];

  try {
    double percentage_value = std::stod(parts[2]);
    // Convert percentage (0-100) to fraction (0.0-1.0)
    double percentage = percentage_value / 100.0;

    // Clamp to valid range
    percentage = std::max(0.0, std::min(1.0, percentage));

    debug_finger(finger, role, percentage);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse percentage in debug gesture: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Percentage should be a number between 0 and 100");
  }
}

}  // namespace arm_hand_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandGestureInterpreter>());
  rclcpp::shutdown();
  return 0;
}
