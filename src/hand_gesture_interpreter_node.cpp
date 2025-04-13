#include "arm_hand_control/hand_gesture_interpreter_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace arm_hand_control
{

HandGestureInterpreter::HandGestureInterpreter()
  : Node("hand_gesture_interpreter"), current_gesture_idx_(0), demo_gesture_in_progress_(false)
{
  // Declare parameters
  this->declare_parameter("config_file", "config/hand/inspire_rh56.yaml");
  this->declare_parameter("auto_demo_enabled", true);
  this->declare_parameter("gesture_duration", 3.0);

  // Get parameters
  config_file_path_ = this->get_parameter("config_file").as_string();
  auto_demo_enabled_ = this->get_parameter("auto_demo_enabled").as_bool();
  gesture_duration_ = this->get_parameter("gesture_duration").as_double();

  // Make the path absolute if it's relative
  if (!std::filesystem::path(config_file_path_).is_absolute())
  {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_hand_control");
    config_file_path_ = pkg_path + "/" + config_file_path_;
  }

  // Load configuration
  load_configuration();

  // Create publisher
  auto qos = rclcpp::QoS(1).reliable().durability_volatile();
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

  // Create subscriber
  gesture_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "hand_gesture", qos, std::bind(&HandGestureInterpreter::gesture_callback, this, std::placeholders::_1));

  // Publish initial joint states
  publish_joint_states();

  RCLCPP_INFO(this->get_logger(), "Hand gesture interpreter started");
  RCLCPP_INFO(this->get_logger(), "Listening for gestures on topic: %s", gesture_subscriber_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Publishing joint states on topic: %s", joint_state_publisher_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Using configuration file: %s", config_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Auto demo enabled: %s", auto_demo_enabled_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Gesture duration: %.2f seconds", gesture_duration_);
  RCLCPP_INFO(this->get_logger(), "Available gestures: %zu", get_all_available_gestures().size());
  for (const auto& gesture : get_all_available_gestures())
  {
    RCLCPP_INFO(this->get_logger(), "  - %s", gesture.c_str());
  }

  // Start demo mode if enabled
  if (auto_demo_enabled_)
  {
    RCLCPP_INFO(this->get_logger(), "Auto demo mode enabled");
    start_demo_mode();
  }
}

//===== DEMO MODE METHODS =====

void HandGestureInterpreter::start_demo_mode()
{
  current_gesture_idx_ = 0;
  // Create timer for cycling through gestures
  demo_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(gesture_duration_ * 1000)),
                                        std::bind(&HandGestureInterpreter::demo_timer_callback, this));

  // Execute first gesture immediately
  demo_timer_callback();
}

void HandGestureInterpreter::stop_demo_mode()
{
  if (demo_timer_)
  {
    demo_timer_->cancel();
    demo_timer_.reset();
    RCLCPP_INFO(this->get_logger(), "Auto demo mode stopped");
  }
}

std::vector<std::string> HandGestureInterpreter::get_all_available_gestures()
{
  return { // Basic hand gestures
           "grasp", "pinch", "three_finger_grasp", "open_hand",

           // Counting gestures
           "one", "two", "three", "four", "five",

           // Communication gestures
           "point", "thumbs_up", "ok", "peace", "call_me",

           // Fun/special gestures
           "rock", "fist_bump", "gun", "spider_man", "hang_loose", "thumbs_middle", "finger_cross", "italian_hand"
  };
}

void HandGestureInterpreter::demo_timer_callback()
{
  auto gestures = get_all_available_gestures();
  if (gestures.empty())
    return;

  std::string current_gesture = gestures[current_gesture_idx_];
  RCLCPP_INFO(this->get_logger(), "Auto demo showing gesture: %s", current_gesture.c_str());

  // Process the gesture
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = current_gesture;

  // Set a flag to indicate this is from the demo
  // This prevents the gesture_callback from stopping the demo mode
  demo_gesture_in_progress_ = true;

  gesture_callback(msg);

  // Reset the flag
  demo_gesture_in_progress_ = false;

  // Move to next gesture
  current_gesture_idx_ = (current_gesture_idx_ + 1) % gestures.size();
}

//===== CONFIGURATION METHODS =====

void HandGestureInterpreter::load_configuration()
{
  try
  {
    RCLCPP_INFO(this->get_logger(), "Loading configuration from: %s", config_file_path_.c_str());
    YAML::Node config = YAML::LoadFile(config_file_path_);

    // Load joint configurations
    YAML::Node joints = config["hand_config"]["joints"];
    if (joints.IsSequence())
    {
      for (size_t i = 0; i < joints.size(); i++)
      {
        JointConfig joint_config;
        joint_config.name = joints[i]["name"].as<std::string>();
        joint_config.finger = joints[i]["finger"].as<std::string>();
        joint_config.role = joints[i]["role"].as<std::string>();
        joint_config.limit_max = joints[i]["limit_max"].as<double>();
        joint_config.default_position = joints[i]["default_position"].as<double>();

        // Store joint info
        joint_names_.push_back(joint_config.name);
        joint_positions_[joint_config.name] = joint_config.default_position;
        joint_limits_[joint_config.name] = joint_config.limit_max;
        joint_configs_[joint_config.name] = joint_config;

        // Create finger-to-role-to-joint mappings
        finger_joints_[joint_config.finger][joint_config.role].push_back(joint_config.name);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully. Controlling %zu joints.", joint_names_.size());

    // Log the finger types found
    RCLCPP_INFO(this->get_logger(), "Fingers detected:");
    for (const auto& finger_entry : finger_joints_)
    {
      RCLCPP_INFO(this->get_logger(), "  Finger: %s", finger_entry.first.c_str());
      for (const auto& role_entry : finger_entry.second)
      {
        RCLCPP_INFO(this->get_logger(), "    Role: %s, Joints: %zu", role_entry.first.c_str(),
                    role_entry.second.size());
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error loading configuration: %s", e.what());
    RCLCPP_INFO(this->get_logger(), "Using default configuration");

    // Define default joints with finger and role classifications
    std::vector<JointConfig> default_joints = { { "thumb_proximal_yaw_joint", "thumb", "yaw", 1.308, 0.0 },
                                                { "thumb_proximal_pitch_joint", "thumb", "pitch", 0.6, 0.0 },
                                                { "index_proximal_joint", "index", "flex", 1.47, 0.0 },
                                                { "middle_proximal_joint", "middle", "flex", 1.47, 0.0 },
                                                { "ring_proximal_joint", "ring", "flex", 1.47, 0.0 },
                                                { "pinky_proximal_joint", "pinky", "flex", 1.47, 0.0 } };

    for (const auto& joint : default_joints)
    {
      joint_names_.push_back(joint.name);
      joint_positions_[joint.name] = joint.default_position;
      joint_limits_[joint.name] = joint.limit_max;
      joint_configs_[joint.name] = joint;
      finger_joints_[joint.finger][joint.role].push_back(joint.name);
    }
  }
}

//===== FINGER ABSTRACTION METHODS =====

void HandGestureInterpreter::set_finger_position(const std::string& finger, const std::string& role, double position)
{
  auto finger_it = finger_joints_.find(finger);
  if (finger_it != finger_joints_.end())
  {
    auto role_it = finger_it->second.find(role);
    if (role_it != finger_it->second.end())
    {
      for (const auto& joint_name : role_it->second)
      {
        joint_positions_[joint_name] = position;
      }
    }
  }
}

void HandGestureInterpreter::set_finger_positions(const std::string& finger, double percentage)
{
  auto finger_it = finger_joints_.find(finger);
  if (finger_it != finger_joints_.end())
  {
    for (const auto& role_entry : finger_it->second)
    {
      for (const auto& joint_name : role_entry.second)
      {
        joint_positions_[joint_name] = joint_limits_[joint_name] * percentage;
      }
    }
  }
}

void HandGestureInterpreter::set_all_fingers_except(const std::vector<std::string>& exceptions, double percentage)
{
  for (const auto& finger_entry : finger_joints_)
  {
    if (std::find(exceptions.begin(), exceptions.end(), finger_entry.first) == exceptions.end())
    {
      set_finger_positions(finger_entry.first, percentage);
    }
  }
}

//===== CORE NODE METHODS =====

void HandGestureInterpreter::publish_joint_states()
{
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();

  for (const auto& joint : joint_positions_)
  {
    msg.name.push_back(joint.first);
    msg.position.push_back(joint.second);
  }

  joint_state_publisher_->publish(msg);
}

void HandGestureInterpreter::gesture_callback(const std_msgs::msg::String::SharedPtr msg)
{
  // Only stop the demo if we receive an external gesture command
  // Don't stop if the gesture is coming from the demo itself
  if (demo_timer_ && !demo_gesture_in_progress_ && msg->data != "demo_start" && msg->data != "demo_stop")
  {
    stop_demo_mode();
  }

  std::string gesture = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received gesture command: %s", gesture.c_str());

  // Handle demo mode commands
  if (gesture == "demo_start")
  {
    start_demo_mode();
    return;
  }
  else if (gesture == "demo_stop")
  {
    stop_demo_mode();
    return;
  }
  // Special parameter gestures
  else if (gesture.find("grasp_") == 0)
  {
    // Parse grasp with percentage: "grasp_0.5"
    try
    {
      size_t pos = gesture.find_last_of('_');
      if (pos != std::string::npos)
      {
        double percentage = std::stod(gesture.substr(pos + 1));
        grasp(percentage);
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse grasp percentage: %s", e.what());
    }
  }
  // Basic hand gestures
  else if (gesture == "grasp")
  {
    grasp(1.0);
  }
  else if (gesture == "pinch")
  {
    pinch(1.0);
  }
  else if (gesture == "three_finger_grasp")
  {
    three_finger_grasp(0.6);
  }
  else if (gesture == "open_hand")
  {
    grasp(0.0);
  }
  // Counting gestures
  else if (gesture == "one")
  {
    count_one();
  }
  else if (gesture == "two")
  {
    count_two();
  }
  else if (gesture == "three")
  {
    count_three();
  }
  else if (gesture == "four")
  {
    count_four();
  }
  else if (gesture == "five")
  {
    count_five();
  }
  // Communication gestures
  else if (gesture == "point")
  {
    point();
  }
  else if (gesture == "thumbs_up")
  {
    thumbs_up();
  }
  else if (gesture == "ok")
  {
    ok();
  }
  else if (gesture == "peace")
  {
    peace();
  }
  else if (gesture == "call_me")
  {
    call_me();
  }
  // Fun/special gestures
  else if (gesture == "rock")
  {
    rock();
  }
  else if (gesture == "fist_bump")
  {
    fist_bump();
  }
  else if (gesture == "gun")
  {
    gun();
  }
  else if (gesture == "spider_man")
  {
    spider_man();
  }
  else if (gesture == "hang_loose")
  {
    hang_loose();
  }
  else if (gesture == "thumbs_middle")
  {
    thumbs_middle();
  }
  else if (gesture == "finger_cross")
  {
    finger_cross();
  }
  else if (gesture == "italian_hand")
  {
    italian_hand();
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown gesture: %s", gesture.c_str());
  }

  // Publish joint states immediately after processing the gesture
  publish_joint_states();
}

//===== JOINT CONTROL METHODS =====

void HandGestureInterpreter::set_joint_position(const std::string& joint_name, double position)
{
  auto it = joint_positions_.find(joint_name);
  if (it != joint_positions_.end())
  {
    it->second = position;
  }
}

void HandGestureInterpreter::reset_joint_positions()
{
  for (const auto& name : joint_names_)
  {
    joint_positions_[name] = 0.0;
  }
}

//===== GESTURE IMPLEMENTATION METHODS =====

// Basic hand gestures
void HandGestureInterpreter::grasp(double percentage)
{
  // Set all fingers to appropriate percentage of their max limit
  for (const auto& finger_entry : finger_joints_)
  {
    set_finger_positions(finger_entry.first, percentage);
  }
}

void HandGestureInterpreter::pinch(double percentage)
{
  // Reset all positions first
  reset_joint_positions();

  // Position the thumb with more control over its joints
  if (finger_joints_.count("thumb") > 0)
  {
    // For the yaw component of the thumb, use a reduced percentage to avoid
    // thumb moving too close to the palm
    if (finger_joints_["thumb"].count("yaw") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["yaw"])
      {
        joint_positions_[joint] = joint_limits_[joint] * 0.9 * percentage;
      }
    }

    // For pitch joints, use a higher percentage to bring thumb tip toward index
    if (finger_joints_["thumb"].count("pitch") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["pitch"])
      {
        joint_positions_[joint] = joint_limits_[joint] * 0.4 * percentage;
      }
    }

    // For other thumb joints (like flex/curl), use a moderate value
    for (const auto& role_entry : finger_joints_["thumb"])
    {
      if (role_entry.first != "yaw" && role_entry.first != "pitch")
      {
        for (const auto& joint_name : role_entry.second)
        {
          joint_positions_[joint_name] = joint_limits_[joint_name] * 0.6 * percentage;
        }
      }
    }
  }

  // Position index finger - slightly less closed to meet the thumb
  set_finger_positions("index", percentage * 0.6);

  // Close other fingers fully or not at all depending on percentage
  double other_percentage = percentage > 0.3 ? 1.0 : 0.0;
  std::vector<std::string> exceptions = { "thumb", "index" };
  set_all_fingers_except(exceptions, other_percentage);

  RCLCPP_DEBUG(this->get_logger(), "Pinch gesture set with percentage: %f", percentage);
}

void HandGestureInterpreter::three_finger_grasp(double percentage)
{
  // Reset all positions first
  reset_joint_positions();

  // Set thumb, index, and middle fingers
  set_finger_positions("thumb", percentage);
  set_finger_positions("index", percentage * 0.8);  // Slightly less closed
  set_finger_positions("middle", percentage * 0.8);

  // Set ring and pinky fully closed or open depending on percentage
  double other_percentage = percentage > 0.5 ? 1.0 : 0.0;
  set_finger_positions("ring", other_percentage);
  set_finger_positions("pinky", other_percentage);
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
  std::vector<std::string> exceptions = { "thumb" };
  set_all_fingers_except(exceptions, 1.0);

  // Open thumb completely
  set_finger_positions("thumb", 0.0);
}

void HandGestureInterpreter::thumbs_down()
{
  // Without wrist joint, thumbs down is the same as thumbs up
  // Redirect to thumbs_up implementation
  RCLCPP_WARN(this->get_logger(), "thumbs_down gesture requires wrist joint, using thumbs_up instead");
  thumbs_up();
}

void HandGestureInterpreter::wave(double position)
{
  // Move all joints based on the position parameter
  for (const auto& finger_entry : finger_joints_)
  {
    set_finger_positions(finger_entry.first, position);
  }
}

void HandGestureInterpreter::ok()
{
  // Reset positions first
  reset_joint_positions();

  // Position thumb to connect with index finger
  if (finger_joints_.count("thumb") > 0)
  {
    if (finger_joints_["thumb"].count("yaw") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["yaw"])
      {
        joint_positions_[joint] = joint_limits_[joint] * 0.5;  // Half of max limit
      }
    }
    if (finger_joints_["thumb"].count("pitch") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["pitch"])
      {
        joint_positions_[joint] = joint_limits_[joint] * 0.7;  // 70% of max limit
      }
    }
  }

  // Curl index finger to meet thumb
  set_finger_positions("index", 0.5);  // Half closed

  // Other fingers slightly flexed
  set_finger_positions("middle", 0.1);
  set_finger_positions("ring", 0.1);
  set_finger_positions("pinky", 0.1);
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

  // Extend thumb, index, and middle
  set_finger_positions("thumb", 0.0);
  set_finger_positions("index", 0.0);
  set_finger_positions("middle", 0.0);
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
  if (finger_joints_.count("thumb") > 0)
  {
    if (finger_joints_["thumb"].count("yaw") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["yaw"])
      {
        joint_positions_[joint] = joint_limits_[joint] * 0.5;  // Half of max limit
      }
    }
    if (finger_joints_["thumb"].count("pitch") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["pitch"])
      {
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

void HandGestureInterpreter::hang_loose()
{
  // Close middle, ring fingers
  set_finger_positions("middle", 1.0);
  set_finger_positions("ring", 1.0);

  // Extend thumb, index, and pinky
  set_finger_positions("thumb", 0.0);
  set_finger_positions("index", 0.0);
  set_finger_positions("pinky", 0.0);
}

void HandGestureInterpreter::thumbs_middle()
{
  // Close all fingers except thumb and middle
  grasp(1.0);

  // Extend middle finger
  set_finger_positions("middle", 0.0);

  // Position thumb to the side
  set_finger_positions("thumb", 0.3);
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
  if (finger_joints_.count("index") > 0)
  {
    for (const auto& role_entry : finger_joints_["index"])
    {
      for (const auto& joint_name : role_entry.second)
      {
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
  if (finger_joints_.count("thumb") > 0)
  {
    if (finger_joints_["thumb"].count("yaw") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["yaw"])
      {
        joint_positions_[joint] = joint_limits_[joint] * 0.6;
      }
    }

    if (finger_joints_["thumb"].count("pitch") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["pitch"])
      {
        joint_positions_[joint] = joint_limits_[joint] * 0.6;
      }
    }
  }
}

}  // namespace arm_hand_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandGestureInterpreter>());
  rclcpp::shutdown();
  return 0;
}
