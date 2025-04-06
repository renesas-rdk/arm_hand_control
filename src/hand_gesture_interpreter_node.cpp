#include "arm_hand_control/hand_gesture_interpreter_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace arm_hand_control
{

HandGestureInterpreter::HandGestureInterpreter() : Node("hand_gesture_interpreter")
{
  // Declare parameters
  this->declare_parameter("config_file", "config/hand/inspire_rh56.yaml");

  // Get parameters
  config_file_path_ = this->get_parameter("config_file").as_string();

  // Make the path absolute if it's relative
  if (!std::filesystem::path(config_file_path_).is_absolute())
  {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_hand_control");
    config_file_path_ = pkg_path + "/" + config_file_path_;
  }

  // Load configuration
  load_configuration();

  // Create publisher
  auto qos = rclcpp::QoS(10).reliable();
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

  // Create subscriber
  gesture_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "hand_gesture", 10, std::bind(&HandGestureInterpreter::gesture_callback, this, std::placeholders::_1));

  // Create timer for publishing joint states
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_)),
                                   std::bind(&HandGestureInterpreter::publish_joint_states, this));

  RCLCPP_INFO(this->get_logger(), "Hand gesture interpreter started");
}

void HandGestureInterpreter::load_configuration()
{
  try
  {
    RCLCPP_INFO(this->get_logger(), "Loading configuration from: %s", config_file_path_.c_str());
    YAML::Node config = YAML::LoadFile(config_file_path_);

    // Get publish rate
    publish_rate_hz_ = config["hand_config"]["publish_rate"].as<double>();

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

    // Set default values in case of error
    publish_rate_hz_ = 10.0;

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

// Helper methods to work with finger abstractions
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
  std::string gesture = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received gesture command: %s", gesture.c_str());

  if (gesture == "grasp")
  {
    grasp(1.0);
  }
  else if (gesture == "pinch")
  {
    pinch(1.0);
  }
  else if (gesture == "point")
  {
    point();
  }
  else if (gesture == "thumbs_up")
  {
    thumbs_up();
  }
  else if (gesture == "thumbs_down")
  {
    thumbs_down();
  }
  else if (gesture == "ok")
  {
    ok();
  }
  else if (gesture == "rock")
  {
    rock();
  }
  else if (gesture == "peace")
  {
    peace();
  }
  else if (gesture == "three_finger_grasp")
  {
    three_finger_grasp(1.0);
  }
  else if (gesture == "call_me")
  {
    call_me();
  }
  else if (gesture == "fist_bump")
  {
    fist_bump();
  }
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
  else if (gesture == "open_hand")
  {
    grasp(0.0);
  }
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
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown gesture: %s", gesture.c_str());
  }
}

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

  // Position thumb
  set_finger_positions("thumb", percentage);

  // Position index finger
  set_finger_positions("index", percentage * 0.7);  // Slightly less closed than a full grasp

  // Close other fingers fully or not at all depending on percentage
  double other_percentage = percentage > 0.3 ? 1.0 : 0.0;
  std::vector<std::string> exceptions = { "thumb", "index" };
  set_all_fingers_except(exceptions, other_percentage);
}

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

void HandGestureInterpreter::wave(double position)
{
  // Move all joints based on the position parameter
  for (const auto& finger_entry : finger_joints_)
  {
    set_finger_positions(finger_entry.first, position);
  }
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

void HandGestureInterpreter::rock()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index and pinky
  set_finger_positions("index", 0.0);
  set_finger_positions("pinky", 0.0);
}

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

void HandGestureInterpreter::thumbs_down()
{
  // Close all fingers except thumb
  std::vector<std::string> exceptions = { "thumb" };
  set_all_fingers_except(exceptions, 1.0);

  // Position thumb downward by fully extending relevant joints
  set_finger_positions("thumb", 1.0);
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

}  // namespace arm_hand_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandGestureInterpreter>());
  rclcpp::shutdown();
  return 0;
}
