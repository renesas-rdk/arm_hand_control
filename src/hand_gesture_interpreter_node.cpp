#include "arm_hand_control/hand_gesture_interpreter_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace arm_hand_control
{

HandGestureInterpreter::HandGestureInterpreter() : Node("hand_gesture_interpreter")
{
  // Declare parameters
  this->declare_parameter("config_file", "config/hand_config.yaml");

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
        joint_config.limit_max = joints[i]["limit_max"].as<double>();
        joint_config.default_position = joints[i]["default_position"].as<double>();

        joint_names_.push_back(joint_config.name);
        joint_positions_[joint_config.name] = joint_config.default_position;
        joint_limits_[joint_config.name] = joint_config.limit_max;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully. Controlling %zu joints.", joint_names_.size());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error loading configuration: %s", e.what());
    RCLCPP_INFO(this->get_logger(), "Using default configuration");

    // Set default values in case of error
    publish_rate_hz_ = 10.0;

    // Define default joints
    joint_names_ = { "thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint", "index_proximal_joint",
                     "middle_proximal_joint",    "ring_proximal_joint",        "pinky_proximal_joint" };

    joint_limits_ = { { "thumb_proximal_yaw_joint", 1.308 }, { "thumb_proximal_pitch_joint", 0.6 },
                      { "index_proximal_joint", 1.47 },      { "middle_proximal_joint", 1.47 },
                      { "ring_proximal_joint", 1.47 },       { "pinky_proximal_joint", 1.47 } };

    for (const auto& name : joint_names_)
    {
      joint_positions_[name] = 0.0;
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
  else if (gesture == "ok_sign")
  {
    ok_sign();
  }
  else if (gesture == "rock_on")
  {
    rock_on();
  }
  else if (gesture == "peace")
  {
    peace_sign();
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
  // Scale percentage to joint limits
  double thumb_value = joint_limits_["thumb_proximal_yaw_joint"] * percentage;
  double thumb_pitch_value = joint_limits_["thumb_proximal_pitch_joint"] * percentage;

  for (const auto& joint : joint_names_)
  {
    if (joint == "thumb_proximal_yaw_joint")
    {
      joint_positions_[joint] = thumb_value;
    }
    else if (joint == "thumb_proximal_pitch_joint")
    {
      joint_positions_[joint] = thumb_pitch_value;
    }
    else
    {
      joint_positions_[joint] = joint_limits_[joint] * percentage;
    }
  }
}

void HandGestureInterpreter::pinch(double percentage)
{
  // Reset all positions first
  reset_joint_positions();

  // Position thumb
  double thumb_yaw = joint_limits_["thumb_proximal_yaw_joint"] * percentage;
  double thumb_pitch = joint_limits_["thumb_proximal_pitch_joint"] * percentage;

  // Position index finger
  double index_value = 1.0 * percentage;

  // Close other fingers fully
  double other_fingers = percentage > 0.3 ? joint_limits_["middle_proximal_joint"] : 0.0;

  joint_positions_["thumb_proximal_yaw_joint"] = thumb_yaw;
  joint_positions_["thumb_proximal_pitch_joint"] = thumb_pitch;
  joint_positions_["index_proximal_joint"] = index_value;
  joint_positions_["middle_proximal_joint"] = other_fingers;
  joint_positions_["ring_proximal_joint"] = other_fingers;
  joint_positions_["pinky_proximal_joint"] = other_fingers;
}

void HandGestureInterpreter::point()
{
  // Close all fingers
  grasp(1.0);

  // Extend index finger
  joint_positions_["index_proximal_joint"] = 0.0;
}

void HandGestureInterpreter::thumbs_up()
{
  // Close all fingers except thumb
  for (const auto& joint : joint_names_)
  {
    if (joint.find("thumb") == std::string::npos)
    {
      joint_positions_[joint] = joint_limits_[joint];
    }
    else
    {
      joint_positions_[joint] = 0.0;
    }
  }
}

void HandGestureInterpreter::wave(double position)
{
  // Calculate sine-like wave pattern for position (0.0-1.0)
  for (const auto& joint : joint_names_)
  {
    if (joint == "thumb_proximal_yaw_joint")
    {
      joint_positions_[joint] = joint_limits_[joint] * position;
    }
    else if (joint == "thumb_proximal_pitch_joint")
    {
      joint_positions_[joint] = joint_limits_[joint] * position;
    }
    else
    {
      joint_positions_[joint] = joint_limits_[joint] * position;
    }
  }
}

void HandGestureInterpreter::three_finger_grasp(double percentage)
{
  // Reset all positions first
  reset_joint_positions();

  // Position primary fingers
  double thumb_value = joint_limits_["thumb_proximal_yaw_joint"] * percentage;
  double finger_value = 1.2 * percentage;

  // Close ring and pinky fully
  double other_fingers = percentage > 0.5 ? joint_limits_["ring_proximal_joint"] : 0.0;

  joint_positions_["thumb_proximal_yaw_joint"] = thumb_value;
  joint_positions_["thumb_proximal_pitch_joint"] = joint_limits_["thumb_proximal_pitch_joint"] * percentage;
  joint_positions_["index_proximal_joint"] = finger_value;
  joint_positions_["middle_proximal_joint"] = finger_value;
  joint_positions_["ring_proximal_joint"] = other_fingers;
  joint_positions_["pinky_proximal_joint"] = other_fingers;
}

void HandGestureInterpreter::ok_sign()
{
  // Reset positions first
  reset_joint_positions();

  // Set thumb position to connect with index finger
  joint_positions_["thumb_proximal_yaw_joint"] = 0.7;
  joint_positions_["thumb_proximal_pitch_joint"] = 0.4;

  // Curl index finger to meet thumb
  joint_positions_["index_proximal_joint"] = 0.8;

  // Extend other fingers
  joint_positions_["middle_proximal_joint"] = 0.1;
  joint_positions_["ring_proximal_joint"] = 0.1;
  joint_positions_["pinky_proximal_joint"] = 0.1;
}

void HandGestureInterpreter::call_me()
{
  // Close all fingers first
  grasp(1.0);

  // Extend thumb
  joint_positions_["thumb_proximal_yaw_joint"] = 0.0;
  joint_positions_["thumb_proximal_pitch_joint"] = 0.0;

  // Extend pinky
  joint_positions_["pinky_proximal_joint"] = 0.0;
}

void HandGestureInterpreter::peace_sign()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index and middle fingers
  joint_positions_["index_proximal_joint"] = 0.0;
  joint_positions_["middle_proximal_joint"] = 0.0;
}

void HandGestureInterpreter::rock_on()
{
  // Close all fingers first
  grasp(1.0);

  // Extend index and pinky
  joint_positions_["index_proximal_joint"] = 0.0;
  joint_positions_["pinky_proximal_joint"] = 0.0;
}

void HandGestureInterpreter::count_one()
{
  // Same as pointing
  point();
}

void HandGestureInterpreter::count_two()
{
  // Same as peace sign
  peace_sign();
}

void HandGestureInterpreter::count_three()
{
  // Close all fingers first
  grasp(1.0);

  // Extend thumb, index, and middle
  joint_positions_["thumb_proximal_yaw_joint"] = 0.0;
  joint_positions_["thumb_proximal_pitch_joint"] = 0.0;
  joint_positions_["index_proximal_joint"] = 0.0;
  joint_positions_["middle_proximal_joint"] = 0.0;
}

void HandGestureInterpreter::count_four()
{
  // Close all fingers first
  grasp(1.0);

  // Keep thumb closed, extend all others
  joint_positions_["index_proximal_joint"] = 0.0;
  joint_positions_["middle_proximal_joint"] = 0.0;
  joint_positions_["ring_proximal_joint"] = 0.0;
  joint_positions_["pinky_proximal_joint"] = 0.0;
}

void HandGestureInterpreter::count_five()
{
  // Just open the hand
  grasp(0.0);
}

void HandGestureInterpreter::thumbs_down()
{
  // Close all fingers except thumb
  for (const auto& joint : joint_names_)
  {
    if (joint.find("thumb") == std::string::npos)
    {
      joint_positions_[joint] = joint_limits_[joint];
    }
    else
    {
      joint_positions_[joint] = 0.0;
    }
  }

  // Position thumb downward by fully extending in yaw
  joint_positions_["thumb_proximal_yaw_joint"] = joint_limits_["thumb_proximal_yaw_joint"];
  joint_positions_["thumb_proximal_pitch_joint"] = joint_limits_["thumb_proximal_pitch_joint"];
}

void HandGestureInterpreter::fist_bump()
{
  // Close all fingers
  grasp(1.0);

  // Position thumb alongside rather than across palm
  joint_positions_["thumb_proximal_yaw_joint"] = 0.7;
  joint_positions_["thumb_proximal_pitch_joint"] = 0.4;
}

}  // namespace arm_hand_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandGestureInterpreter>());
  rclcpp::shutdown();
  return 0;
}
