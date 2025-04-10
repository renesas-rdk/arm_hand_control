#include <cmath>

#include "arm_hand_control/hand_landmark_interpreter_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace arm_hand_control
{

HandLandmarkInterpreter::HandLandmarkInterpreter() : Node("hand_landmark_interpreter")
{
  // Declare parameters
  this->declare_parameter("config_file", "config/hand/inspire_rh56.yaml");

  // Get parameters
  config_file_path_ = this->get_parameter("config_file").as_string();

  // Load configuration
  load_configuration();

  // Set up the node's QoS settings
  auto qos = rclcpp::QoS(1).best_effort().durability_volatile();

  // Create subscriber to hand landmarks
  landmark_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "hand_landmarks", qos, std::bind(&HandLandmarkInterpreter::landmark_callback, this, std::placeholders::_1));

  // Create publisher for joint states
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

  RCLCPP_INFO(this->get_logger(), "Hand landmark interpreter started");
}

HandLandmarkInterpreter::~HandLandmarkInterpreter()
{
  RCLCPP_INFO(this->get_logger(), "Hand landmark interpreter shutting down");
  landmark_subscriber_.reset();
  joint_state_publisher_.reset();
}

void HandLandmarkInterpreter::load_configuration()
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

void HandLandmarkInterpreter::landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.size() >= static_cast<size_t>(HAND_LANDMARK_COUNT))
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received %zu hand landmarks",
                         msg->poses.size());
    process_landmarks(msg->poses);
    publish_joint_states();
  }
  else
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Incomplete hand landmarks received: %zu (expected 21)", msg->poses.size());
  }
}

void HandLandmarkInterpreter::publish_joint_states()
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

void HandLandmarkInterpreter::set_finger_position(const std::string& finger, const std::string& role, double percentage)
{
  auto finger_it = finger_joints_.find(finger);
  if (finger_it != finger_joints_.end())
  {
    auto role_it = finger_it->second.find(role);
    if (role_it != finger_it->second.end())
    {
      for (const auto& joint_name : role_it->second)
      {
        joint_positions_[joint_name] = joint_limits_[joint_name] * percentage;
      }
    }
  }
}

void HandLandmarkInterpreter::set_finger_positions(const std::string& finger, double percentage)
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

void HandLandmarkInterpreter::set_all_fingers_except(const std::vector<std::string>& exceptions, double percentage)
{
  for (const auto& finger_entry : finger_joints_)
  {
    if (std::find(exceptions.begin(), exceptions.end(), finger_entry.first) == exceptions.end())
    {
      set_finger_positions(finger_entry.first, percentage);
    }
  }
}

void HandLandmarkInterpreter::set_joint_position(const std::string& joint_name, double percentage)
{
  auto it = joint_positions_.find(joint_name);
  if (it != joint_positions_.end())
  {
    it->second = joint_limits_[joint_name] * percentage;
  }
}

void HandLandmarkInterpreter::reset_joint_positions()
{
  for (const auto& name : joint_names_)
  {
    joint_positions_[name] = 0.0;
  }
}

double HandLandmarkInterpreter::calculate_finger_curl(const std::vector<geometry_msgs::msg::Pose>& landmarks,
                                                      const std::string& finger)
{
  // Struct to define parameters for finger angle calculation
  struct finger_angle_config
  {
    int tip_idx;               // landmark index of fingertip
    int mid_idx;               // landmark index of middle joint
    int base_idx;              // landmark index of base joint
    double offset;             // angle offset
    double scale;              // scaling factor
    double additional_offset;  // additional offset for thumb
    double max_angle;          // maximum angle (for percentage normalization)
  };

  // Map fingers to their landmark configuration for angle calculation
  static const std::map<std::string, finger_angle_config> finger_configs = {
    { "pinky", { PINKY_TIP_IDX, PINKY_PIP_IDX, PINKY_MCP_IDX, -20.0, 1.25, 0.0, 180.0 } },
    { "ring", { RING_TIP_IDX, RING_PIP_IDX, RING_MCP_IDX, -20.0, 1.25, 0.0, 180.0 } },
    { "middle", { MIDDLE_TIP_IDX, MIDDLE_PIP_IDX, MIDDLE_MCP_IDX, -20.0, 1.25, 0.0, 180.0 } },
    { "index", { INDEX_TIP_IDX, INDEX_PIP_IDX, INDEX_MCP_IDX, -20.0, 1.25, 0.0, 180.0 } },
    { "thumb_pitch", { THUMB_TIP_IDX, THUMB_MCP_IDX, THUMB_CMC_IDX, -100.0, 1.25, -30.0, 180.0 } },
    { "thumb_yaw", { THUMB_MCP_IDX, THUMB_CMC_IDX, INDEX_MCP_IDX, 0.0, 2.5, 70.0, 180.0 } }
  };

  const auto& config = finger_configs.at(finger);

  // Extract vectors for angle calculation
  std::vector<double> tip_to_mid = { landmarks[config.tip_idx].position.x - landmarks[config.mid_idx].position.x,
                                     landmarks[config.tip_idx].position.y - landmarks[config.mid_idx].position.y };

  std::vector<double> base_to_mid = { landmarks[config.base_idx].position.x - landmarks[config.mid_idx].position.x,
                                      landmarks[config.base_idx].position.y - landmarks[config.mid_idx].position.y };

  // Calculate vector magnitudes
  double mag_tip_to_mid = std::sqrt(tip_to_mid[0] * tip_to_mid[0] + tip_to_mid[1] * tip_to_mid[1]);
  double mag_base_to_mid = std::sqrt(base_to_mid[0] * base_to_mid[0] + base_to_mid[1] * base_to_mid[1]);

  // Prevent division by zero
  if (mag_tip_to_mid < 0.0001 || mag_base_to_mid < 0.0001)
  {
    return 0.0;
  }

  // Calculate angle between vectors
  double dot_product = tip_to_mid[0] * base_to_mid[0] + tip_to_mid[1] * base_to_mid[1];
  double cosine_theta = std::min(1.0, std::max(-1.0, dot_product / (mag_tip_to_mid * mag_base_to_mid)));
  double angle_degrees = std::acos(cosine_theta) * 180.0 / M_PI;

  // Apply the scaling and offset from the original algorithm
  double raw_angle = (angle_degrees + config.offset) * config.scale;

  // Convert to percentage (0.0 to 1.0)
  // For straight finger, angle is large; for curled finger, angle is small
  double percentage = 1.0 - (raw_angle / config.max_angle);

  // Clamp percentage to valid range
  return std::min(1.0, std::max(0.0, percentage));
}

void HandLandmarkInterpreter::process_landmarks(const std::vector<geometry_msgs::msg::Pose>& landmarks)
{
  // Calculate curl (bend) for each finger
  double thumb_yaw_curl = calculate_finger_curl(landmarks, "thumb_yaw");
  double thumb_pitch_curl = calculate_finger_curl(landmarks, "thumb_pitch");
  double index_curl = calculate_finger_curl(landmarks, "index");
  double middle_curl = calculate_finger_curl(landmarks, "middle");
  double ring_curl = calculate_finger_curl(landmarks, "ring");
  double pinky_curl = calculate_finger_curl(landmarks, "pinky");

  // Set positions for fingers
  set_finger_position("thumb", "yaw", thumb_yaw_curl);
  set_finger_position("thumb", "pitch", thumb_pitch_curl);
  set_finger_positions("index", index_curl);
  set_finger_positions("middle", middle_curl);
  set_finger_positions("ring", ring_curl);
  set_finger_positions("pinky", pinky_curl);

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Thumb Yaw: %.2f, Thumb Pitch: %.2f, Index: %.2f, Middle: %.2f, Ring: %.2f, Pinky: %.2f",
                       thumb_yaw_curl, thumb_pitch_curl, index_curl, middle_curl, ring_curl, pinky_curl);
}

}  // namespace arm_hand_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandLandmarkInterpreter>());
  rclcpp::shutdown();
  return 0;
}
