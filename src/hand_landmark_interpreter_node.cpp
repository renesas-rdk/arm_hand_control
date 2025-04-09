#include "arm_hand_control/hand_landmark_interpreter_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <cmath>

namespace arm_hand_control
{

HandLandmarkInterpreter::HandLandmarkInterpreter() : Node("hand_landmark_interpreter")
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
  auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

  // Create subscriber to hand landmarks
  landmark_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "hand_landmarks", qos, std::bind(&HandLandmarkInterpreter::landmark_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Hand landmark interpreter started");
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
  if (msg->poses.size() >= 21)  // Check for complete hand landmarks (MediaPipe model has 21 landmarks)
  {
    landmarks_received_ = true;
    last_landmarks_ = msg->poses;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received %zu hand landmarks",
                         last_landmarks_.size());

    // Process the landmarks to update joint positions
    process_landmarks(last_landmarks_);

    // Publish joint states immediately after processing landmarks
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
  if (!landmarks_received_)
  {
    return;  // Don't publish until we've received landmarks
  }

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();

  for (const auto& joint : joint_positions_)
  {
    msg.name.push_back(joint.first);
    msg.position.push_back(joint.second);
  }

  joint_state_publisher_->publish(msg);
}

// Helper methods to work with finger abstractions
void HandLandmarkInterpreter::set_finger_position(const std::string& finger, const std::string& role, double position)
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

void HandLandmarkInterpreter::set_joint_position(const std::string& joint_name, double position)
{
  auto it = joint_positions_.find(joint_name);
  if (it != joint_positions_.end())
  {
    it->second = position;
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
                                                      int start_idx, int num_joints)
{
  // Calculate the curl (bending) of a finger based on landmark positions
  if (landmarks.size() < static_cast<size_t>(start_idx + num_joints))
  {
    return 0.0;
  }

  double curl = 0.0;

  // Calculate joint angles with specific focus on MCP joint angles
  for (int i = start_idx; i < start_idx + num_joints - 2; i++)
  {
    // Create vectors between joints using 2D information (x,y)
    double v1x = landmarks[i + 1].position.x - landmarks[i].position.x;
    double v1y = landmarks[i + 1].position.y - landmarks[i].position.y;

    double v2x = landmarks[i + 2].position.x - landmarks[i + 1].position.x;
    double v2y = landmarks[i + 2].position.y - landmarks[i + 1].position.y;

    // Normalize vectors
    double len1 = sqrt(v1x * v1x + v1y * v1y);
    double len2 = sqrt(v2x * v2x + v2y * v2y);

    if (len1 > 0 && len2 > 0)
    {
      v1x /= len1;
      v1y /= len1;
      v2x /= len2;
      v2y /= len2;

      // Calculate the angle between vectors using dot product
      double dot_product = v1x * v2x + v1y * v2y;
      dot_product = std::max(-1.0, std::min(1.0, dot_product));
      double angle = acos(dot_product);

      // Apply different weights based on joint position
      double weight = 1.0;
      if (i == start_idx)
      {
        // If this is the MCP joint (landmarks 2, 6, 10, 14, 18)
        weight = 2.5;
      }
      else if (i == start_idx + 1)
      {
        // PIP joint (landmarks 3, 7, 11, 15, 19)
        weight = 2.0;
      }

      curl += angle * weight;
    }
  }

  // For a fist, also consider the proximity of fingertip to palm
  int tip_idx = start_idx + num_joints - 1;  // Fingertip
  int mcp_idx = start_idx;                   // MCP joint

  // Calculate 2D distance from fingertip to MCP joint
  double dx = landmarks[tip_idx].position.x - landmarks[mcp_idx].position.x;
  double dy = landmarks[tip_idx].position.y - landmarks[mcp_idx].position.y;
  double dist_tip_to_mcp = sqrt(dx * dx + dy * dy);

  // Calculate an expected length of the finger when extended
  double expected_finger_length = 0.0;
  for (int i = start_idx; i < start_idx + num_joints - 1; i++)
  {
    double dx = landmarks[i + 1].position.x - landmarks[i].position.x;
    double dy = landmarks[i + 1].position.y - landmarks[i].position.y;
    expected_finger_length += sqrt(dx * dx + dy * dy);
  }

  // If the finger is curled, the tip-to-MCP distance will be much smaller than extended length
  if (expected_finger_length > 0)
  {
    double curl_factor = 1.0 - (dist_tip_to_mcp / expected_finger_length);
    curl_factor = std::max(0.0, curl_factor);
    curl += curl_factor * 2.2;
  }

  // Normalize to [0, 1] range
  double max_theoretical_curl = (M_PI * 2.0 * 3.5) + 1.8;
  curl = std::min(curl / max_theoretical_curl, 1.0);

  // Apply non-linear scaling for better sensitivity
  curl = std::pow(curl, 0.6);

  return curl;
}

void HandLandmarkInterpreter::process_landmarks(const std::vector<geometry_msgs::msg::Pose>& landmarks)
{
  if (landmarks.size() < 21)
  {
    return;
  }

  // Calculate curl (bend) for each finger
  double thumb_curl = calculate_finger_curl(landmarks, THUMB_CMC_IDX, 4);
  double index_curl = calculate_finger_curl(landmarks, INDEX_MCP_IDX, 4);
  double middle_curl = calculate_finger_curl(landmarks, MIDDLE_MCP_IDX, 4);
  double ring_curl = calculate_finger_curl(landmarks, RING_MCP_IDX, 4);
  double pinky_curl = calculate_finger_curl(landmarks, PINKY_MCP_IDX, 4);

  // Lower threshold for more sensitive detection of fist state
  const double CLOSED_THRESHOLD = 0.58;

  // Apply an amplification factor to finger curls (except thumb)
  index_curl = std::min(index_curl * 1.3, 1.0);
  middle_curl = std::min(middle_curl * 1.3, 1.0);
  ring_curl = std::min(ring_curl * 1.3, 1.0);
  pinky_curl = std::min(pinky_curl * 1.3, 1.0);

  // Apply threshold to finger curls
  if (index_curl > CLOSED_THRESHOLD)
    index_curl = 1.0;
  if (middle_curl > CLOSED_THRESHOLD)
    middle_curl = 1.0;
  if (ring_curl > CLOSED_THRESHOLD)
    ring_curl = 1.0;
  if (pinky_curl > CLOSED_THRESHOLD)
    pinky_curl = 1.0;

  // Detect fist gesture
  bool is_fist = (index_curl > CLOSED_THRESHOLD && middle_curl > CLOSED_THRESHOLD && ring_curl > CLOSED_THRESHOLD &&
                  pinky_curl > CLOSED_THRESHOLD);

  // If fingers are close to a fist but not quite there, enhance the curl values
  bool near_fist = (index_curl > 0.4 && middle_curl > 0.4 && ring_curl > 0.4 && pinky_curl > 0.4);
  if (near_fist && !is_fist)
  {
    index_curl = std::min(index_curl + 0.2, 1.0);
    middle_curl = std::min(middle_curl + 0.2, 1.0);
    ring_curl = std::min(ring_curl + 0.2, 1.0);
    pinky_curl = std::min(pinky_curl + 0.2, 1.0);
  }

  // When a fist is detected, ensure all fingers are fully closed
  if (is_fist)
  {
    index_curl = 1.0;
    middle_curl = 1.0;
    ring_curl = 1.0;
    pinky_curl = 1.0;
    thumb_curl = std::max(thumb_curl, 0.9);
  }

  // Calculate thumb opposition
  double thumb_oppose = 0.0;
  if (landmarks.size() >= static_cast<size_t>(THUMB_TIP_IDX + 1))
  {
    // Calculate the thumb opposition using the position relative to the index MCP
    double dx = landmarks[THUMB_TIP_IDX].position.x - landmarks[INDEX_MCP_IDX].position.x;
    double dy = landmarks[THUMB_TIP_IDX].position.y - landmarks[INDEX_MCP_IDX].position.y;

    // Normalize by hand size
    double hand_size = std::sqrt(std::pow(landmarks[MIDDLE_MCP_IDX].position.x - landmarks[WRIST_IDX].position.x, 2) +
                                 std::pow(landmarks[MIDDLE_MCP_IDX].position.y - landmarks[WRIST_IDX].position.y, 2));

    if (hand_size > 0)
    {
      double raw_oppose = std::sqrt(dx * dx + dy * dy) / hand_size;
      // Map the raw opposition to a range that works well for the thumb
      thumb_oppose = std::max(0.0, std::min(1.0 - (raw_oppose * 0.5), 1.0));
    }
  }

  // Thumb processing - both curl and opposition
  if (finger_joints_.count("thumb") > 0)
  {
    // Amplify thumb curl to make it more responsive
    thumb_curl = std::min(thumb_curl * 1.5, 1.0);

    // Set thumb yaw (side-to-side movement)
    if (finger_joints_["thumb"].count("yaw") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["yaw"])
      {
        joint_positions_[joint] = joint_limits_[joint] * thumb_oppose;
      }
    }

    // Set thumb pitch (bending movement)
    if (finger_joints_["thumb"].count("pitch") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["pitch"])
      {
        joint_positions_[joint] = joint_limits_[joint] * thumb_curl;
      }
    }

    // Set thumb flex joints if they exist
    if (finger_joints_["thumb"].count("flex") > 0)
    {
      for (const auto& joint : finger_joints_["thumb"]["flex"])
      {
        joint_positions_[joint] = joint_limits_[joint] * thumb_curl;
      }
    }
  }

  // Set positions for other fingers with scaling for stronger bending
  if (finger_joints_.count("index") > 0)
  {
    set_finger_positions("index", index_curl);
    // Apply extra bending to proximal joints
    for (const auto& role_entry : finger_joints_["index"])
    {
      for (const auto& joint_name : role_entry.second)
      {
        if (joint_name.find("proximal") != std::string::npos)
        {
          double current_value = joint_positions_[joint_name];
          joint_positions_[joint_name] = std::min(current_value * 1.15, joint_limits_[joint_name]);
        }
      }
    }
  }

  // Apply same pattern to middle, ring, and pinky fingers
  if (finger_joints_.count("middle") > 0)
  {
    set_finger_positions("middle", middle_curl);
    for (const auto& role_entry : finger_joints_["middle"])
    {
      for (const auto& joint_name : role_entry.second)
      {
        if (joint_name.find("proximal") != std::string::npos)
        {
          double current_value = joint_positions_[joint_name];
          joint_positions_[joint_name] = std::min(current_value * 1.15, joint_limits_[joint_name]);
        }
      }
    }
  }

  if (finger_joints_.count("ring") > 0)
  {
    set_finger_positions("ring", ring_curl);
    for (const auto& role_entry : finger_joints_["ring"])
    {
      for (const auto& joint_name : role_entry.second)
      {
        if (joint_name.find("proximal") != std::string::npos)
        {
          double current_value = joint_positions_[joint_name];
          joint_positions_[joint_name] = std::min(current_value * 1.15, joint_limits_[joint_name]);
        }
      }
    }
  }

  if (finger_joints_.count("pinky") > 0)
  {
    set_finger_positions("pinky", pinky_curl);
    for (const auto& role_entry : finger_joints_["pinky"])
    {
      for (const auto& joint_name : role_entry.second)
      {
        if (joint_name.find("proximal") != std::string::npos)
        {
          double current_value = joint_positions_[joint_name];
          joint_positions_[joint_name] = std::min(current_value * 1.15, joint_limits_[joint_name]);
        }
      }
    }
  }

  // Apply extra palm closing for fist gesture
  if (is_fist)
  {
    for (const auto& joint_entry : joint_configs_)
    {
      const auto& config = joint_entry.second;
      if (config.role == "palm" || config.role == "closure" || config.name.find("palm") != std::string::npos ||
          config.name.find("close") != std::string::npos)
      {
        joint_positions_[config.name] = joint_limits_[config.name];
      }
    }
  }
}

}  // namespace arm_hand_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandLandmarkInterpreter>());
  rclcpp::shutdown();
  return 0;
}
