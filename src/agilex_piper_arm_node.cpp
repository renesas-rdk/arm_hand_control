#include "arm_hand_control/agilex_piper_arm_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace arm_hand_control
{

AgilexPiperArmNode::AgilexPiperArmNode(const rclcpp::NodeOptions& options) : Node("piper_controller_node", options)
{
  // Declare and get parameters
  declare_and_get_parameters();

  // Resolve the config file path
  std::string config_file_path = resolve_config_file_path(get_parameter("config_file").as_string());

  // Initialize controller components
  initialize_joint_data();
  setup_publishers_and_subscribers();
  initialize_controller();

  // Load configuration and apply limits
  load_and_apply_configuration(config_file_path);

  // Create timer for periodic updates
  update_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / update_frequency_),
                                          std::bind(&AgilexPiperArmNode::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Piper controller node initialized");
  RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface_.c_str());
}

void AgilexPiperArmNode::declare_and_get_parameters()
{
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<double>("update_frequency", 50.0);
  this->declare_parameter<std::string>("config_file", "config/arm/agilex_piper.yaml");

  can_interface_ = this->get_parameter("can_interface").as_string();
  update_frequency_ = this->get_parameter("update_frequency").as_double();
}

std::string AgilexPiperArmNode::resolve_config_file_path(const std::string& config_file)
{
  std::string config_file_path;
  try
  {
    // Check if the path is absolute
    if (config_file[0] == '/')
    {
      config_file_path = config_file;
    }
    else
    {
      // Try to find the file relative to the package share directory
      std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("arm_hand_control");
      config_file_path = std::filesystem::path(pkg_share_dir) / config_file;

      // If that doesn't exist, try the current working directory
      if (!std::filesystem::exists(config_file_path))
      {
        config_file_path = std::filesystem::current_path() / config_file;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Using config file: %s", config_file_path.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error resolving config file path: %s", e.what());
    config_file_path = config_file;  // Fallback to original path
  }
  return config_file_path;
}

void AgilexPiperArmNode::initialize_joint_data()
{
  // Initialize joint names and positions
  joint_names_ = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
  joint_positions_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
}

void AgilexPiperArmNode::setup_publishers_and_subscribers()
{
  // Create publishers
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("piper/current_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("piper/status", 10);

  // Create subscribers
  joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "piper/joint_command", 10, std::bind(&AgilexPiperArmNode::joint_command_callback, this, std::placeholders::_1));
  pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "piper/pose_command", 10, std::bind(&AgilexPiperArmNode::pose_command_callback, this, std::placeholders::_1));
  control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "piper/control_mode", 10, std::bind(&AgilexPiperArmNode::control_mode_callback, this, std::placeholders::_1));
}

void AgilexPiperArmNode::initialize_controller()
{
  controller_ = std::make_unique<agilex::piper::PiperController>(can_interface_);

  if (!controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Will apply SDK joint limits when connected.");
  }
}

void AgilexPiperArmNode::load_and_apply_configuration(const std::string& config_file_path)
{
  if (!load_joint_config(config_file_path))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load joint configuration from %s", config_file_path.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Successfully loaded joint configuration from %s", config_file_path.c_str());
    // Apply the joint limits to the SDK if controller is connected
    if (controller_->is_connected())
    {
      apply_joint_limits_to_sdk();
    }
  }
}

AgilexPiperArmNode::~AgilexPiperArmNode()
{
  // Ensure controller is properly shut down
  if (controller_)
  {
    controller_->disable_arm();
    controller_->disconnect();
  }
  RCLCPP_INFO(this->get_logger(), "Piper controller node shutdown");
}

bool AgilexPiperArmNode::load_joint_config(const std::string& config_file)
{
  try
  {
    if (!std::filesystem::exists(config_file))
    {
      RCLCPP_ERROR(this->get_logger(), "Error loading configuration: file not found: %s", config_file.c_str());
      return false;
    }

    YAML::Node config = YAML::LoadFile(config_file);

    if (!config["arm_config"] || !config["arm_config"]["joints"])
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid configuration file format in %s", config_file.c_str());
      return false;
    }

    // Store joint configuration
    joint_config_.clear();
    YAML::Node joints = config["arm_config"]["joints"];
    for (size_t i = 0; i < joints.size(); i++)
    {
      JointConfig joint;
      joint.name = joints[i]["name"].as<std::string>();
      joint.role = joints[i]["role"].as<std::string>();
      joint.limit_min = joints[i]["limit_min"].as<double>();
      joint.limit_max = joints[i]["limit_max"].as<double>();
      joint.default_position = joints[i]["default_position"].as<double>();

      joint_config_[joint.name] = joint;
      RCLCPP_DEBUG(this->get_logger(), "Loaded joint %s: min=%f, max=%f, default=%f", joint.name.c_str(),
                   joint.limit_min, joint.limit_max, joint.default_position);
    }

    // Get control parameters if available
    if (config["arm_config"]["update_rate"])
    {
      double update_rate = config["arm_config"]["update_rate"].as<double>();
      update_frequency_ = update_rate;
      RCLCPP_INFO(this->get_logger(), "Setting update frequency to %f Hz from config", update_frequency_);
    }

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error loading configuration: %s from file: %s", e.what(), config_file.c_str());
    return false;
  }
}

void AgilexPiperArmNode::apply_joint_limits_to_sdk()
{
  if (!controller_ || !controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot apply SDK joint limits.");
    return;
  }

  for (const auto& [joint_name, config] : joint_config_)
  {
    RCLCPP_INFO(this->get_logger(), "Setting SDK joint limit for %s: min=%f, max=%f", joint_name.c_str(),
                config.limit_min, config.limit_max);
    controller_->set_sdk_joint_limit_param(joint_name, config.limit_min, config.limit_max);
  }

  RCLCPP_INFO(this->get_logger(), "Applied all joint limits to SDK");
}

void AgilexPiperArmNode::update_callback()
{
  if (!controller_ || !controller_->is_connected())
  {
    static bool connection_attempted = false;

    if (!connection_attempted)
    {
      RCLCPP_WARN(this->get_logger(), "Controller not connected, attempting to connect and apply joint limits");
      if (controller_->connect_port())
      {
        RCLCPP_INFO(this->get_logger(), "Controller connected successfully");
        apply_joint_limits_to_sdk();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to connect to controller. Will not retry automatically.");
      }
      connection_attempted = true;  // Mark as attempted regardless of success or failure
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Controller not connected");
    }
    return;
  }

  publish_joint_states();
  publish_end_pose();
  publish_arm_status();
}

void AgilexPiperArmNode::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!controller_ || !controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot send joint command.");
    return;
  }

  // Check if the message contains valid joint data
  if (msg->position.size() < 6)
  {
    RCLCPP_ERROR(this->get_logger(), "Joint command has fewer than 6 position values");
    return;
  }

  // Extract joint angles (in radians) and convert to controller format (0.001 degrees)
  int j1 = static_cast<int>(msg->position[0] * 180.0 / M_PI * 1000.0);
  int j2 = static_cast<int>(msg->position[1] * 180.0 / M_PI * 1000.0);
  int j3 = static_cast<int>(msg->position[2] * 180.0 / M_PI * 1000.0);
  int j4 = static_cast<int>(msg->position[3] * 180.0 / M_PI * 1000.0);
  int j5 = static_cast<int>(msg->position[4] * 180.0 / M_PI * 1000.0);
  int j6 = static_cast<int>(msg->position[5] * 180.0 / M_PI * 1000.0);

  // Send joint command to controller
  if (!controller_->set_joint_angles(j1, j2, j3, j4, j5, j6))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send joint command to controller");
  }
}

void AgilexPiperArmNode::pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!controller_ || !controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot send pose command.");
    return;
  }

  // Extract position in meters and convert to controller format (0.001 mm)
  int x = static_cast<int>(msg->pose.position.x * 1000000.0);
  int y = static_cast<int>(msg->pose.position.y * 1000000.0);
  int z = static_cast<int>(msg->pose.position.z * 1000000.0);

  // Convert quaternion orientation to Euler angles (roll, pitch, yaw) in degrees * 1000
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  int rx = static_cast<int>(roll * 180.0 / M_PI * 1000.0);   // Convert to deg * 1000
  int ry = static_cast<int>(pitch * 180.0 / M_PI * 1000.0);  // Convert to deg * 1000
  int rz = static_cast<int>(yaw * 180.0 / M_PI * 1000.0);    // Convert to deg * 1000

  // Send pose command to controller
  if (!controller_->set_end_pose(x, y, z, rx, ry, rz))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send pose command to controller");
  }
}

void AgilexPiperArmNode::control_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!controller_ || !controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot set control mode.");
    return;
  }

  // Parse the control mode message
  std::string mode = msg->data;

  if (mode == "enable")
  {
    controller_->enable_arm();
    RCLCPP_INFO(this->get_logger(), "Enabled all joints");
  }
  else if (mode == "disable")
  {
    controller_->disable_arm();
    RCLCPP_INFO(this->get_logger(), "Disabled all joints");
  }
  else if (mode == "emergency_stop")
  {
    // Emergency stop
    controller_->motion_control_1(0x01);  // 0x01 = emergency stop
    RCLCPP_INFO(this->get_logger(), "Emergency stop activated");
  }
  else if (mode == "resume_emergency")
  {
    // Resume emergency stop
    controller_->motion_control_1(0x02);  // 0x02 = resume emergency stop
    RCLCPP_INFO(this->get_logger(), "Emergency stop resume");
  }
  else if (mode == "joint_mode")
  {
    // Set to joint control mode
    controller_->set_mode(0x01, 0x01, 50);
    // 0x01 = position control mode, 0x01 = joint mode, 50 = speed rate (50%)
    RCLCPP_INFO(this->get_logger(), "Set to joint control mode");
  }
  else if (mode == "cartesian_mode")
  {
    // Set to Cartesian control mode
    controller_->set_mode(0x01, 0x00, 50);
    // 0x01 = position control mode, 0x00 = Cartesian mode, 50 = speed rate (50%)
    RCLCPP_INFO(this->get_logger(), "Set to Cartesian control mode");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown control mode: %s", mode.c_str());
  }
}

void AgilexPiperArmNode::publish_joint_states()
{
  // Get the current joint angles from the controller
  agilex::piper::ArmJoint arm_joint = controller_->get_arm_joint();

  // Convert from controller format (0.001 degree) to radians
  joint_positions_[0] = arm_joint.j1 * 0.001 * M_PI / 180.0;
  joint_positions_[1] = arm_joint.j2 * 0.001 * M_PI / 180.0;
  joint_positions_[2] = arm_joint.j3 * 0.001 * M_PI / 180.0;
  joint_positions_[3] = arm_joint.j4 * 0.001 * M_PI / 180.0;
  joint_positions_[4] = arm_joint.j5 * 0.001 * M_PI / 180.0;
  joint_positions_[5] = arm_joint.j6 * 0.001 * M_PI / 180.0;

  // Create and publish joint state message
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = this->now();
  joint_state_msg->name = joint_names_;
  joint_state_msg->position = joint_positions_;

  joint_state_pub_->publish(std::move(joint_state_msg));
}

void AgilexPiperArmNode::publish_end_pose()
{
  // Get the current end pose from the controller
  agilex::piper::ArmEndPose end_pose = controller_->get_arm_end_pose();

  // Create a new PoseStamped message
  auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

  // Set header
  pose_msg->header.stamp = this->now();
  pose_msg->header.frame_id = "base_link";  // Set appropriate frame ID

  // Position: convert from 0.001 mm to meters
  pose_msg->pose.position.x = end_pose.x * 0.000001;
  pose_msg->pose.position.y = end_pose.y * 0.000001;
  pose_msg->pose.position.z = end_pose.z * 0.000001;

  // Orientation: convert from degrees*1000 to quaternion
  double roll = end_pose.rx * 0.001 * M_PI / 180.0;   // Convert to radians
  double pitch = end_pose.ry * 0.001 * M_PI / 180.0;  // Convert to radians
  double yaw = end_pose.rz * 0.001 * M_PI / 180.0;    // Convert to radians

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  pose_msg->pose.orientation.x = q.x();
  pose_msg->pose.orientation.y = q.y();
  pose_msg->pose.orientation.z = q.z();
  pose_msg->pose.orientation.w = q.w();

  pose_pub_->publish(std::move(pose_msg));
}

void AgilexPiperArmNode::publish_arm_status()
{
  // Get the current arm status from the controller
  agilex::piper::ArmStatus status = controller_->get_arm_status();

  // Create a string representation of the status
  std::ostringstream oss;
  oss << "Control Mode: " << static_cast<int>(status.ctrl_mode) << ", ";
  oss << "Arm Status: " << static_cast<int>(status.arm_status) << ", ";
  oss << "Mode Feedback: " << static_cast<int>(status.mode_feed) << ", ";
  oss << "Teach Status: " << static_cast<int>(status.teach_status) << ", ";
  oss << "Motion Status: " << static_cast<int>(status.motion_status) << ", ";
  oss << "Trajectory Number: " << static_cast<int>(status.trajectory_num) << ", ";
  oss << "Communication Error: " << static_cast<int>(status.err_code_comm) << ", ";
  oss << "Angle Error: " << static_cast<int>(status.err_code_angle);

  auto status_msg = std::make_unique<std_msgs::msg::String>();
  status_msg->data = oss.str();

  status_pub_->publish(std::move(status_msg));

  // Also log status if there's an error
  if (status.err_code_comm != 0 || status.err_code_angle != 0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Arm status error: %s", oss.str().c_str());
  }
}

}  // namespace arm_hand_control

// Main entry point for the ROS node
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_hand_control::AgilexPiperArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}