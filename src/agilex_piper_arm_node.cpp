#include "arm_hand_control/agilex_piper_arm_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <numeric>
#include <sstream>

using namespace std::chrono_literals;

namespace arm_hand_control
{

AgilexPiperArmNode::AgilexPiperArmNode(const rclcpp::NodeOptions& options) : Node("piper_controller_node", options)
{
  // Declare parameters with default values
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<double>("update_frequency", 50.0);
  this->declare_parameter<std::string>("config_file", "config/arm/agilex_piper.yaml");
  this->declare_parameter<bool>("arm_enabled", false);
  this->declare_parameter<int>("motion_mode", 1);  // 0=Cartesian, 1=Joint
  this->declare_parameter<bool>("listen_only", false);

  // Get parameter values
  can_interface_ = this->get_parameter("can_interface").as_string();
  update_frequency_ = this->get_parameter("update_frequency").as_double();
  arm_enabled_ = this->get_parameter("arm_enabled").as_bool();
  motion_mode_ = this->get_parameter("motion_mode").as_int();
  listen_only_ = this->get_parameter("listen_only").as_bool();

  // Initialize joint names and positions
  joint_names_.reserve(NUM_JOINTS);
  joint_positions_.reserve(NUM_JOINTS);
  for (size_t i = 1; i <= NUM_JOINTS; ++i)
  {
    joint_names_.emplace_back("joint" + std::to_string(i));
    joint_positions_.emplace_back(0.0);
  }

  // Create publishers
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("piper/joint_states", 10);
  gripper_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("piper/gripper_joint_states", 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("piper/current_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("piper/status", 10);

  // Create subscribers
  joint_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "piper/joint_command", 10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { joint_command_callback(msg); });

  pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "piper/pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_command_callback(msg); });

  gripper_cmd_sub_ = this->create_subscription<control_msgs::msg::GripperCommand>(
      "piper/gripper_command", 10,
      [this](const control_msgs::msg::GripperCommand::SharedPtr msg) { gripper_command_callback(msg); });

  // Initialize controller
  controller_ = std::make_unique<agilex::piper::PiperController>(can_interface_);

  // Load configuration
  const std::string config_file_path = resolve_config_file_path(get_parameter("config_file").as_string());
  if (!load_joint_config(config_file_path))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load joint configuration from %s", config_file_path.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Successfully loaded joint configuration from %s", config_file_path.c_str());
  }

  // Setup controller connection
  if (!setup_controller_connection())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Will attempt connection during operation.");
  }

  // Register parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) { return on_set_parameters_callback(params); });

  // Create update timer
  update_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / update_frequency_), [this]() { update_callback(); });

  RCLCPP_INFO(this->get_logger(), "Piper controller node initialized");
  RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface_.c_str());
  if (listen_only_)
  {
    RCLCPP_INFO(this->get_logger(), "Running in listen-only mode. Commands will be received but not executed.");
  }
}

AgilexPiperArmNode::~AgilexPiperArmNode()
{
  if (controller_)
  {
    disable_arm_and_gripper();
    controller_->disconnect();
  }
  RCLCPP_INFO(this->get_logger(), "Piper controller node shutdown");
}

bool AgilexPiperArmNode::setup_controller_connection()
{
  if (!controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Will apply SDK joint limits when connected.");
    return false;
  }

  apply_joint_limits_to_sdk();

  if (!listen_only_)
  {
    if (arm_enabled_)
    {
      enable_arm_and_gripper();
    }
    else
    {
      disable_arm_and_gripper();
    }
    apply_motion_mode();
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "In listen-only mode: arm settings not applied on connection");
  }

  return true;
}

void AgilexPiperArmNode::enable_arm_and_gripper()
{
  if (controller_ && controller_->is_connected())
  {
    controller_->enable_arm();
    controller_->control_gripper(0, DEFAULT_GRIPPER_EFFORT, GRIPPER_ENABLE, 0x00);
    RCLCPP_INFO(this->get_logger(), "Arm and gripper enabled");
  }
}

void AgilexPiperArmNode::disable_arm_and_gripper()
{
  if (controller_ && controller_->is_connected())
  {
    controller_->disable_arm();
    controller_->control_gripper(0, DEFAULT_GRIPPER_EFFORT, GRIPPER_DISABLE_CLEAR, 0x00);
    RCLCPP_INFO(this->get_logger(), "Arm and gripper disabled");
  }
}

bool AgilexPiperArmNode::is_controller_ready() const
{
  return controller_ && controller_->is_connected();
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

rcl_interfaces::msg::SetParametersResult
AgilexPiperArmNode::on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto& param : parameters)
  {
    if (param.get_name() == "arm_enabled")
    {
      const bool new_state = param.as_bool();
      if (new_state != arm_enabled_)
      {
        arm_enabled_ = new_state;
        RCLCPP_INFO(this->get_logger(), "Arm %s", arm_enabled_ ? "enabled" : "disabled");

        if (!listen_only_ && is_controller_ready())
        {
          if (arm_enabled_)
          {
            enable_arm_and_gripper();
          }
          else
          {
            disable_arm_and_gripper();
          }
          apply_motion_mode();
        }
        else if (listen_only_)
        {
          RCLCPP_INFO(this->get_logger(), "In listen-only mode: arm enable/disable command not executed");
        }
      }
    }
    else if (param.get_name() == "motion_mode")
    {
      const int new_mode = param.as_int();
      if (new_mode != motion_mode_ && (new_mode == 0 || new_mode == 1))
      {
        motion_mode_ = new_mode;
        RCLCPP_INFO(this->get_logger(), "Motion mode set to %s", motion_mode_ == 0 ? "Cartesian" : "Joint");

        if (!listen_only_)
        {
          apply_motion_mode();
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "In listen-only mode: motion mode change not applied");
        }
      }
      else if (new_mode != 0 && new_mode != 1)
      {
        result.successful = false;
        result.reason = "Invalid motion mode. Valid values are 0 (Cartesian) and 1 (Joint).";
      }
    }
    else if (param.get_name() == "listen_only")
    {
      const bool new_state = param.as_bool();
      if (new_state != listen_only_)
      {
        const bool was_listen_only = listen_only_;
        listen_only_ = new_state;
        RCLCPP_INFO(this->get_logger(), "Listen-only mode %s", listen_only_ ? "enabled" : "disabled");

        // Apply pending commands when transitioning from listen-only to active mode
        if (was_listen_only && !listen_only_ && is_controller_ready())
        {
          RCLCPP_INFO(this->get_logger(), "Transitioning from listen-only mode, applying pending commands");
          if (arm_enabled_)
          {
            enable_arm_and_gripper();
          }
          else
          {
            disable_arm_and_gripper();
          }
          apply_motion_mode();
        }
      }
    }
  }

  return result;
}

void AgilexPiperArmNode::apply_motion_mode()
{
  // Set to joint or Cartesian motion mode
  // 0x01 = CAN control mode
  // motion_mode_ = 0 (Cartesian mode) or 1 (Joint mode)
  // 100 = speed rate (100%)
  controller_->set_mode(0x01, motion_mode_, 100, 0);
  RCLCPP_INFO(this->get_logger(), "Set to %s motion mode", motion_mode_ == 0 ? "Cartesian" : "joint");
}

void AgilexPiperArmNode::update_callback()
{
  if (!is_controller_ready())
  {
    static bool connection_attempted = false;

    if (!connection_attempted)
    {
      RCLCPP_WARN(this->get_logger(), "Controller not connected, attempting to connect and apply joint limits");
      if (controller_->connect_port())
      {
        RCLCPP_INFO(this->get_logger(), "Controller connected successfully");
        if (!setup_controller_connection())
        {
          RCLCPP_WARN(this->get_logger(), "Failed to setup controller after connection");
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to connect to controller. Will not retry automatically.");
      }
      connection_attempted = true;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Controller not connected");
    }
    return;
  }

  // Publish current state
  publish_joint_states();
  publish_gripper_joint_states();
  publish_end_pose();
  publish_arm_status();
}

bool AgilexPiperArmNode::validate_joint_command(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) const
{
  if (msg->points.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Joint trajectory command has no points");
    return false;
  }

  if (msg->joint_names.size() < NUM_JOINTS)
  {
    RCLCPP_ERROR(this->get_logger(), "Joint trajectory command has fewer than %zu joint names", NUM_JOINTS);
    return false;
  }

  if (msg->points[0].positions.size() < NUM_JOINTS)
  {
    RCLCPP_ERROR(this->get_logger(), "Joint trajectory command has fewer than %zu position values", NUM_JOINTS);
    return false;
  }

  return true;
}

void AgilexPiperArmNode::joint_command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!is_controller_ready())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot send joint command.");
    return;
  }

  if (listen_only_)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Received joint command but running in listen-only mode. Command ignored.");
    return;
  }

  if (!validate_joint_command(msg))
  {
    return;
  }

  const auto& point = msg->points[0];

  // Find correct joint indices
  std::vector<size_t> joint_indices(NUM_JOINTS);
  std::iota(joint_indices.begin(), joint_indices.end(), 0);  // Default sequential ordering

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    const auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_names_[i]);
    if (it != msg->joint_names.end())
    {
      joint_indices[i] = std::distance(msg->joint_names.begin(), it);
    }
  }

  // Convert joint angles from radians to controller format (0.001 degrees)
  std::array<int, NUM_JOINTS> joint_commands;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joint_commands[i] = static_cast<int>(point.positions[joint_indices[i]] * 180.0 / M_PI * 1000.0);
  }

  // Send command to controller
  if (!controller_->set_joint_angles(joint_commands[0], joint_commands[1], joint_commands[2], joint_commands[3],
                                     joint_commands[4], joint_commands[5]))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send joint command to controller");
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "Sent joint command to controller: [%d, %d, %d, %d, %d, %d]", joint_commands[0],
                 joint_commands[1], joint_commands[2], joint_commands[3], joint_commands[4], joint_commands[5]);
  }
}

void AgilexPiperArmNode::pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!controller_ || !controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot send pose command.");
    return;
  }

  if (listen_only_)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Received pose command but running in listen-only mode. Command ignored.");
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

void AgilexPiperArmNode::gripper_command_callback(const control_msgs::msg::GripperCommand::SharedPtr msg)
{
  if (!is_controller_ready())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot send gripper command.");
    return;
  }

  if (listen_only_)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Received gripper command but running in listen-only mode. Command ignored.");
    return;
  }

  // Convert position from meters to controller format (0.001mm units)
  const int grippers_angle = static_cast<int>(msg->position * 1000000.0);

  // Convert and clamp effort
  const uint16_t grippers_effort = static_cast<uint16_t>(std::clamp(msg->max_effort * 1000.0, 0.0, 5000.0));

  // Use default effort if not specified
  const uint16_t final_effort = (grippers_effort == 0) ? DEFAULT_GRIPPER_EFFORT : grippers_effort;

  if (!controller_->control_gripper(grippers_angle, final_effort, GRIPPER_ENABLE, 0x00))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send gripper command to controller");
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "Sent gripper command: position=%d (0.001mm), effort=%d (0.001N·m)",
                 grippers_angle, final_effort);
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

void AgilexPiperArmNode::publish_gripper_joint_states()
{
  // Get the current gripper state from the controller
  agilex::piper::ArmGripper gripper_state = controller_->get_arm_gripper();

  // Create and publish gripper joint state message
  auto gripper_joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  gripper_joint_state_msg->header.stamp = this->now();

  // Use generic gripper joint names - you may want to customize these
  gripper_joint_state_msg->name = { "gripper_joint" };

  // Convert from controller format (0.001 units) to appropriate units
  // Note: You may need to adjust this conversion based on your gripper type
  gripper_joint_state_msg->position = { gripper_state.grippers_angle * 0.001 };

  // Add effort information (converted from 0.001 N·m to N·m)
  gripper_joint_state_msg->effort = { gripper_state.grippers_effort * 0.001 };

  // Add velocity (if available - setting to 0 for now)
  gripper_joint_state_msg->velocity = { 0.0 };

  gripper_joint_state_pub_->publish(std::move(gripper_joint_state_msg));
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

  // The arm could be in the teaching mode triggered by the user pressing the teach button
  if ((status.ctrl_mode == 0x02) && arm_enabled_)
  {
    arm_enabled_ = false;
    this->set_parameter(rclcpp::Parameter("arm_enabled", arm_enabled_));
    RCLCPP_WARN(this->get_logger(), "Arm disabled due to teaching mode triggered: %d", status.ctrl_mode);
    RCLCPP_WARN(this->get_logger(), "Set parameter arm_enabled back to true to recover from teaching mode!");
  }

  // Pack status fields into UInt8MultiArray
  auto status_msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
  status_msg->data = { static_cast<uint8_t>(status.ctrl_mode),     static_cast<uint8_t>(status.arm_status),
                       static_cast<uint8_t>(status.mode_feed),     static_cast<uint8_t>(status.teach_status),
                       static_cast<uint8_t>(status.motion_status), static_cast<uint8_t>(status.trajectory_num),
                       static_cast<uint8_t>(status.err_code_comm), static_cast<uint8_t>(status.err_code_angle) };

  status_pub_->publish(std::move(status_msg));

  // Also log status if there's an error
  if (status.err_code_comm != 0 || status.err_code_angle != 0)
  {
    std::ostringstream oss;
    oss << "Arm status error - Control Mode: " << static_cast<int>(status.ctrl_mode)
        << ", Communication Error: " << static_cast<int>(status.err_code_comm)
        << ", Angle Error: " << static_cast<int>(status.err_code_angle);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s", oss.str().c_str());
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