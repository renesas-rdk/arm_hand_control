// Standard includes
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <fstream>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// Project includes
#include "arm_hand_control/teleop_twist_controller_node.hpp"

namespace arm_hand_control
{

TeleopTwistControllerNode::TeleopTwistControllerNode(const rclcpp::NodeOptions& options)
  : Node("teleop_twist_controller_node", options), have_pose_(false), have_joints_(false)
{
  // Declare and get parameters
  this->declare_parameter("control_mode", CONTROL_MODE_JOINT);
  this->declare_parameter("linear_scale", 0.01);                           // m per unit twist
  this->declare_parameter("angular_scale", 0.01);                          // rad per unit twist
  this->declare_parameter("joint_vel_scale", 0.01);                        // rad per unit twist for joints
  this->declare_parameter("config_file", "config/arm/agilex_piper.yaml");  // Path to configuration file

  control_mode_ = this->get_parameter("control_mode").as_string();
  linear_scale_ = this->get_parameter("linear_scale").as_double();
  angular_scale_ = this->get_parameter("angular_scale").as_double();
  joint_vel_scale_ = this->get_parameter("joint_vel_scale").as_double();
  std::string config_file = this->get_parameter("config_file").as_string();

  // Set up parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TeleopTwistControllerNode::parameter_callback, this, std::placeholders::_1));

  // Initialize joints from configuration file
  load_joint_config(config_file);

  // Create subscribers
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TeleopTwistControllerNode::twist_callback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "current_pose", 10, std::bind(&TeleopTwistControllerNode::pose_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&TeleopTwistControllerNode::joint_state_callback, this, std::placeholders::_1));

  // Create publishers
  pose_cmd_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_command", 10);
  joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
  control_mode_pub_ = this->create_publisher<std_msgs::msg::String>("control_mode", 10);

  // Set initial control mode
  set_control_mode(control_mode_);

  RCLCPP_INFO(this->get_logger(), "Teleop twist controller node initialized in %s mode", control_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Linear scale: %.2f, Angular scale: %.2f, Joint vel scale: %.2f", linear_scale_,
              angular_scale_, joint_vel_scale_);
}

void TeleopTwistControllerNode::load_joint_config(const std::string& config_file_name)
{
  try
  {
    std::string config_path;

    // Check if the path is absolute
    if (config_file_name[0] == '/')
    {
      config_path = config_file_name;
    }
    else
    {
      // If not, assume it's relative to the package share directory
      config_path = ament_index_cpp::get_package_share_directory("arm_hand_control") + "/" + config_file_name;
    }
    RCLCPP_INFO(this->get_logger(), "Loading joint configuration from: %s", config_path.c_str());

    // Load the YAML file
    YAML::Node config = YAML::LoadFile(config_path);

    if (!config["arm_config"] || !config["arm_config"]["joints"])
    {
      throw std::runtime_error("Missing arm_config or joints section in config file");
    }

    auto joints = config["arm_config"]["joints"];

    // Clear existing joint data
    current_joints_.name.clear();
    current_joints_.position.clear();

    // Populate joint names and default positions from the config
    for (const auto& joint : joints)
    {
      if (joint["name"] && joint["default_position"])
      {
        current_joints_.name.push_back(joint["name"].as<std::string>());
        current_joints_.position.push_back(joint["default_position"].as<double>());

        RCLCPP_DEBUG(this->get_logger(), "Loaded joint: %s, default position: %.2f",
                     joint["name"].as<std::string>().c_str(), joint["default_position"].as<double>());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Skipping joint with missing name or default position");
      }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu joints from configuration", current_joints_.name.size());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load joint configuration: %s", e.what());

    // Fallback to default joint configuration
    RCLCPP_WARN(this->get_logger(), "Using default joint configuration");
    current_joints_.name = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
    current_joints_.position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }
}

void TeleopTwistControllerNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (control_mode_ == CONTROL_MODE_CARTESIAN && have_pose_)
  {
    // Cartesian mode - use current pose as starting point and modify based on twist
    geometry_msgs::msg::PoseStamped new_pose = current_pose_;

    // Apply linear velocities
    new_pose.pose.position.x += msg->linear.x * linear_scale_;
    new_pose.pose.position.y += msg->linear.y * linear_scale_;
    new_pose.pose.position.z += msg->linear.z * linear_scale_;

    // Convert current orientation to RPY
    tf2::Quaternion q(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y,
                      current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Apply angular velocities
    roll += msg->angular.x * angular_scale_;
    pitch += msg->angular.y * angular_scale_;
    yaw += msg->angular.z * angular_scale_;

    // Convert back to quaternion
    q.setRPY(roll, pitch, yaw);
    new_pose.pose.orientation.x = q.x();
    new_pose.pose.orientation.y = q.y();
    new_pose.pose.orientation.z = q.z();
    new_pose.pose.orientation.w = q.w();

    // Update timestamp
    new_pose.header.stamp = this->now();
    new_pose.header.frame_id = current_pose_.header.frame_id.empty() ? "base_link" : current_pose_.header.frame_id;

    publish_pose_command(new_pose);
  }
  else if (control_mode_ == CONTROL_MODE_JOINT && have_joints_)
  {
    // Joint mode - use twist to directly control joint positions
    sensor_msgs::msg::JointState new_joints = current_joints_;

    // Map twist components to joint changes (this is a simplified mapping)
    if (new_joints.position.size() >= 6)
    {
      new_joints.position[0] += msg->angular.z * joint_vel_scale_;  // base rotation from angular z
      new_joints.position[1] += msg->linear.z * joint_vel_scale_;   // shoulder from linear z
      new_joints.position[2] += msg->linear.y * joint_vel_scale_;   // elbow from linear y
      new_joints.position[3] += msg->linear.x * joint_vel_scale_;   // wrist1 from linear x
      new_joints.position[4] += msg->angular.y * joint_vel_scale_;  // wrist2 from angular y
      new_joints.position[5] += msg->angular.x * joint_vel_scale_;  // wrist3 from angular x

      publish_joint_command(new_joints);
    }
  }
  else
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Cannot process twist command: %s feedback not available or invalid mode",
                         control_mode_ == CONTROL_MODE_CARTESIAN ? "pose" : "joint state");
  }
}

void TeleopTwistControllerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  have_pose_ = true;
}

void TeleopTwistControllerNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Check if we have all the joints we need
  if (msg->name.size() >= 6 && msg->position.size() >= 6)
  {
    // Find and map the right joints by name
    std::vector<size_t> indices(6, std::string::npos);
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      for (size_t j = 0; j < current_joints_.name.size(); ++j)
      {
        if (msg->name[i] == current_joints_.name[j])
        {
          indices[j] = i;
          break;
        }
      }
    }

    // Update joint positions if all joints were found
    bool all_joints_found = true;
    for (const auto& idx : indices)
    {
      if (idx == std::string::npos)
      {
        all_joints_found = false;
        break;
      }
    }

    if (all_joints_found)
    {
      for (size_t i = 0; i < indices.size(); ++i)
      {
        current_joints_.position[i] = msg->position[indices[i]];
      }
      have_joints_ = true;
    }
  }
}

void TeleopTwistControllerNode::set_control_mode(const std::string& mode)
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = mode;
  control_mode_pub_->publish(std::move(msg));
  control_mode_ = mode;

  RCLCPP_INFO(this->get_logger(), "Control mode set to: %s", mode.c_str());
}

void TeleopTwistControllerNode::publish_pose_command(const geometry_msgs::msg::PoseStamped& pose)
{
  auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  *msg = pose;

  // Ensure the message has a valid timestamp and frame id
  if (msg->header.stamp == rclcpp::Time(0))
  {
    msg->header.stamp = this->now();
  }
  if (msg->header.frame_id.empty())
  {
    msg->header.frame_id = "base_link";
  }

  pose_cmd_pub_->publish(std::move(msg));
}

void TeleopTwistControllerNode::publish_joint_command(const sensor_msgs::msg::JointState& joint_state)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();
  msg->name = joint_state.name;
  msg->position = joint_state.position;
  joint_cmd_pub_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult
TeleopTwistControllerNode::parameter_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto& param : parameters)
  {
    if (param.get_name() == "control_mode")
    {
      std::string new_mode = param.as_string();
      if (new_mode == CONTROL_MODE_CARTESIAN || new_mode == CONTROL_MODE_JOINT)
      {
        set_control_mode(new_mode);
      }
      else
      {
        result.successful = false;
        result.reason = "Invalid control mode. Use 'cartesian_mode' or 'joint_mode'";
      }
    }
    else if (param.get_name() == "linear_scale")
    {
      linear_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated linear scale to: %.2f", linear_scale_);
    }
    else if (param.get_name() == "angular_scale")
    {
      angular_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated angular scale to: %.2f", angular_scale_);
    }
    else if (param.get_name() == "joint_vel_scale")
    {
      joint_vel_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated joint velocity scale to: %.2f", joint_vel_scale_);
    }
  }

  return result;
}

}  // namespace arm_hand_control

// Main entry point for the ROS node
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_hand_control::TeleopTwistControllerNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
