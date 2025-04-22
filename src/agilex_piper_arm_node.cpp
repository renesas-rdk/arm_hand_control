#include "arm_hand_control/agilex_piper_arm_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <sstream>
#include <chrono>

using namespace std::chrono_literals;

namespace arm_hand_control
{

AgilexPiperArmNode::AgilexPiperArmNode(const rclcpp::NodeOptions& options) : Node("piper_controller_node", options)
{
  // Declare and get parameters
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<double>("update_frequency", 50.0);

  can_interface_ = this->get_parameter("can_interface").as_string();
  update_frequency_ = this->get_parameter("update_frequency").as_double();

  // Initialize joint names and positions
  joint_names_ = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
  joint_positions_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Create publishers
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("piper/end_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("piper/status", 10);

  // Create subscribers
  joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "piper/joint_command", 10, std::bind(&AgilexPiperArmNode::joint_command_callback, this, std::placeholders::_1));
  pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "piper/pose_command", 10, std::bind(&AgilexPiperArmNode::pose_command_callback, this, std::placeholders::_1));
  control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "piper/control_mode", 10, std::bind(&AgilexPiperArmNode::control_mode_callback, this, std::placeholders::_1));

  // Initialize controller
  controller_ = std::make_unique<agilex::piper::PiperController>(can_interface_);

  // Create timer for periodic updates
  update_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / update_frequency_),
                                          std::bind(&AgilexPiperArmNode::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Piper controller node initialized");
  RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface_.c_str());
}

AgilexPiperArmNode::~AgilexPiperArmNode()
{
  // Ensure controller is properly shut down
  if (controller_)
  {
    controller_->disconnect();
  }
  RCLCPP_INFO(this->get_logger(), "Piper controller node shutdown");
}

void AgilexPiperArmNode::update_callback()
{
  if (!controller_ || !controller_->is_connected())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Controller not connected");
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

  // Extract joint angles (in radians) and convert to controller format (0.001 rad)
  int j1 = static_cast<int>(msg->position[0] * 1000.0);
  int j2 = static_cast<int>(msg->position[1] * 1000.0);
  int j3 = static_cast<int>(msg->position[2] * 1000.0);
  int j4 = static_cast<int>(msg->position[3] * 1000.0);
  int j5 = static_cast<int>(msg->position[4] * 1000.0);
  int j6 = static_cast<int>(msg->position[5] * 1000.0);

  // Send joint command to controller
  if (!controller_->set_joint_angles(j1, j2, j3, j4, j5, j6))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send joint command to controller");
  }
}

void AgilexPiperArmNode::pose_command_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  if (!controller_ || !controller_->is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "Controller not connected. Cannot send pose command.");
    return;
  }

  // Extract position in meters and convert to controller format (0.001 mm)
  int x = static_cast<int>(msg->position.x * 1000000.0);
  int y = static_cast<int>(msg->position.y * 1000000.0);
  int z = static_cast<int>(msg->position.z * 1000000.0);

  // Convert quaternion orientation to Euler angles (roll, pitch, yaw) in degrees * 1000
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
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
    // Enable all joints
    controller_->enable_arm(0x07);  // 0x07 = all joints
    RCLCPP_INFO(this->get_logger(), "Enabled all joints");
  }
  else if (mode == "disable")
  {
    // Disable all joints
    controller_->disable_arm(0x07);  // 0x07 = all joints
    RCLCPP_INFO(this->get_logger(), "Disabled all joints");
  }
  else if (mode == "emergency_stop")
  {
    // Emergency stop
    controller_->emergency_stop(0x01);  // 0x01 = emergency stop
    RCLCPP_INFO(this->get_logger(), "Emergency stop activated");
  }
  else if (mode == "reset_emergency")
  {
    // Reset emergency stop
    controller_->emergency_stop(0x02);  // 0x02 = reset emergency stop
    RCLCPP_INFO(this->get_logger(), "Emergency stop reset");
  }
  else if (mode == "joint_mode")
  {
    // Set to joint control mode
    controller_->set_mode(0x01, 0x01, 50, 0x00);
    // 0x01 = position control mode, 0x01 = joint mode, 50 = speed rate (50%), 0x00 = not MIT mode
    RCLCPP_INFO(this->get_logger(), "Set to joint control mode");
  }
  else if (mode == "cartesian_mode")
  {
    // Set to Cartesian control mode
    controller_->set_mode(0x01, 0x00, 50, 0x00);
    // 0x01 = position control mode, 0x00 = Cartesian mode, 50 = speed rate (50%), 0x00 = not MIT mode
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

  // Convert from controller format (0.001 rad) to radians
  joint_positions_[0] = arm_joint.j1 * 0.001;
  joint_positions_[1] = arm_joint.j2 * 0.001;
  joint_positions_[2] = arm_joint.j3 * 0.001;
  joint_positions_[3] = arm_joint.j4 * 0.001;
  joint_positions_[4] = arm_joint.j5 * 0.001;
  joint_positions_[5] = arm_joint.j6 * 0.001;

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

  // Convert from controller format to ROS format
  auto pose_msg = std::make_unique<geometry_msgs::msg::Pose>();

  // Position: convert from 0.001 mm to meters
  pose_msg->position.x = end_pose.x * 0.000001;
  pose_msg->position.y = end_pose.y * 0.000001;
  pose_msg->position.z = end_pose.z * 0.000001;

  // Orientation: convert from degrees*1000 to quaternion
  double roll = end_pose.rx * 0.001 * M_PI / 180.0;   // Convert to radians
  double pitch = end_pose.ry * 0.001 * M_PI / 180.0;  // Convert to radians
  double yaw = end_pose.rz * 0.001 * M_PI / 180.0;    // Convert to radians

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  pose_msg->orientation.x = q.x();
  pose_msg->orientation.y = q.y();
  pose_msg->orientation.z = q.z();
  pose_msg->orientation.w = q.w();

  pose_pub_->publish(std::move(pose_msg));
}

void AgilexPiperArmNode::publish_arm_status()
{
  // Get the current arm status from the controller
  agilex::piper::ArmStatus status = controller_->get_arm_status();

  // Create a string representation of the status
  std::ostringstream oss;
  oss << "Mode: " << static_cast<int>(status.arm_mode) << ", ";
  oss << "Work mode: " << static_cast<int>(status.arm_work_mode) << ", ";
  oss << "Motor state: " << static_cast<int>(status.arm_motor_state) << ", ";
  oss << "Error state: " << static_cast<int>(status.arm_err_state) << ", ";
  oss << "Emergency stop: " << static_cast<int>(status.arm_emergency_stop_state) << ", ";
  oss << "Teach state: " << static_cast<int>(status.arm_teach_state) << ", ";
  oss << "Collision: " << static_cast<int>(status.arm_collision_state) << ", ";
  oss << "Servo: " << static_cast<int>(status.arm_servo_state);

  auto status_msg = std::make_unique<std_msgs::msg::String>();
  status_msg->data = oss.str();

  status_pub_->publish(std::move(status_msg));

  // Also log status if there's an error
  if (status.arm_err_state != 0 || status.arm_emergency_stop_state != 0 || status.arm_collision_state != 0)
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