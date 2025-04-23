// Standard includes
#include <cmath>
#include <string>
#include <vector>
#include <memory>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Project includes
#include "arm_hand_control/piper_teleop_node.hpp"

namespace arm_hand_control
{

// Constant definitions for control modes
const std::string CONTROL_MODE_CARTESIAN = "cartesian_mode";
const std::string CONTROL_MODE_JOINT = "joint_mode";

PiperTeleopNode::PiperTeleopNode(const rclcpp::NodeOptions& options)
  : Node("piper_teleop_node", options), have_pose_(false), have_joints_(false)
{
  // Declare and get parameters
  this->declare_parameter("control_mode", CONTROL_MODE_CARTESIAN);
  this->declare_parameter("linear_scale", 0.1);      // m per unit twist
  this->declare_parameter("angular_scale", 0.1);     // rad per unit twist
  this->declare_parameter("joint_vel_scale", 0.05);  // rad per unit twist for joints

  control_mode_ = this->get_parameter("control_mode").as_string();
  linear_scale_ = this->get_parameter("linear_scale").as_double();
  angular_scale_ = this->get_parameter("angular_scale").as_double();
  joint_vel_scale_ = this->get_parameter("joint_vel_scale").as_double();

  // Initialize current_joints_
  current_joints_.name = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
  current_joints_.position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Create subscribers
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "piper/cmd_vel", 10, std::bind(&PiperTeleopNode::twist_callback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "piper/end_pose", 10, std::bind(&PiperTeleopNode::pose_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&PiperTeleopNode::joint_state_callback, this, std::placeholders::_1));

  // Create publishers
  pose_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("piper/pose_command", 10);
  joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("piper/joint_command", 10);
  control_mode_pub_ = this->create_publisher<std_msgs::msg::String>("piper/control_mode", 10);

  // Set initial control mode
  set_control_mode(control_mode_);

  RCLCPP_INFO(this->get_logger(), "Piper teleop node initialized in %s mode", control_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Linear scale: %.2f, Angular scale: %.2f, Joint vel scale: %.2f", linear_scale_,
              angular_scale_, joint_vel_scale_);
}

void PiperTeleopNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (control_mode_ == CONTROL_MODE_CARTESIAN && have_pose_)
  {
    // Cartesian mode - use current pose as starting point and modify based on twist
    geometry_msgs::msg::Pose new_pose = current_pose_;

    // Apply linear velocities
    new_pose.position.x += msg->linear.x * linear_scale_;
    new_pose.position.y += msg->linear.y * linear_scale_;
    new_pose.position.z += msg->linear.z * linear_scale_;

    // Convert current orientation to RPY
    tf2::Quaternion q(current_pose_.orientation.x, current_pose_.orientation.y, current_pose_.orientation.z,
                      current_pose_.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Apply angular velocities
    roll += msg->angular.x * angular_scale_;
    pitch += msg->angular.y * angular_scale_;
    yaw += msg->angular.z * angular_scale_;

    // Convert back to quaternion
    q.setRPY(roll, pitch, yaw);
    new_pose.orientation.x = q.x();
    new_pose.orientation.y = q.y();
    new_pose.orientation.z = q.z();
    new_pose.orientation.w = q.w();

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

void PiperTeleopNode::pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  current_pose_ = *msg;
  have_pose_ = true;
}

void PiperTeleopNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

void PiperTeleopNode::set_control_mode(const std::string& mode)
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = mode;
  control_mode_pub_->publish(std::move(msg));
  control_mode_ = mode;

  RCLCPP_INFO(this->get_logger(), "Control mode set to: %s", mode.c_str());
}

void PiperTeleopNode::publish_pose_command(const geometry_msgs::msg::Pose& pose)
{
  auto msg = std::make_unique<geometry_msgs::msg::Pose>();
  *msg = pose;
  pose_cmd_pub_->publish(std::move(msg));
}

void PiperTeleopNode::publish_joint_command(const sensor_msgs::msg::JointState& joint_state)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();
  msg->name = joint_state.name;
  msg->position = joint_state.position;
  joint_cmd_pub_->publish(std::move(msg));
}

}  // namespace arm_hand_control

// Main entry point for the ROS node
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_hand_control::PiperTeleopNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
