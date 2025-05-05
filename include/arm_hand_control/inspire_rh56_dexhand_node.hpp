#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace arm_hand_control
{

struct JointInfo
{
  std::string name;
  std::string finger;
  std::string role;
  double limit_max;         // in radians
  double default_position;  // in radians
  size_t command_index;     // index in the command array
};

/**
 * Inspire RH56 Dexhand controller node
 *
 * This node serves as the hardware interface for the Inspire RH56 Dexhand.
 * It receives joint position commands and translates them into the appropriate
 * serial commands for the physical hand hardware. The node manages the serial
 * connection and handles command thresholding to reduce unnecessary updates.
 *
 * Topics:
 * - Subscriptions:
 *   - joint_states (sensor_msgs/JointState):
 *     Joint positions to be applied to the robotic hand
 *
 * Hardware Interface:
 * - Serial connection to the Inspire RH56 Dexhand
 *
 * Parameters:
 * - port (string): Serial port for the hand (e.g., "/dev/ttyUSB0")
 * - baudrate (int): Serial baudrate (default: 115200)
 * - config_file (string): Path to joint configuration file
 * - command_threshold (int): Threshold for sending new commands (to avoid sending
 *   identical commands or commands with minimal differences)
 */
class InspireRH56DexhandNode : public rclcpp::Node
{
public:
  InspireRH56DexhandNode();
  ~InspireRH56DexhandNode();

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool load_joint_config(const std::string& config_file);
  bool init_serial(const std::string& port, int baudrate);
  void close_serial();
  std::vector<int> convert_positions_to_commands(const std::map<std::string, double>& joint_positions);
  bool send_commands(const std::vector<int>& command_values);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::vector<JointInfo> joints_;
  int serial_port_ = -1;

  std::vector<int> last_command_values_;
  int command_threshold_;
};

}  // namespace arm_hand_control
