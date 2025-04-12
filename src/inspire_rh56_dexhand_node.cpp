#include "arm_hand_control/inspire_rh56_dexhand_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <algorithm>
#include <numeric>
#include <yaml-cpp/yaml.h>

namespace arm_hand_control
{

using namespace std::chrono_literals;

InspireRH56DexhandNode::InspireRH56DexhandNode() : Node("inspire_rh56_dexhand_node")
{
  // Declare parameters
  declare_parameter("config_file", "config/hand/inspire_rh56.yaml");
  declare_parameter("serial_port", "/dev/ttyUSB0");
  declare_parameter("baudrate", 115200);
  declare_parameter("command_threshold", 50);

  // Get parameters
  std::string config_file = get_parameter("config_file").as_string();
  std::string serial_port = get_parameter("serial_port").as_string();
  int baudrate = get_parameter("baudrate").as_int();
  command_threshold_ = get_parameter("command_threshold").as_int();

  // Load joint configuration
  if (!load_joint_config(config_file))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load joint configuration from %s", config_file.c_str());
    throw std::runtime_error("Failed to load joint configuration");
  }

  // Initialize serial port
  if (!init_serial(serial_port, baudrate))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port %s", serial_port.c_str());
    throw std::runtime_error("Failed to initialize serial port");
  }

  // Send TWO gesture command to initialize the hand
  unsigned char TWO[] = { 235, 144, 1, 15, 18, 206, 5, 0, 0, 0, 0, 192, 3, 192, 3, 45, 0, 0, 0, 168 };
  int bytes_written = write(serial_port_, TWO, sizeof(TWO));
  if (bytes_written < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send TWO gesture command: %s", strerror(errno));
    close_serial();
    throw std::runtime_error("Failed to send TWO gesture command");
  }

  // Define QoS profile to match the publisher - use sensor data profile as a base
  auto qos = rclcpp::SensorDataQoS();

  // Create joint state subscriber
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos, std::bind(&InspireRH56DexhandNode::joint_state_callback, this, std::placeholders::_1));

  // Initialize with -1 values
  last_command_values_.resize(joints_.size(), -1);

  RCLCPP_INFO(this->get_logger(), "InspireRH56DexhandNode initialized with %ld joints", joints_.size());
  RCLCPP_INFO(this->get_logger(), "Listening for joint_states and sending commands to %s", serial_port.c_str());
  RCLCPP_INFO(this->get_logger(), "Command threshold: %d/1000 (minimum change to send new commands)",
              command_threshold_);
}

InspireRH56DexhandNode::~InspireRH56DexhandNode()
{
  close_serial();
}

void InspireRH56DexhandNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received joint state message");

  // Check if the message contains the expected number of joints
  if (msg->name.size() != joints_.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Received joint state message with unexpected number of joints");
    return;
  }
  // Check if the message contains the expected joint names
  std::set<std::string> received_joints(msg->name.begin(), msg->name.end());
  for (const auto& joint : joints_)
  {
    if (received_joints.find(joint.name) == received_joints.end())
    {
      RCLCPP_ERROR(this->get_logger(), "Required joint '%s' not found in received joint state message",
                   joint.name.c_str());
      return;
    }
  }

  // Create map of joint positions
  std::map<std::string, double> joint_positions;
  for (size_t i = 0; i < msg->name.size(); i++)
  {
    joint_positions[msg->name[i]] = msg->position[i];
  }

  std::vector<int> command_values = convert_positions_to_commands(joint_positions);
  send_commands(command_values);
}

bool InspireRH56DexhandNode::load_joint_config(const std::string& config_file)
{
  try
  {
    YAML::Node config = YAML::LoadFile(config_file);

    if (!config["hand_config"] || !config["hand_config"]["joints"])
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid configuration file format");
      return false;
    }

    std::map<std::string, size_t> finger_command_indices = {
      { "pinky", 0 },   // little finger
      { "ring", 1 },    // ring finger
      { "middle", 2 },  // middle finger
      { "index", 3 },   // index finger
      { "thumb", 4 }    // thumb (role: pitch for bending)
    };

    YAML::Node joints = config["hand_config"]["joints"];
    for (size_t i = 0; i < joints.size(); i++)
    {
      JointInfo joint;
      joint.name = joints[i]["name"].as<std::string>();
      joint.finger = joints[i]["finger"].as<std::string>();
      joint.role = joints[i]["role"].as<std::string>();
      joint.limit_max = joints[i]["limit_max"].as<double>();
      joint.default_position = joints[i]["default_position"].as<double>();

      // Set command index based on finger and role
      if (joint.name == "thumb_proximal_yaw_joint")
      {
        // Thumb yaw (rotation) is at index 5
        joint.command_index = 5;
      }
      else if (finger_command_indices.find(joint.finger) != finger_command_indices.end())
      {
        joint.command_index = finger_command_indices[joint.finger];
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unknown finger type: %s for joint %s", joint.finger.c_str(),
                    joint.name.c_str());
        continue;
      }

      joints_.push_back(joint);
    }

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error loading configuration: %s", e.what());
    return false;
  }
}

bool InspireRH56DexhandNode::init_serial(const std::string& port, int baudrate)
{
  serial_port_ = open(port.c_str(), O_RDWR);
  if (serial_port_ < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_port_, &tty) != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
    close_serial();
    return false;
  }

  tty.c_cflag &= ~PARENB;         // Clear parity bit
  tty.c_cflag &= ~CSTOPB;         // One stop bit
  tty.c_cflag &= ~CSIZE;          // Clear size bits
  tty.c_cflag |= CS8;             // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines

  tty.c_lflag &= ~ICANON;  // Non-canonical mode
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;  // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;  // Raw output
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 1;  // Wait for up to 100ms
  tty.c_cc[VMIN] = 0;   // Return as soon as any data is received

  // Set baud rate
  speed_t baud;
  switch (baudrate)
  {
    case 19200:
      baud = B19200;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, using 115200", baudrate);
      baud = B115200;
      break;
  }

  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
    close_serial();
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Serial port %s initialized at %d baud", port.c_str(), baudrate);
  return true;
}

void InspireRH56DexhandNode::close_serial()
{
  if (serial_port_ >= 0)
  {
    close(serial_port_);
    serial_port_ = -1;
  }
}

std::vector<int>
InspireRH56DexhandNode::convert_positions_to_commands(const std::map<std::string, double>& joint_positions)
{
  std::vector<int> command_values(joints_.size(), 0);

  for (const auto& joint : joints_)
  {
    // Check if the joint is in the received message
    auto it = joint_positions.find(joint.name);
    if (it == joint_positions.end())
    {
      // Joint not in message, use no response value
      command_values[joint.command_index] = -1;
      continue;
    }

    double position = it->second;  // Position in radians
    int value = static_cast<int>((position / joint.limit_max) * 1000.0);
    value = std::max(0, std::min(value, 1000));
    value = 1000 - value;  // Invert value for the command ANGLE_SET (See 2.4.11)

    // If the change is less than the threshold and we have a previous valid command,
    // use -1 to tell the servo to keep its current position
    if (last_command_values_[joint.command_index] != -1 &&
        std::abs(value - last_command_values_[joint.command_index]) <= command_threshold_)
    {
      command_values[joint.command_index] = -1;  // No change needed
    }
    else
    {
      command_values[joint.command_index] = value;
      last_command_values_[joint.command_index] = value;
    }
  }

  return command_values;
}

bool InspireRH56DexhandNode::send_commands(const std::vector<int>& command_values)
{
  // Command packet format (Ref to the device's protocol manual):
  // [235, 144, 1, 15, 18, 206, 5, val1_lo, val1_hi, val2_lo, val2_hi, ..., checksum]
  std::vector<unsigned char> cmd = { 235, 144, 1, 15, 18, 206, 5 };

  // Add command values
  for (int val : command_values)
  {
    cmd.push_back(val & 0xff);           // low byte
    cmd.push_back((val & 0xff00) >> 8);  // high byte
  }

  // Calculate checksum (sum of bytes 2 to end-1, then & 0xff)
  unsigned char checksum = std::accumulate(cmd.begin() + 2, cmd.end(), 0) & 0xff;
  cmd.push_back(checksum);

  // Send command via serial port
  ssize_t bytes_written = write(serial_port_, cmd.data(), cmd.size());
  if (bytes_written != static_cast<ssize_t>(cmd.size()))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
    return false;
  }

  return true;
}

}  // namespace arm_hand_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_hand_control::InspireRH56DexhandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
