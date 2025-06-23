// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************
#include "arm_hand_control/hand_landmark_pose_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>

namespace arm_hand_control
{

HandLandmarkPosePublisher::HandLandmarkPosePublisher()
: Node("hand_landmark_pose_publisher"),
  has_reference_(false),
  reference_z_distance_(0.0),
  last_grasp_percentage_(-1.0),
  last_detection_time_(std::chrono::steady_clock::now()),
  continuous_detection_start_(std::chrono::steady_clock::now())
{
  // Declare and get general parameters
  declare_parameter("smoothing_factor", 0.8);
  declare_parameter("position_scale", 1.0);
  declare_parameter("dead_zone_threshold", 0.01);
  declare_parameter("camera_width", 640.0);
  declare_parameter("camera_height", 480.0);
  declare_parameter("target_frame", "base_link");

  smoothing_factor_ = get_parameter("smoothing_factor").as_double();
  position_scale_ = get_parameter("position_scale").as_double();
  dead_zone_threshold_ = get_parameter("dead_zone_threshold").as_double();
  camera_width_ = get_parameter("camera_width").as_double();
  camera_height_ = get_parameter("camera_height").as_double();
  target_frame_ = get_parameter("target_frame").as_string();

  // Declare and get pose mapping parameters
  declare_parameter("initial_pose_x", 0.3);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.2);
  declare_parameter("initial_pose_roll", 0.0);
  declare_parameter("initial_pose_pitch", 0.0);
  declare_parameter("initial_pose_yaw", 0.0);
  declare_parameter("max_pose_x", 0.5);
  declare_parameter("max_pose_y", 0.3);
  declare_parameter("max_pose_z", 0.4);
  declare_parameter("min_pose_x", 0.1);
  declare_parameter("min_pose_y", -0.3);
  declare_parameter("min_pose_z", 0.05);

  initial_pose_x_ = get_parameter("initial_pose_x").as_double();
  initial_pose_y_ = get_parameter("initial_pose_y").as_double();
  initial_pose_z_ = get_parameter("initial_pose_z").as_double();
  initial_pose_roll_ = get_parameter("initial_pose_roll").as_double();
  initial_pose_pitch_ = get_parameter("initial_pose_pitch").as_double();
  initial_pose_yaw_ = get_parameter("initial_pose_yaw").as_double();
  max_pose_x_ = get_parameter("max_pose_x").as_double();
  max_pose_y_ = get_parameter("max_pose_y").as_double();
  max_pose_z_ = get_parameter("max_pose_z").as_double();
  min_pose_x_ = get_parameter("min_pose_x").as_double();
  min_pose_y_ = get_parameter("min_pose_y").as_double();
  min_pose_z_ = get_parameter("min_pose_z").as_double();

  // Declare and get gripper control parameters
  declare_parameter("max_gripper_position", 0.06);  // in meters
  declare_parameter("min_gripper_position", 0.0);
  declare_parameter("max_gripper_effort", 1.0);  // in newtons
  declare_parameter("gripper_command_threshold", 0.05);

  max_gripper_position_ = get_parameter("max_gripper_position").as_double();
  min_gripper_position_ = get_parameter("min_gripper_position").as_double();
  max_gripper_effort_ = get_parameter("max_gripper_effort").as_double();
  gripper_command_threshold_ = get_parameter("gripper_command_threshold").as_double();

  // Create ROS2 components
  landmark_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "hand_landmarks", 10,
    std::bind(&HandLandmarkPosePublisher::landmark_callback, this, std::placeholders::_1));

  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  gripper_publisher_ = create_publisher<control_msgs::msg::GripperCommand>("gripper_command", 10);

  gesture_client_ = rclcpp_action::create_client<ExecuteGesture>(this, "execute_gesture");

  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&HandLandmarkPosePublisher::check_detection_timeout, this));

  // Initialize previous pose to initial values
  previous_pose_.header.frame_id = target_frame_;
  previous_pose_.pose.position.x = initial_pose_x_;
  previous_pose_.pose.position.y = initial_pose_y_;
  previous_pose_.pose.position.z = initial_pose_z_;

  // Set initial orientation from roll, pitch, yaw parameters
  tf2::Quaternion initial_quat;
  initial_quat.setRPY(initial_pose_roll_, initial_pose_pitch_, initial_pose_yaw_);
  previous_pose_.pose.orientation.x = initial_quat.x();
  previous_pose_.pose.orientation.y = initial_quat.y();
  previous_pose_.pose.orientation.z = initial_quat.z();
  previous_pose_.pose.orientation.w = initial_quat.w();

  RCLCPP_INFO(get_logger(), "Hand Landmark Pose Publisher initialized");
  RCLCPP_INFO(get_logger(), "Target frame: %s", target_frame_.c_str());
  RCLCPP_INFO(
    get_logger(), "Initial pose: [%.3f, %.3f, %.3f]", initial_pose_x_, initial_pose_y_,
    initial_pose_z_);
  RCLCPP_INFO(
    get_logger(), "Initial orientation (RPY): [%.3f, %.3f, %.3f]", initial_pose_roll_,
    initial_pose_pitch_, initial_pose_yaw_);
  RCLCPP_INFO(get_logger(), "Pose range X: [%.3f, %.3f]", min_pose_x_, max_pose_x_);
  RCLCPP_INFO(get_logger(), "Pose range Y: [%.3f, %.3f]", min_pose_y_, max_pose_y_);
  RCLCPP_INFO(get_logger(), "Pose range Z: [%.3f, %.3f]", min_pose_z_, max_pose_z_);
  RCLCPP_INFO(
    get_logger(), "Using fixed MediaPipe hand landmark indices for position mapping only");
  RCLCPP_INFO(get_logger(), "Gripper control parameters:");
  RCLCPP_INFO(
    get_logger(), "Position range: [%.3f, %.3f] m", min_gripper_position_, max_gripper_position_);
  RCLCPP_INFO(get_logger(), "Max effort: %.1f N", max_gripper_effort_);
  RCLCPP_INFO(get_logger(), "Command threshold: %.3f", gripper_command_threshold_);
}

void HandLandmarkPosePublisher::landmark_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  auto current_time = std::chrono::steady_clock::now();

  if (msg->poses.size() != HAND_LANDMARK_COUNT) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Expected %d landmarks, got %zu", HAND_LANDMARK_COUNT,
      msg->poses.size());
    if (!has_reference_) {
      continuous_detection_start_ = current_time;
    }
    return;
  }

  last_detection_time_ = current_time;

  // Establish reference if needed
  if (!has_reference_) {
    auto time_since_start = current_time - continuous_detection_start_;
    if (time_since_start > std::chrono::seconds(5)) {
      continuous_detection_start_ = current_time;
    }

    auto detection_duration = current_time - continuous_detection_start_;
    if (detection_duration >= DETECTION_REQUIRED_DURATION) {
      reference_landmarks_ = msg->poses;
      reference_x_landmark_ = msg->poses[MIDDLE_MCP_IDX];
      reference_y_landmark_ = msg->poses[MIDDLE_MCP_IDX];
      reference_z_distance_ =
        calculate_distance(msg->poses[INDEX_MCP_IDX], msg->poses[PINKY_MCP_IDX]);
      has_reference_ = true;

      RCLCPP_INFO(
        get_logger(), "Reference established after %.1f seconds",
        std::chrono::duration<double>(detection_duration).count());
    }
    return;
  }

  // Process grasp gesture
  process_grasp_gesture(msg->poses);

  // Calculate target pose
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.stamp = this->now();
  target_pose.header.frame_id = target_frame_;

  // Calculate position changes with dynamic range estimation
  double current_palm_size_pixels =
    calculate_distance(msg->poses[INDEX_MCP_IDX], msg->poses[PINKY_MCP_IDX]);

  double x_change = calculate_x_position_change(msg->poses, current_palm_size_pixels);
  double y_change = calculate_y_position_change(msg->poses, current_palm_size_pixels);
  double z_change = calculate_z_position_change(msg->poses);

  // Apply dead zone
  x_change = apply_dead_zone(x_change, dead_zone_threshold_);
  y_change = apply_dead_zone(y_change, dead_zone_threshold_);
  z_change = apply_dead_zone(z_change, dead_zone_threshold_);

  // Map to target pose ranges
  target_pose.pose.position.x =
    map_to_range(x_change * position_scale_, min_pose_x_, max_pose_x_, initial_pose_x_);
  target_pose.pose.position.y =
    map_to_range(y_change * position_scale_, min_pose_y_, max_pose_y_, initial_pose_y_);
  target_pose.pose.position.z =
    map_to_range(z_change * position_scale_, min_pose_z_, max_pose_z_, initial_pose_z_);

  // Set initial orientation (use configured RPY values)
  tf2::Quaternion quat;
  quat.setRPY(initial_pose_roll_, initial_pose_pitch_, initial_pose_yaw_);
  target_pose.pose.orientation.x = quat.x();
  target_pose.pose.orientation.y = quat.y();
  target_pose.pose.orientation.z = quat.z();
  target_pose.pose.orientation.w = quat.w();

  // Apply final smoothing to the complete pose
  target_pose.pose.position.x =
    apply_smoothing(target_pose.pose.position.x, previous_pose_.pose.position.x, smoothing_factor_);
  target_pose.pose.position.y =
    apply_smoothing(target_pose.pose.position.y, previous_pose_.pose.position.y, smoothing_factor_);
  target_pose.pose.position.z =
    apply_smoothing(target_pose.pose.position.z, previous_pose_.pose.position.z, smoothing_factor_);

  // Store the previous pose for smoothing
  previous_pose_ = target_pose;

  // Publish the target pose
  pose_publisher_->publish(target_pose);
}

double HandLandmarkPosePublisher::calculate_x_position_change(
  const std::vector<geometry_msgs::msg::Pose> & landmarks, double current_palm_size_pixels)
{
  double x_change_pixels = landmarks[MIDDLE_MCP_IDX].position.x - reference_x_landmark_.position.x;

  // Estimate available movement range based on current palm size and camera width
  double estimated_hand_width = current_palm_size_pixels * 3;  // Rough hand width estimation
  double available_x_range =
    (camera_width_ - estimated_hand_width) * 0.75;  // 75% of camera width for movement

  // Normalize movement as percentage of available range
  double x_change_normalized =
    (available_x_range > 0.0) ? (x_change_pixels / available_x_range) : 0.0;

  // Clamp to reasonable movement range
  x_change_normalized = std::clamp(x_change_normalized, -1.0, 1.0);

  return -x_change_normalized;  // Invert X for camera convention
}

double HandLandmarkPosePublisher::calculate_y_position_change(
  const std::vector<geometry_msgs::msg::Pose> & landmarks, double current_palm_size_pixels)
{
  double y_change_pixels = landmarks[MIDDLE_MCP_IDX].position.y - reference_y_landmark_.position.y;

  // Estimate available movement range based on current palm size and camera height
  double estimated_hand_height = current_palm_size_pixels * 3;  // Rough hand height estimation
  double available_y_range =
    (camera_height_ - estimated_hand_height) * 0.75;  // 75% of camera height for movement

  // Normalize movement as percentage of available range
  double y_change_normalized =
    (available_y_range > 0.0) ? (y_change_pixels / available_y_range) : 0.0;

  // Clamp to reasonable movement range
  y_change_normalized = std::clamp(y_change_normalized, -1.0, 1.0);

  return -y_change_normalized;  // Invert Y for camera convention
}

double HandLandmarkPosePublisher::calculate_z_position_change(
  const std::vector<geometry_msgs::msg::Pose> & landmarks)
{
  double current_distance = calculate_distance(landmarks[INDEX_MCP_IDX], landmarks[PINKY_MCP_IDX]);
  double distance_change = reference_z_distance_ - current_distance;

  return (reference_z_distance_ > 0.0) ? distance_change / reference_z_distance_ : 0.0;
}

double HandLandmarkPosePublisher::calculate_distance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  double dz = p1.position.z - p2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double HandLandmarkPosePublisher::apply_smoothing(double current, double previous, double factor)
{
  return factor * previous + (1.0 - factor) * current;
}

double HandLandmarkPosePublisher::apply_dead_zone(double value, double threshold)
{
  return (std::abs(value) < threshold) ? 0.0 : value;
}

double HandLandmarkPosePublisher::clamp_value(double value, double min_val, double max_val)
{
  return std::clamp(value, min_val, max_val);
}

double HandLandmarkPosePublisher::map_to_range(
  double normalized_value, double min_range, double max_range, double initial_value)
{
  // Map normalized value (-1 to 1) to the specified range around initial value
  double range_size = max_range - min_range;
  double mapped_value = initial_value + normalized_value * range_size;
  return clamp_value(mapped_value, min_range, max_range);
}

void HandLandmarkPosePublisher::check_detection_timeout()
{
  if (!has_reference_) return;

  auto current_time = std::chrono::steady_clock::now();
  auto time_since_last_detection = current_time - last_detection_time_;

  if (time_since_last_detection >= DETECTION_TIMEOUT_DURATION) {
    has_reference_ = false;

    // Reset to initial pose when detection times out
    geometry_msgs::msg::PoseStamped reset_pose;
    reset_pose.header.stamp = this->now();
    reset_pose.header.frame_id = target_frame_;
    reset_pose.pose.position.x = initial_pose_x_;
    reset_pose.pose.position.y = initial_pose_y_;
    reset_pose.pose.position.z = initial_pose_z_;

    // Set initial orientation
    tf2::Quaternion reset_quat;
    reset_quat.setRPY(initial_pose_roll_, initial_pose_pitch_, initial_pose_yaw_);
    reset_pose.pose.orientation.x = reset_quat.x();
    reset_pose.pose.orientation.y = reset_quat.y();
    reset_pose.pose.orientation.z = reset_quat.z();
    reset_pose.pose.orientation.w = reset_quat.w();

    pose_publisher_->publish(reset_pose);

    RCLCPP_WARN(
      get_logger(),
      "Reference invalidated after %.1f seconds without detection, reset to initial pose",
      std::chrono::duration<double>(time_since_last_detection).count());
  }
}

double HandLandmarkPosePublisher::calculate_thumb_index_distance(
  const std::vector<geometry_msgs::msg::Pose> & landmarks)
{
  return calculate_distance(landmarks[THUMB_TIP_IDX], landmarks[INDEX_MCP_IDX]);
}

void HandLandmarkPosePublisher::process_grasp_gesture(
  const std::vector<geometry_msgs::msg::Pose> & landmarks)
{
  double current_thumb_index_distance = calculate_thumb_index_distance(landmarks);
  double current_palm_size = calculate_distance(landmarks[INDEX_MCP_IDX], landmarks[PINKY_MCP_IDX]);

  // Calculate percentage based on thumb-index distance relative to current palm size
  // When thumb and index are closer together relative to palm size, percentage should be higher (more closed grasp)
  double distance_ratio =
    (current_palm_size > 0.0) ? (current_thumb_index_distance / current_palm_size) : 1.0;

  // Map the ratio to grasp percentage
  // Typical open hand: thumb-index distance ~= palm size (ratio ~1.0)
  // Typical closed grasp: thumb-index distance ~= 0.3 * palm size (ratio ~0.3)
  // Map ratio 1.0->0.0 (open) and 0.3->1.0 (closed)
  double grasp_percentage = std::clamp((1.0 - distance_ratio) / 0.7, 0.0, 1.0);

  // Only send commands if percentage changed significantly (avoid spam)
  if (std::abs(grasp_percentage - last_grasp_percentage_) > gripper_command_threshold_) {
    // Send gripper command
    send_gripper_command(grasp_percentage);

    // Send gesture action (for hand control)
    if (gesture_client_->wait_for_action_server(std::chrono::milliseconds(10))) {
      send_grasp_goal(static_cast<float>(grasp_percentage));
    }

    last_grasp_percentage_ = grasp_percentage;
  }
}

void HandLandmarkPosePublisher::send_gripper_command(double grasp_percentage)
{
  auto gripper_msg = control_msgs::msg::GripperCommand();

  // Map grasp percentage to gripper position
  // grasp_percentage 0.0 = fully open (max position)
  // grasp_percentage 1.0 = fully closed (min position)
  gripper_msg.position =
    max_gripper_position_ - (grasp_percentage * (max_gripper_position_ - min_gripper_position_));

  // Set effort based on how closed the gripper should be
  // More closed = more effort needed
  gripper_msg.max_effort = max_gripper_effort_ * (0.3 + 0.7 * grasp_percentage);

  gripper_publisher_->publish(gripper_msg);

  RCLCPP_DEBUG(
    get_logger(), "Published gripper command: position=%.3f, effort=%.1f", gripper_msg.position,
    gripper_msg.max_effort);
}

void HandLandmarkPosePublisher::send_grasp_goal(float percentage)
{
  auto goal_msg = ExecuteGesture::Goal();
  goal_msg.gesture_name = percentage < 0.2 ? "open_hand" : "three_finger_grasp";
  goal_msg.duration = 0.1f;
  goal_msg.percentage =
    percentage > 0.55 ? 0.55f : percentage;  // 0.55 is the max for three_finger_grasp

  auto send_goal_options = rclcpp_action::Client<ExecuteGesture>::SendGoalOptions();

  // Simple result callback (no feedback needed for quick updates)
  send_goal_options.result_callback =
    [this](const GoalHandleExecuteGesture::WrappedResult & result) {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_DEBUG(get_logger(), "Grasp goal failed");
      }
    };

  gesture_client_->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_DEBUG(get_logger(), "Sent grasp goal: %.3f", percentage);
}

}  // namespace arm_hand_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandLandmarkPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
