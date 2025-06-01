#include "arm_hand_control/hand_landmark_twist_publisher.hpp"
#include <cmath>
#include <algorithm>

namespace arm_hand_control
{

HandLandmarkTwistPublisher::HandLandmarkTwistPublisher()
  : Node("hand_landmark_twist_publisher")
  , has_reference_(false)
  , reference_index_pinky_distance_(0.0)
  , reference_thumb_index_distance_(0.0)
  , last_grasp_percentage_(-1.0)
  , continuous_detection_start_(std::chrono::steady_clock::now())
  , last_detection_time_(std::chrono::steady_clock::now())
{
  // Declare and get parameters
  declare_parameter("smoothing_factor", 0.8);
  declare_parameter("position_scale", 1.0);
  declare_parameter("orientation_scale", 1.0);
  declare_parameter("dead_zone_threshold", 0.1);
  declare_parameter("max_twist_linear", 0.5);
  declare_parameter("max_twist_angular", 1.0);
  declare_parameter("camera_width", 640.0);
  declare_parameter("camera_height", 480.0);

  smoothing_factor_ = get_parameter("smoothing_factor").as_double();
  position_scale_ = get_parameter("position_scale").as_double();
  orientation_scale_ = get_parameter("orientation_scale").as_double();
  dead_zone_threshold_ = get_parameter("dead_zone_threshold").as_double();
  max_twist_linear_ = get_parameter("max_twist_linear").as_double();
  max_twist_angular_ = get_parameter("max_twist_angular").as_double();
  camera_width_ = get_parameter("camera_width").as_double();
  camera_height_ = get_parameter("camera_height").as_double();

  // Create ROS2 components
  landmark_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "hand_landmarks", 10, std::bind(&HandLandmarkTwistPublisher::landmark_callback, this, std::placeholders::_1));

  twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("pose/cmd_vel", 10);

  gesture_client_ = rclcpp_action::create_client<ExecuteGesture>(this, "execute_gesture");

  timeout_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&HandLandmarkTwistPublisher::check_detection_timeout, this));

  RCLCPP_INFO(get_logger(), "Hand Landmark Twist Publisher initialized");
}

void HandLandmarkTwistPublisher::landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  auto current_time = std::chrono::steady_clock::now();

  if (msg->poses.size() != HAND_LANDMARK_COUNT)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Expected %d landmarks, got %zu", HAND_LANDMARK_COUNT,
                         msg->poses.size());
    if (!has_reference_)
    {
      continuous_detection_start_ = current_time;
    }
    return;
  }

  last_detection_time_ = current_time;

  // Establish reference if needed
  if (!has_reference_)
  {
    auto time_since_start = current_time - continuous_detection_start_;
    if (time_since_start > std::chrono::seconds(5))
    {
      continuous_detection_start_ = current_time;
    }

    auto detection_duration = current_time - continuous_detection_start_;
    if (detection_duration >= DETECTION_REQUIRED_DURATION)
    {
      reference_landmarks_ = msg->poses;
      reference_index_pinky_distance_ = calculate_distance(msg->poses[INDEX_MCP_IDX], msg->poses[PINKY_MCP_IDX]);
      reference_thumb_index_distance_ = calculate_distance(msg->poses[THUMB_TIP_IDX], msg->poses[INDEX_MCP_IDX]);
      reference_middle_finger_position_ = msg->poses[MIDDLE_MCP_IDX];
      has_reference_ = true;

      RCLCPP_INFO(get_logger(), "Reference established after %.1f seconds",
                  std::chrono::duration<double>(detection_duration).count());
    }
    return;
  }

  // Process grasp gesture
  process_grasp_gesture(msg->poses);

  // Calculate twist command
  geometry_msgs::msg::Twist twist_cmd;

  // Linear motion
  double z_change = calculate_z_position_change(msg->poses);
  twist_cmd.linear.z = apply_smoothing(z_change, previous_twist_.linear.z, smoothing_factor_);

  double x_change, y_change;
  calculate_xy_position_change(msg->poses, x_change, y_change);
  twist_cmd.linear.x = apply_smoothing(x_change, previous_twist_.linear.x, smoothing_factor_);
  twist_cmd.linear.y = apply_smoothing(y_change, previous_twist_.linear.y, smoothing_factor_);

  // Angular motion
  tf2::Vector3 orientation_change = calculate_orientation_change(msg->poses);
  twist_cmd.angular.x = apply_smoothing(orientation_change.x(), previous_twist_.angular.x, smoothing_factor_);
  twist_cmd.angular.y = apply_smoothing(orientation_change.y(), previous_twist_.angular.y, smoothing_factor_);
  twist_cmd.angular.z = apply_smoothing(orientation_change.z(), previous_twist_.angular.z, smoothing_factor_);

  // Store the previous twist command for smoothing
  previous_twist_ = twist_cmd;

  // Apply scaling and limits
  twist_cmd.linear.x = clamp_value(twist_cmd.linear.x * position_scale_, -max_twist_linear_, max_twist_linear_);
  twist_cmd.linear.y = clamp_value(twist_cmd.linear.y * position_scale_, -max_twist_linear_, max_twist_linear_);
  twist_cmd.linear.z = clamp_value(twist_cmd.linear.z * position_scale_, -max_twist_linear_, max_twist_linear_);
  twist_cmd.angular.x = clamp_value(twist_cmd.angular.x * orientation_scale_, -max_twist_angular_, max_twist_angular_);
  twist_cmd.angular.y = clamp_value(twist_cmd.angular.y * orientation_scale_, -max_twist_angular_, max_twist_angular_);
  twist_cmd.angular.z = clamp_value(twist_cmd.angular.z * orientation_scale_, -max_twist_angular_, max_twist_angular_);

  // Apply dead zone and publish
  twist_cmd.linear.x = apply_dead_zone(twist_cmd.linear.x, dead_zone_threshold_);
  twist_cmd.linear.y = apply_dead_zone(twist_cmd.linear.y, dead_zone_threshold_);
  twist_cmd.linear.z = apply_dead_zone(twist_cmd.linear.z, dead_zone_threshold_);
  twist_cmd.angular.x = apply_dead_zone(twist_cmd.angular.x, dead_zone_threshold_);
  twist_cmd.angular.y = apply_dead_zone(twist_cmd.angular.y, dead_zone_threshold_);
  twist_cmd.angular.z = apply_dead_zone(twist_cmd.angular.z, dead_zone_threshold_);

  twist_publisher_->publish(twist_cmd);
}

double HandLandmarkTwistPublisher::calculate_z_position_change(const std::vector<geometry_msgs::msg::Pose>& landmarks)
{
  double current_distance = calculate_distance(landmarks[INDEX_MCP_IDX], landmarks[PINKY_MCP_IDX]);
  double distance_change = reference_index_pinky_distance_ - current_distance;

  return (reference_index_pinky_distance_ > 0.0) ? distance_change / reference_index_pinky_distance_ : 0.0;
}

void HandLandmarkTwistPublisher::calculate_xy_position_change(const std::vector<geometry_msgs::msg::Pose>& landmarks,
                                                              double& x_change, double& y_change)
{
  x_change = landmarks[MIDDLE_MCP_IDX].position.x - reference_middle_finger_position_.position.x;
  y_change = landmarks[MIDDLE_MCP_IDX].position.y - reference_middle_finger_position_.position.y;

  if (camera_width_ > 0.0 && camera_height_ > 0.0)
  {
    x_change /= camera_width_;
    y_change /= camera_height_;
  }

  x_change = -x_change;  // Invert X for camera convention
  y_change = -y_change;  // Invert Y for camera convention
}

tf2::Vector3
HandLandmarkTwistPublisher::calculate_orientation_change(const std::vector<geometry_msgs::msg::Pose>& landmarks)
{
  // We need better way to calculate orientation change
  // Uncomment the following line to disable orientation change calculation
  return tf2::Vector3(0.0, 0.0, 0.0);

  // Get current and reference triangle points
  auto get_vector = [](const geometry_msgs::msg::Pose& p) { return tf2::Vector3(p.position.x, p.position.y, 0.0); };

  tf2::Vector3 wrist = get_vector(landmarks[WRIST_IDX]);
  tf2::Vector3 index_mcp = get_vector(landmarks[INDEX_MCP_IDX]);
  tf2::Vector3 pinky_mcp = get_vector(landmarks[PINKY_MCP_IDX]);

  tf2::Vector3 ref_wrist = get_vector(reference_landmarks_[WRIST_IDX]);
  tf2::Vector3 ref_index_mcp = get_vector(reference_landmarks_[INDEX_MCP_IDX]);
  tf2::Vector3 ref_pinky_mcp = get_vector(reference_landmarks_[PINKY_MCP_IDX]);

  // YAW: Triangle orientation change
  tf2::Vector3 current_index_pinky = pinky_mcp - index_mcp;
  tf2::Vector3 ref_index_pinky = ref_pinky_mcp - ref_index_mcp;

  double current_angle = std::atan2(current_index_pinky.y(), current_index_pinky.x());
  double ref_angle = std::atan2(ref_index_pinky.y(), ref_index_pinky.x());
  double yaw = current_angle - ref_angle;

  while (yaw > M_PI)
    yaw -= 2.0 * M_PI;
  while (yaw < -M_PI)
    yaw += 2.0 * M_PI;

  // PITCH: Triangle area change
  tf2::Vector3 current_v1 = index_mcp - wrist;
  tf2::Vector3 current_v2 = pinky_mcp - wrist;
  tf2::Vector3 ref_v1 = ref_index_mcp - ref_wrist;
  tf2::Vector3 ref_v2 = ref_pinky_mcp - ref_wrist;

  double current_area = std::abs(current_v1.x() * current_v2.y() - current_v1.y() * current_v2.x()) * 0.5;
  double ref_area = std::abs(ref_v1.x() * ref_v2.y() - ref_v1.y() * ref_v2.x()) * 0.5;

  double area_ratio = (ref_area > 1e-6) ? (current_area / ref_area) : 1.0;
  double pitch = std::asin(std::clamp(1.0 - area_ratio, -1.0, 1.0));

  // ROLL: Triangle aspect ratio change
  auto normalize_safe = [](tf2::Vector3 v) { return (v.length() > 1e-6) ? v.normalized() : tf2::Vector3(1, 0, 0); };

  tf2::Vector3 current_normalized = normalize_safe(current_index_pinky);
  tf2::Vector3 ref_normalized = normalize_safe(ref_index_pinky);

  tf2::Vector3 wrist_to_index = index_mcp - wrist;
  tf2::Vector3 ref_wrist_to_index = ref_index_mcp - ref_wrist;

  double current_height =
      std::abs(wrist_to_index.x() * current_normalized.y() - wrist_to_index.y() * current_normalized.x());
  double ref_height =
      std::abs(ref_wrist_to_index.x() * ref_normalized.y() - ref_wrist_to_index.y() * ref_normalized.x());

  double current_base = current_index_pinky.length();
  double ref_base = ref_index_pinky.length();

  double current_aspect = (current_base > 1e-6) ? (current_height / current_base) : 0.0;
  double ref_aspect = (ref_base > 1e-6) ? (ref_height / ref_base) : 0.0;

  double roll = (current_aspect - ref_aspect) * 2.0;

  return tf2::Vector3(roll, pitch, yaw);
}

double HandLandmarkTwistPublisher::calculate_distance(const geometry_msgs::msg::Pose& p1,
                                                      const geometry_msgs::msg::Pose& p2)
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  double dz = p1.position.z - p2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double HandLandmarkTwistPublisher::apply_smoothing(double current, double previous, double factor)
{
  return factor * previous + (1.0 - factor) * current;
}

double HandLandmarkTwistPublisher::apply_dead_zone(double value, double threshold)
{
  return (std::abs(value) < threshold) ? 0.0 : value;
}

double HandLandmarkTwistPublisher::clamp_value(double value, double min_val, double max_val)
{
  return std::clamp(value, min_val, max_val);
}

void HandLandmarkTwistPublisher::check_detection_timeout()
{
  if (!has_reference_)
    return;

  auto current_time = std::chrono::steady_clock::now();
  auto time_since_last_detection = current_time - last_detection_time_;

  if (time_since_last_detection >= DETECTION_TIMEOUT_DURATION)
  {
    has_reference_ = false;
    twist_publisher_->publish(geometry_msgs::msg::Twist{});

    RCLCPP_WARN(get_logger(), "Reference invalidated after %.1f seconds without detection",
                std::chrono::duration<double>(time_since_last_detection).count());
  }
}

double
HandLandmarkTwistPublisher::calculate_thumb_index_distance(const std::vector<geometry_msgs::msg::Pose>& landmarks)
{
  return calculate_distance(landmarks[THUMB_TIP_IDX], landmarks[INDEX_MCP_IDX]);
}

void HandLandmarkTwistPublisher::process_grasp_gesture(const std::vector<geometry_msgs::msg::Pose>& landmarks)
{
  if (!gesture_client_->wait_for_action_server(std::chrono::milliseconds(10)))
  {
    return;  // Action server not available, skip this time
  }

  double current_thumb_index_distance = calculate_thumb_index_distance(landmarks);

  // Calculate percentage based on distance change from reference
  // When thumb and index are closer together, percentage should be higher (more closed grasp)
  double distance_ratio =
      (reference_thumb_index_distance_ > 0.0) ? (current_thumb_index_distance / reference_thumb_index_distance_) : 1.0;

  // Invert the ratio so closer fingers = higher percentage
  // Clamp between 0.0 and 1.0 (not 0-100)
  double grasp_percentage = std::clamp(1.0 - distance_ratio, 0.0, 1.0);

  // Only send goal if percentage changed significantly (avoid spam)
  // Use 0.05 (5%) threshold for 0.0-1.0 range
  if (std::abs(grasp_percentage - last_grasp_percentage_) > 0.05)
  {
    send_grasp_goal(static_cast<float>(grasp_percentage));
    last_grasp_percentage_ = grasp_percentage;
  }
}

void HandLandmarkTwistPublisher::send_grasp_goal(float percentage)
{
  auto goal_msg = ExecuteGesture::Goal();
  goal_msg.gesture_name = percentage < 0.1 ? "open_hand" : "three_finger_grasp";
  goal_msg.duration = 0.1f;
  goal_msg.percentage = percentage;

  auto send_goal_options = rclcpp_action::Client<ExecuteGesture>::SendGoalOptions();

  // Simple result callback (no feedback needed for quick updates)
  send_goal_options.result_callback = [this](const GoalHandleExecuteGesture::WrappedResult& result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_DEBUG(get_logger(), "Grasp goal failed");
    }
  };

  gesture_client_->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_DEBUG(get_logger(), "Sent grasp goal: %.3f", percentage);
}

}  // namespace arm_hand_control

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandLandmarkTwistPublisher>());
  rclcpp::shutdown();
  return 0;
}