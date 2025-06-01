#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <chrono>

#include "arm_hand_control/action/execute_gesture.hpp"

namespace arm_hand_control
{

/**
 * HandLandmarkTwistPublisher converts hand landmarks to twist commands for arm control
 *
 * The node processes MediaPipe hand landmarks and generates twist commands
 * for robotic arm end effector control based on hand gestures and movements.
 *
 * Subscribed topics:
 *   - hand_landmarks (geometry_msgs/PoseArray): 2D positions of hand landmarks
 *     (21 points matching MediaPipe model)
 *
 * Published topics:
 *   - pose/cmd_vel (geometry_msgs/Twist): Twist commands for robotic arm end effector
 *
 * Parameters:
 *   - smoothing_factor (double): EMA smoothing factor for twist commands (0.0-1.0)
 *   - position_scale (double): Scaling factor for position commands
 *   - orientation_scale (double): Scaling factor for orientation commands
 *   - dead_zone_threshold (double): Minimum change threshold to filter noise
 *   - max_twist_linear (double): Maximum linear velocity limit
 *   - max_twist_angular (double): Maximum angular velocity limit
 *   - camera_width (double): Camera width for coordinate normalization
 *   - camera_height (double): Camera height for coordinate normalization
 */
class HandLandmarkTwistPublisher : public rclcpp::Node
{
public:
  using ExecuteGesture = arm_hand_control::action::ExecuteGesture;
  using GoalHandleExecuteGesture = rclcpp_action::ClientGoalHandle<ExecuteGesture>;

  HandLandmarkTwistPublisher();
  ~HandLandmarkTwistPublisher() = default;

private:
  void landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void check_detection_timeout();

  // Twist calculation methods
  double calculate_z_position_change(const std::vector<geometry_msgs::msg::Pose>& landmarks);
  void calculate_xy_position_change(const std::vector<geometry_msgs::msg::Pose>& landmarks, double& x_change,
                                    double& y_change);
  tf2::Vector3 calculate_orientation_change(const std::vector<geometry_msgs::msg::Pose>& landmarks);

  // Gesture control methods
  void process_grasp_gesture(const std::vector<geometry_msgs::msg::Pose>& landmarks);
  double calculate_thumb_index_distance(const std::vector<geometry_msgs::msg::Pose>& landmarks);
  void send_grasp_goal(float percentage);

  // Utility methods
  static double calculate_distance(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2);
  static double apply_smoothing(double current, double previous, double factor);
  static double apply_dead_zone(double value, double threshold);
  static double clamp_value(double value, double min_val, double max_val);

  // ROS2 components
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp_action::Client<ExecuteGesture>::SharedPtr gesture_client_;

  // Parameters
  double smoothing_factor_, position_scale_, orientation_scale_;
  double dead_zone_threshold_, max_twist_linear_, max_twist_angular_;
  double camera_width_, camera_height_;

  // State tracking
  bool has_reference_;
  std::vector<geometry_msgs::msg::Pose> reference_landmarks_;
  geometry_msgs::msg::Twist previous_twist_;
  double reference_index_pinky_distance_;
  geometry_msgs::msg::Pose reference_middle_finger_position_;

  // Grasp gesture state
  double reference_thumb_index_distance_;
  double last_grasp_percentage_;

  // Timing
  std::chrono::steady_clock::time_point continuous_detection_start_;
  std::chrono::steady_clock::time_point last_detection_time_;
  static constexpr auto DETECTION_REQUIRED_DURATION = std::chrono::seconds(2);
  static constexpr auto DETECTION_TIMEOUT_DURATION = std::chrono::seconds(1);

  // MediaPipe hand landmark indices
  static constexpr int HAND_LANDMARK_COUNT = 21;
  static constexpr int WRIST_IDX = 0;
  static constexpr int THUMB_TIP_IDX = 4;
  static constexpr int INDEX_MCP_IDX = 5;
  static constexpr int MIDDLE_MCP_IDX = 9;
  static constexpr int PINKY_MCP_IDX = 17;
};

}  // namespace arm_hand_control
