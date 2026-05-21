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
#include "arm_hand_control/hand_landmark_gripper_retargeter_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

namespace arm_hand_control
{

HandLandmarkGripperRetargeter::HandLandmarkGripperRetargeter()
: Node("hand_landmark_gripper_retargeter"),
  has_prev_gripper_width_(false),
  prev_gripper_width_(0.0),
  observed_ratio_min_(std::numeric_limits<double>::infinity()),
  observed_ratio_max_(-std::numeric_limits<double>::infinity())
{
  // Declare parameters
  this->declare_parameter("gripper_max_width", 0.06);
  this->declare_parameter("max_width_topic", "gripper_max_width");
  this->declare_parameter("pinch_open_ratio", 1.4);
  this->declare_parameter("pinch_close_ratio", 0.15);
  this->declare_parameter("gripper_smooth_factor", 0.7);

  // Get parameters
  gripper_max_width_ = this->get_parameter("gripper_max_width").as_double();
  max_width_topic_ = this->get_parameter("max_width_topic").as_string();
  pinch_open_ratio_ = this->get_parameter("pinch_open_ratio").as_double();
  pinch_close_ratio_ = this->get_parameter("pinch_close_ratio").as_double();
  gripper_smooth_factor_ = this->get_parameter("gripper_smooth_factor").as_double();

  // Validate
  if (gripper_max_width_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "gripper_max_width (%.4f) must be positive; falling back to 0.06 m",
      gripper_max_width_);
    gripper_max_width_ = 0.06;
  }
  if (pinch_open_ratio_ <= pinch_close_ratio_) {
    RCLCPP_WARN(
      this->get_logger(),
      "pinch_open_ratio (%.3f) must be > pinch_close_ratio (%.3f); using defaults 1.4/0.15",
      pinch_open_ratio_, pinch_close_ratio_);
    pinch_open_ratio_ = 1.4;
    pinch_close_ratio_ = 0.15;
  }

  auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
  landmark_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "hand_landmarks", qos,
    std::bind(&HandLandmarkGripperRetargeter::landmark_callback, this, std::placeholders::_1));

  auto max_width_qos = rclcpp::QoS(1).transient_local().reliable();
  max_width_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    max_width_topic_, max_width_qos,
    std::bind(&HandLandmarkGripperRetargeter::max_width_callback, this, std::placeholders::_1));

  // Output topic name is fixed; remap at launch time to wire it onto the
  // consumer's expected topic (e.g. hand_gripper_action_adapter's
  // "hand_gripper_command" or "gripper_command").
  gripper_command_publisher_ =
    this->create_publisher<control_msgs::msg::GripperCommand>("gripper_command", 10);

  RCLCPP_INFO(this->get_logger(), "Hand landmark gripper retargeter started");
  RCLCPP_INFO(
    this->get_logger(), "Publishing GripperCommand on '%s' (max width: %.4f m)",
    gripper_command_publisher_->get_topic_name(), gripper_max_width_);
  RCLCPP_INFO(
    this->get_logger(), "Listening for runtime gripper max width on '%s'",
    max_width_subscriber_->get_topic_name());
  RCLCPP_INFO(
    this->get_logger(), "Pinch ratio mapping: close=%.3f -> 0.0 m, open=%.3f -> %.4f m",
    pinch_close_ratio_, pinch_open_ratio_, gripper_max_width_);
}

HandLandmarkGripperRetargeter::~HandLandmarkGripperRetargeter()
{
  RCLCPP_INFO(this->get_logger(), "Hand landmark gripper retargeter shutting down");
  landmark_subscriber_.reset();
  max_width_subscriber_.reset();
  gripper_command_publisher_.reset();
}

void HandLandmarkGripperRetargeter::landmark_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.size() < static_cast<size_t>(HAND_LANDMARK_COUNT)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Incomplete hand landmarks received: %zu (expected %d)", msg->poses.size(),
      HAND_LANDMARK_COUNT);
    return;
  }

  const auto & landmarks = msg->poses;

  auto dist3d = [](const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) {
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    const double dz = a.position.z - b.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };

  // Palm length (wrist -> middle MCP) is used as a scale-invariant reference so
  // the mapping is robust to camera distance and hand size.
  const double palm_ref = dist3d(landmarks[WRIST_IDX], landmarks[MIDDLE_MCP_IDX]);
  if (palm_ref < 1e-4) {
    return;  // Degenerate landmarks; skip this frame.
  }

  // Pinch is the average distance from the thumb tip to the index and middle
  // fingertips. This couples all three fingers and behaves well for both pinch
  // and tripod grasps.
  const double thumb_index = dist3d(landmarks[THUMB_TIP_IDX], landmarks[INDEX_TIP_IDX]);
  const double thumb_middle = dist3d(landmarks[THUMB_TIP_IDX], landmarks[MIDDLE_TIP_IDX]);
  const double pinch = 0.5 * (thumb_index + thumb_middle);
  const double pinch_ratio = pinch / palm_ref;

  // Linearly retarget the normalized pinch ratio onto [kMinGripperWidth,
  // gripper_max_width_]. The minimum is intentionally non-zero so the
  // downstream hand never closes fully (which can stall the fingers against
  // each other on the dexhand).
  constexpr double kMinGripperWidth = 0.001;
  const double span = pinch_open_ratio_ - pinch_close_ratio_;
  double normalized = (pinch_ratio - pinch_close_ratio_) / span;
  normalized = std::clamp(normalized, 0.0, 1.0);

  double current_max_width = 0.06;
  double gripper_width = kMinGripperWidth;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    current_max_width = gripper_max_width_;

    gripper_width = kMinGripperWidth + normalized * (current_max_width - kMinGripperWidth);

    // EMA smoothing for stability.
    if (has_prev_gripper_width_) {
      const double s = std::clamp(gripper_smooth_factor_, 0.0, 0.99);
      gripper_width = s * prev_gripper_width_ + (1.0 - s) * gripper_width;
    }
    gripper_width = std::clamp(gripper_width, kMinGripperWidth, current_max_width);
    prev_gripper_width_ = gripper_width;
    has_prev_gripper_width_ = true;
  }

  control_msgs::msg::GripperCommand cmd;
  cmd.position = gripper_width;
  cmd.max_effort = 0.0;  // Effort is intentionally not retargeted.
  gripper_command_publisher_->publish(cmd);

  // Track the observed range of the pinch ratio so users can read off suitable
  // values for pinch_close_ratio / pinch_open_ratio without external tooling.
  // See the class docstring for the calibration procedure.
  observed_ratio_min_ = std::min(observed_ratio_min_, pinch_ratio);
  observed_ratio_max_ = std::max(observed_ratio_max_, pinch_ratio);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "pinch ratio: now=%.3f  observed=[%.3f, %.3f]  width=%.4f m", pinch_ratio, observed_ratio_min_,
    observed_ratio_max_, gripper_width);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Gripper: pinch=%.4f, palm=%.4f, ratio=%.3f -> width=%.4f m", pinch, palm_ref, pinch_ratio,
    gripper_width);
}

void HandLandmarkGripperRetargeter::max_width_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!std::isfinite(msg->data) || msg->data <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "Ignoring invalid runtime gripper max width: %.4f m", msg->data);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    if (std::abs(gripper_max_width_ - msg->data) < 1e-9) {
      return;
    }
    gripper_max_width_ = msg->data;
    if (has_prev_gripper_width_) {
      prev_gripper_width_ = std::clamp(prev_gripper_width_, 0.001, gripper_max_width_);
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Runtime gripper max width updated from mapping: %.4f m", msg->data);
}

}  // namespace arm_hand_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_hand_control::HandLandmarkGripperRetargeter>());
  rclcpp::shutdown();
  return 0;
}
