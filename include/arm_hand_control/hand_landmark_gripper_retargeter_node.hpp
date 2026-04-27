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
#pragma once

#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_hand_control
{

/**
 * Hand landmark gripper retargeter node
 *
 * Subscribes to MediaPipe-style hand landmarks (21-point PoseArray) and converts
 * the thumb / index / middle fingertip pinch geometry into a 1-DoF parallel
 * gripper command (control_msgs/GripperCommand). The mapped width is published
 * on a topic that is intended to be consumed by hand_gripper_action_adapter.
 *
 * Retargeting model
 * -----------------
 * Each frame the node computes a scale-invariant pinch ratio:
 *
 *   pinch     = 0.5 * (||thumb_tip - index_tip|| + ||thumb_tip - middle_tip||)
 *   palm_ref  = ||wrist - middle_MCP||
 *   ratio     = pinch / palm_ref
 *
 * The ratio is then linearly mapped onto the gripper width range:
 *
 *   width = clamp((ratio - pinch_close_ratio) / (pinch_open_ratio - pinch_close_ratio), 0, 1)
 *           * gripper_max_width
 *
 * Effort is fixed to 0.0 by design (only position is retargeted).
 *
 * Calibrating pinch_open_ratio / pinch_close_ratio
 * ------------------------------------------------
 * The defaults (close=0.15, open=1.20) work for most users. To fine-tune for a
 * specific camera or user, just run the node and watch its INFO log. Every
 * second it prints the current ratio plus the running min/max observed since
 * startup, e.g.:
 *
 *   pinch ratio: now=0.18  observed=[0.12, 1.34]  width=0.001 m
 *
 * Calibration procedure (no extra tools required):
 *   1. Start the node and let it print a few lines.
 *   2. Hold a tight pinch (thumb touching index/middle) for ~2 seconds; read
 *      the new "observed min" -> use that as pinch_close_ratio.
 *   3. Spread thumb/index/middle apart in a comfortable maximum-open pose for
 *      ~2 seconds; read the new "observed max" -> use that as pinch_open_ratio.
 *   4. Restart the node with the new parameter values (or set them via
 *      `ros2 param set` at runtime if you have a parameter callback wired).
 *
 * Topics:
 * - Subscriptions:
 *   - hand_landmarks (geometry_msgs/PoseArray)
 * - Publications:
 *   - gripper_command (control_msgs/GripperCommand) -- remap at launch time
 *     to the consumer's expected topic (e.g. "hand_gripper_command").
 *
 * Parameters:
 * - gripper_max_width (double): Upper bound of the gripper width in metres (default 0.06)
 * - pinch_open_ratio (double): pinch/palm ratio mapped to gripper_max_width (default 1.2)
 * - pinch_close_ratio (double): pinch/palm ratio mapped to 0.0 (default 0.15)
 * - gripper_smooth_factor (double): EMA smoothing factor in [0, 1) (default 0.7)
 */
class HandLandmarkGripperRetargeter : public rclcpp::Node
{
public:
  HandLandmarkGripperRetargeter();
  ~HandLandmarkGripperRetargeter() override;

private:
  void landmark_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_subscriber_;
  rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_publisher_;

  // Retargeting parameters
  double gripper_max_width_;
  double pinch_open_ratio_;
  double pinch_close_ratio_;
  double gripper_smooth_factor_;

  // EMA state
  bool has_prev_gripper_width_;
  double prev_gripper_width_;

  // Running min/max of observed pinch ratio (logged at INFO to assist calibration)
  double observed_ratio_min_;
  double observed_ratio_max_;

  // MediaPipe hand landmark indices used by this node
  static constexpr int HAND_LANDMARK_COUNT = 21;
  static constexpr int WRIST_IDX = 0;
  static constexpr int THUMB_TIP_IDX = 4;
  static constexpr int INDEX_TIP_IDX = 8;
  static constexpr int MIDDLE_MCP_IDX = 9;
  static constexpr int MIDDLE_TIP_IDX = 12;
};

}  // namespace arm_hand_control
