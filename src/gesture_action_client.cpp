#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_hand_control/action/execute_gesture.hpp"

class GestureActionClient : public rclcpp::Node
{
public:
  using ExecuteGesture = arm_hand_control::action::ExecuteGesture;
  using GoalHandleExecuteGesture = rclcpp_action::ClientGoalHandle<ExecuteGesture>;

  explicit GestureActionClient(const std::string& node_name = "gesture_client") : Node(node_name)
  {
    this->client_ptr_ = rclcpp_action::create_client<ExecuteGesture>(this, "execute_gesture");

    this->timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GestureActionClient::send_goal, this));
  }

  void send_goal()
  {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = ExecuteGesture::Goal();

    // Parse command line arguments for gesture
    std::vector<std::string> args = this->get_parameter("args").as_string_array();
    if (args.size() >= 1)
    {
      goal_msg.gesture_name = args[0];
    }
    else
    {
      goal_msg.gesture_name = "open_hand";
    }

    // Parse duration if provided
    if (args.size() >= 2)
    {
      try
      {
        goal_msg.duration = std::stof(args[1]);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to parse duration, using default");
      }
    }

    // Parse percentage if provided
    if (args.size() >= 3)
    {
      try
      {
        goal_msg.percentage = std::stof(args[2]);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to parse percentage, using default");
      }
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal: %s (duration=%.2f, percentage=%.2f)", goal_msg.gesture_name.c_str(),
                goal_msg.duration, goal_msg.percentage);

    auto send_goal_options = rclcpp_action::Client<ExecuteGesture>::SendGoalOptions();
    send_goal_options.feedback_callback =
        std::bind(&GestureActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&GestureActionClient::result_callback, this, std::placeholders::_1);

    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteGesture>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void feedback_callback(GoalHandleExecuteGesture::SharedPtr,
                         const std::shared_ptr<const ExecuteGesture::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Gesture completion: " << static_cast<int>(feedback->percentage_complete * 100.0) << "%";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  void result_callback(const GoalHandleExecuteGesture::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<GestureActionClient>();

  // Pass command line arguments to the node
  if (argc > 1)
  {
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i)
    {
      args.push_back(argv[i]);
    }
    client_node->declare_parameter("args", args);
  }
  else
  {
    client_node->declare_parameter("args", std::vector<std::string>{ "open_hand" });
  }

  rclcpp::spin(client_node);
  rclcpp::shutdown();
  return 0;
}
