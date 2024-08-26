#include "robot_driver_cpp/turn_right_action_server.hpp"

using namespace std::chrono_literals;

TurnRightActionServer::TurnRightActionServer() : Node("turn_right_action_server")
{
    this->declare_parameter<double>("turn_speed", 0.2);
    this->get_parameter("turn_speed", turn_speed_);

    action_server_ = rclcpp_action::create_server<TurnRight>(
        this,
        "turn_right",
        std::bind(&TurnRightActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TurnRightActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&TurnRightActionServer::handle_accepted, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

rclcpp_action::GoalResponse TurnRightActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const TurnRight::Goal> goal
)
{
    RCLCPP_INFO(this->get_logger(), "Received request to turn right.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurnRightActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleTurnRight> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request. Stopping the robot.");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurnRightActionServer::handle_accepted(const std::shared_ptr<GoalHandleTurnRight> goal_handle)
{
    std::thread{std::bind(&TurnRightActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void TurnRightActionServer::execute(const std::shared_ptr<GoalHandleTurnRight> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing turn right with speed %f...", turn_speed_);
    
    twist_msg_.angular.z = -turn_speed_;
    timer_ = this->create_wall_timer(
        100ms, [this]() { publish_twist_msg(); }
    );

    while (rclcpp::ok() && !goal_handle->is_canceling()) {
        std::this_thread::sleep_for(100ms);
    }

    if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled. Stopping the robot.");
        stop_robot();
        goal_handle->canceled(std::make_shared<TurnRight::Result>());
    } else {
        goal_handle->succeed(std::make_shared<TurnRight::Result>());
    }
}

void TurnRightActionServer::publish_twist_msg()
{
    if (publisher_) {
        publisher_->publish(twist_msg_);
    }
}

void TurnRightActionServer::stop_robot()
{
    twist_msg_.linear.x = 0.0;
    twist_msg_.angular.z = 0.0;
    publisher_->publish(twist_msg_);
    if (timer_) {
        timer_->cancel();
    }
}
