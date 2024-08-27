#include "robot_driver_cpp/turn_left_action_server.hpp"

using namespace std::chrono_literals;

TurnLeftActionServer::TurnLeftActionServer() : Node("turn_left_action_server")
{

    action_server_ = rclcpp_action::create_server<TurnLeft>(
        this,
        "turn_left",
        std::bind(&TurnLeftActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TurnLeftActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&TurnLeftActionServer::handle_accepted, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

rclcpp_action::GoalResponse TurnLeftActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const TurnLeft::Goal> goal
)
{
    RCLCPP_INFO(this->get_logger(), "Received request to turn left.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurnLeftActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleTurnLeft> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request. Stopping the robot.");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurnLeftActionServer::handle_accepted(const std::shared_ptr<GoalHandleTurnLeft> goal_handle)
{
    std::thread{std::bind(&TurnLeftActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void TurnLeftActionServer::execute(const std::shared_ptr<GoalHandleTurnLeft> goal_handle)
{
    auto goal = goal_handle->get_goal();
    turn_speed_ = goal->speed;
    RCLCPP_INFO(this->get_logger(), "Executing turn left with speed %f...", turn_speed_);
    
    twist_msg_.angular.z = turn_speed_;
    timer_ = this->create_wall_timer(
        100ms, [this]() { publish_twist_msg(); }
    );

    while (rclcpp::ok() && !goal_handle->is_canceling()) {
        std::this_thread::sleep_for(100ms);
    }

    if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled. Stopping the robot.");
        stop_robot();
        goal_handle->canceled(std::make_shared<TurnLeft::Result>());
    } else {
        goal_handle->succeed(std::make_shared<TurnLeft::Result>());
    }
}

void TurnLeftActionServer::publish_twist_msg()
{
    if (publisher_) {
        publisher_->publish(twist_msg_);
    }
}

void TurnLeftActionServer::stop_robot()
{
    twist_msg_.linear.x = 0.0;
    twist_msg_.angular.z = 0.0;
    publisher_->publish(twist_msg_);
    if (timer_) {
        timer_->cancel();
    }
}
