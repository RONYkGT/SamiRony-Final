#include "robot_driver_cpp/move_right_action_server.hpp"

using namespace std::chrono_literals;

MoveRightActionServer::MoveRightActionServer() : Node("move_right_action_server")
{

    action_server_ = rclcpp_action::create_server<TurnRight>(
        this,
        "move_right",
        std::bind(&MoveRightActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveRightActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveRightActionServer::handle_accepted, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

rclcpp_action::GoalResponse MoveRightActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const TurnRight::Goal> goal
)
{
    RCLCPP_INFO(this->get_logger(), "Received request to move right.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveRightActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleTurnRight> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request. Stopping the robot.");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveRightActionServer::handle_accepted(const std::shared_ptr<GoalHandleTurnRight> goal_handle)
{
    std::thread{std::bind(&MoveRightActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void MoveRightActionServer::execute(const std::shared_ptr<GoalHandleTurnRight> goal_handle)
{
    auto goal = goal_handle->get_goal();
    turn_speed_ = goal->speed;
    RCLCPP_INFO(this->get_logger(), "Executing move right with speed %f...", turn_speed_);
    
    twist_msg_.linear.y = turn_speed_;
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

void MoveRightActionServer::publish_twist_msg()
{
    if (publisher_) {
        publisher_->publish(twist_msg_);
    }
}

void MoveRightActionServer::stop_robot()
{
    twist_msg_.linear.y = 0.0;
    twist_msg_.angular.z = 0.0;
    publisher_->publish(twist_msg_);
    if (timer_) {
        timer_->cancel();
    }
}
