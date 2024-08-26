#include "robot_driver_cpp/move_forward_action_server.hpp"

using namespace std::chrono_literals;

MoveForwardActionServer::MoveForwardActionServer() : Node("move_forward_action_server")
{
    this->declare_parameter<double>("move_speed", 0.2);
    this->get_parameter("move_speed", move_speed_);

    auto options = rclcpp_action::ServerOptions();
    options.result_timeout = std::chrono::milliseconds(0);
    options.goal_handling_timeout = std::chrono::milliseconds(0);
    
    action_server_ = rclcpp_action::create_server<MoveForward>(
        this,
        "move_forward",
        std::bind(&MoveForwardActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveForwardActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveForwardActionServer::handle_accepted, this, std::placeholders::_1),
        options
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

rclcpp_action::GoalResponse MoveForwardActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveForward::Goal> goal
)
{
    RCLCPP_INFO(this->get_logger(), "Received request to move forward.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveForwardActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMoveForward> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request. Stopping the robot.");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveForwardActionServer::handle_accepted(const std::shared_ptr<GoalHandleMoveForward> goal_handle)
{
    std::thread{std::bind(&MoveForwardActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void MoveForwardActionServer::execute(const std::shared_ptr<GoalHandleMoveForward> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing move forward with speed %f...", move_speed_);
    
    twist_msg_.linear.x = move_speed_;
    timer_ = this->create_wall_timer(
        100ms, [this]() { publish_twist_msg(); }
    );

    while (rclcpp::ok() && !goal_handle->is_canceling()) {
        std::this_thread::sleep_for(100ms);
    }

    if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled. Stopping the robot.");
        stop_robot();
        goal_handle->canceled(std::make_shared<MoveForward::Result>());
    } else {
        goal_handle->succeed(std::make_shared<MoveForward::Result>());
    }
}

void MoveForwardActionServer::publish_twist_msg()
{
    if (publisher_) {
        publisher_->publish(twist_msg_);
    }
}

void MoveForwardActionServer::stop_robot()
{
    twist_msg_.linear.x = 0.0;
    twist_msg_.angular.z = 0.0;
    publisher_->publish(twist_msg_);
    if (timer_) {
        timer_->cancel();
    }
}
