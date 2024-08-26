#ifndef MOVE_FORWARD_ACTION_SERVER_HPP
#define MOVE_FORWARD_ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bot_behavior_interfaces/action/move_forward.hpp"

class MoveForwardActionServer : public rclcpp::Node
{
public:
    using MoveForward = bot_behavior_interfaces::action::MoveForward;
    using GoalHandleMoveForward = rclcpp_action::ServerGoalHandle<MoveForward>;

    MoveForwardActionServer();

private:
    rclcpp_action::Server<MoveForward>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double move_speed_;
    geometry_msgs::msg::Twist twist_msg_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveForward::Goal> goal
    );

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveForward> goal_handle
    );

    void handle_accepted(const std::shared_ptr<GoalHandleMoveForward> goal_handle);

    void execute(const std::shared_ptr<GoalHandleMoveForward> goal_handle);

    void publish_twist_msg();

    void stop_robot();
};

#endif // MOVE_FORWARD_ACTION_SERVER_HPP
