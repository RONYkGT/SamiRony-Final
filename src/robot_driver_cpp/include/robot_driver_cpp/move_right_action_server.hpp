#ifndef MOVE_RIGHT_ACTION_SERVER_HPP
#define MOVE_RIGHT_ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bot_behavior_interfaces/action/turn_right.hpp"

class MoveRightActionServer : public rclcpp::Node
{
public:
    using TurnRight = bot_behavior_interfaces::action::TurnRight;
    using GoalHandleTurnRight = rclcpp_action::ServerGoalHandle<TurnRight>;

    MoveRightActionServer();

private:
    rclcpp_action::Server<TurnRight>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double turn_speed_;
    geometry_msgs::msg::Twist twist_msg_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TurnRight::Goal> goal
    );

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTurnRight> goal_handle
    );

    void handle_accepted(const std::shared_ptr<GoalHandleTurnRight> goal_handle);

    void execute(const std::shared_ptr<GoalHandleTurnRight> goal_handle);

    void publish_twist_msg();

    void stop_robot();
};

#endif // MOVE_RIGHT_ACTION_SERVER_HPP
