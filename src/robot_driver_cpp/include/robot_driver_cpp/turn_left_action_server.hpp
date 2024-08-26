#ifndef TURN_LEFT_ACTION_SERVER_HPP
#define TURN_LEFT_ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bot_behavior_interfaces/action/turn_left.hpp"

class TurnLeftActionServer : public rclcpp::Node
{
public:
    using TurnLeft = bot_behavior_interfaces::action::TurnLeft;
    using GoalHandleTurnLeft = rclcpp_action::ServerGoalHandle<TurnLeft>;

    TurnLeftActionServer();

private:
    rclcpp_action::Server<TurnLeft>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double turn_speed_;
    geometry_msgs::msg::Twist twist_msg_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TurnLeft::Goal> goal
    );

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTurnLeft> goal_handle
    );

    void handle_accepted(const std::shared_ptr<GoalHandleTurnLeft> goal_handle);

    void execute(const std::shared_ptr<GoalHandleTurnLeft> goal_handle);

    void publish_twist_msg();

    void stop_robot();
};

#endif // TURN_LEFT_ACTION_SERVER_HPP
