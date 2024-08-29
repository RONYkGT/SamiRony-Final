#ifndef FIND_CAN_HPP
#define FIND_CAN_HPP

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include "bot_behavior_interfaces/Constants.hpp" 
#include "bot_behavior_interfaces/action/turn_left.hpp"
#include "bot_behavior_interfaces/action/turn_right.hpp"

class FindCan : public BT::StatefulActionNode
{
public:
    FindCan(const std::string &name, const BT::NodeConfiguration &config);

    // The overridden methods for the stateful action node
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts() {
        return {};
    }

    // Callback function for the subscription
    void callback(const std_msgs::msg::UInt8::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
    rclcpp_action::Client<bot_behavior_interfaces::action::TurnLeft>::SharedPtr turn_left_client_;
    rclcpp_action::Client<bot_behavior_interfaces::action::TurnRight>::SharedPtr turn_right_client_;

    rclcpp_action::ClientGoalHandle<bot_behavior_interfaces::action::TurnLeft>::SharedPtr turn_left_goal_handle_;
    rclcpp_action::ClientGoalHandle<bot_behavior_interfaces::action::TurnRight>::SharedPtr turn_right_goal_handle_;

    bool turning_in_progress_;
    uint8_t can_position_;
    float fast_turn_speed;
    float slow_turn_speed;
};

#endif // FIND_CAN_HPP
