#ifndef FIND_CAN_HPP
#define FIND_CAN_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/rclcpp.hpp>
#include "bot_behavior_interfaces/action/turn_left.hpp"
#include "bot_behavior_interfaces/action/turn_right.hpp"
#include "bot_behavior_interfaces/Constants.hpp" 

class FindCan : public BT::ActionNodeBase
{
public:
    FindCan(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;
    void halt() override;

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    void callback(const std_msgs::msg::UInt8::SharedPtr msg);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
    rclcpp_action::Client<bot_behavior_interfaces::action::TurnLeft>::SharedPtr turn_left_client_;
    rclcpp_action::Client<bot_behavior_interfaces::action::TurnRight>::SharedPtr turn_right_client_;
    rclcpp_action::ClientGoalHandle<bot_behavior_interfaces::action::TurnLeft>::SharedPtr turn_left_goal_handle_;
    rclcpp_action::ClientGoalHandle<bot_behavior_interfaces::action::TurnRight>::SharedPtr turn_right_goal_handle_;
    bool turning_in_progress_ = false;
    uint8_t can_position_ = item_position::NOT_IN_VIEW;  // 0=NOT_IN_VIEW, 1=LEFT, 2=CENTER, 3=RIGHT
};

#endif // FIND_CAN_HPP
