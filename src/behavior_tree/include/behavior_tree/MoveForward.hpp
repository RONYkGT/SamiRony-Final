#ifndef MOVEFORWARD_HPP
#define MOVEFORWARD_HPP
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include "bot_behavior_interfaces/action/move_forward.hpp"

class MoveForward : public BT::ActionNodeBase
{
public:
    MoveForward(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;
    void halt() override;
    
    static BT::PortsList providedPorts() {
        return {};
    }

private:
    std::chrono::steady_clock::time_point start_time_;
    rclcpp_action::Client<bot_behavior_interfaces::action::MoveForward>::SharedPtr move_forward_client_;
    rclcpp_action::ClientGoalHandle<bot_behavior_interfaces::action::MoveForward>::SharedPtr move_forward_handle_;
    bool is_running_;
    rclcpp::Node::SharedPtr node_;

};

#endif // MOVEFORWARD_HPP
