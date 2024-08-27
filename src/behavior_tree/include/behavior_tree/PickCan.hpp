#ifndef PICKCAN_HPP
#define PICKCAN_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_hardware_interfaces/action/gripper_action.hpp"
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <thread>

class PickCan : public BT::SyncActionNode
{
public:
    // Constructor
    PickCan(const std::string &name, const BT::NodeConfiguration &config);

    // Override tick() from BT::SyncActionNode
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    // ROS 2 node pointer
    rclcpp::Node::SharedPtr node_;
    bool can_picked_;
    // Action client for the gripper action
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr switch_to_qr_publisher_;
    rclcpp_action::Client<robot_hardware_interfaces::action::GripperAction>::SharedPtr gripper_action_client_;
};


#endif // PICKCAN_HPP
