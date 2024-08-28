#ifndef DROPCAN_HPP
#define DROPCAN_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_hardware_interfaces/action/gripper_action.hpp"
#include <chrono>
#include <thread>

class DropCan : public BT::SyncActionNode
{
public:
    DropCan(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
private:
    // ROS 2 node pointer
    rclcpp::Node::SharedPtr node_;
    // Action client for the gripper action
    rclcpp_action::Client<robot_hardware_interfaces::action::GripperAction>::SharedPtr gripper_action_client_;
};

#endif // DROPCAN_HPP
