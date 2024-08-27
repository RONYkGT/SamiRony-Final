#ifndef CAN_PICKED_HPP
#define CAN_PICKED_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <std_msgs/msg/bool.hpp>

class CanPicked : public BT::SyncActionNode
{
public:
    // Constructor
    CanPicked(const std::string &name, const BT::NodeConfiguration &config);

    // Override tick() from BT::SyncActionNode
    BT::NodeStatus tick() override;

    // Static method to define ports
    static BT::PortsList providedPorts();

private:
    // ROS 2 node pointer
    rclcpp::Node::SharedPtr node_;

    // Publisher for the switch_to_qr topic
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr switch_to_qr_publisher_;
};

#endif // CAN_PICKED_HPP
