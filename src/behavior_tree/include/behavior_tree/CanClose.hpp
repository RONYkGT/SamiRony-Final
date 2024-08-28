#ifndef CANCLOSE_HPP
#define CANCLOSE_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"


class CanClose : public BT::ConditionNode
{
public:
    CanClose(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    void callback(const std_msgs::msg::Float32::SharedPtr msg);

    float can_distance_;
    float max_distance;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
};

#endif // CANCLOSE_HPP
