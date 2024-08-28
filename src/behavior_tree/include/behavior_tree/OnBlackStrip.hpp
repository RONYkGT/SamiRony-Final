#ifndef ONBLACKSTRIP_HPP
#define ONBLACKSTRIP_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <bitset>


class OnBlackStrip : public BT::ConditionNode
{
public:
    OnBlackStrip(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    void callback(const std_msgs::msg::Int8::SharedPtr msg);

    int on_black_strip_ = 0;
    std::bitset<5> binary_; 
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscriber_;
};

#endif // ONBLACKSTRIP_HPP
