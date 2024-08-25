#ifndef ONBLACKSTRIP_HPP
#define ONBLACKSTRIP_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>


class OnBlackStrip : public BT::ConditionNode
{
public:
    OnBlackStrip(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg);

    bool on_black_strip_ = false;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
};

#endif // ONBLACKSTRIP_HPP
