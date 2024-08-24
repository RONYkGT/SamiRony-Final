#ifndef CANINVIEW_HPP
#define CANINVIEW_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "bot_behavior_interfaces/Constants.hpp" 

class CanInView : public BT::ConditionNode
{
public:
    CanInView(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    void callback(const std_msgs::msg::UInt8::SharedPtr msg);

    uint8_t can_position_;
    bool position_found = false;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
};

#endif // CANINVIEW_HPP
