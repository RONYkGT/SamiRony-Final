#ifndef CAN_CLOSE_HPP
#define CAN_CLOSE_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class CanClose : public BT::ConditionNode
{
public:
    CanClose(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
    {
        return {};
    }
    static BT::NodeConfiguration 
    configure()
    {
        return BT::NodeConfiguration{};
    }

private:
    rclcpp::Node::SharedPtr node_;
};

#endif // CAN_CLOSE_HPP
