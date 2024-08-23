#ifndef CANPICKED_HPP
#define CANPICKED_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>
#include <thread>

class CanPicked : public BT::ConditionNode
{
public:
    CanPicked(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
};

#endif // CANPICKED_HPP
