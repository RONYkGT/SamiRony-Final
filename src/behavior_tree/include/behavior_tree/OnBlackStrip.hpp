#ifndef ONBLACKSTRIP_HPP
#define ONBLACKSTRIP_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>
#include <thread>

class OnBlackStrip : public BT::ConditionNode
{
public:
    OnBlackStrip(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
};

#endif // ONBLACKSTRIP_HPP
