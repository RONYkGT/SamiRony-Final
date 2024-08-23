#ifndef CANINVIEW_HPP
#define CANINVIEW_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>
#include <thread>

class CanInView : public BT::ConditionNode
{
public:
    CanInView(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
};

#endif // CANINVIEW_HPP
