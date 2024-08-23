#ifndef QRINVIEW_HPP
#define QRINVIEW_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>
#include <thread>

class QRinView : public BT::ConditionNode
{
public:
    QRinView(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
};

#endif // QRINVIEW_HPP
