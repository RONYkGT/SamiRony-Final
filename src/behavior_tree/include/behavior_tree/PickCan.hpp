#ifndef PICKCAN_HPP
#define PICKCAN_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <chrono>
#include <thread>

class PickCan : public BT::SyncActionNode
{
public:
    PickCan(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
};

#endif // PICKCAN_HPP
