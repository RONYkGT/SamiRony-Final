#ifndef FINDCAN_HPP
#define FINDCAN_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <chrono>
#include <thread>

class FindCan : public BT::SyncActionNode
{
public:
    FindCan(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
};

#endif // FINDCAN_HPP
