#ifndef DROPCAN_HPP
#define DROPCAN_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <chrono>
#include <thread>

class DropCan : public BT::SyncActionNode
{
public:
    DropCan(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }
};

#endif // DROPCAN_HPP
