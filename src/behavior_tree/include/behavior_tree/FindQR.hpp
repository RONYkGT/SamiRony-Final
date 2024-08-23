#ifndef FINDQR_HPP_
#define FINDQR_HPP_

#include <behaviortree_cpp_v3/action_node.h>

class FindQR : public BT::SyncActionNode
{
public:
    FindQR(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

#endif  // FINDQR_HPP_
