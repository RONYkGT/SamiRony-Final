#include "behavior_tree/FindQR.hpp"
#include <chrono>
#include <thread>

FindQR::FindQR(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList FindQR::providedPorts() {
    return {};
}

BT::NodeStatus FindQR::tick()
{
    // Placeholder implementation: Wait for 3 seconds
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Return success (can be adjusted based on actual implementation)
    return BT::NodeStatus::SUCCESS;
}
