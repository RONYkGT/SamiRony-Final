#include "behavior_tree/CanPicked.hpp"

BT::NodeStatus CanPicked::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}
