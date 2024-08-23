#include "behavior_tree/DropCan.hpp"

BT::NodeStatus DropCan::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}
