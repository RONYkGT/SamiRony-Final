#include "behavior_tree/PickCan.hpp"

BT::NodeStatus PickCan::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}
