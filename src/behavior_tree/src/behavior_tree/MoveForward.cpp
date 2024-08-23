#include "behavior_tree/MoveForward.hpp"

BT::NodeStatus MoveForward::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}
