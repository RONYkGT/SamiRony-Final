#include "behavior_tree/CanPicked.hpp"

BT::NodeStatus CanPicked::tick()
{
    // Check the blackboard key 'can_picked'
    bool can_picked = false;
    if (config().blackboard->get("can_picked", can_picked) && can_picked)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}