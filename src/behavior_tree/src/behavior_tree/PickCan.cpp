#include "behavior_tree/PickCan.hpp"

BT::NodeStatus PickCan::tick()
{
    // Simulate picking the can (always successful in this example)
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Simulate some work

    // Set the blackboard key 'can_picked' to true
    config().blackboard->set("can_picked", true);

    return BT::NodeStatus::SUCCESS;
}
