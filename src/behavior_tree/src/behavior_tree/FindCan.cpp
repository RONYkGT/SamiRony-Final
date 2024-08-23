#include "behavior_tree/FindCan.hpp"

BT::NodeStatus FindCan::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}
