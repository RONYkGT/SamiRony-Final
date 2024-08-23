#include "behavior_tree/CanInView.hpp"

BT::NodeStatus CanInView::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}
