#include "behavior_tree/QRinView.hpp"

BT::NodeStatus QRinView::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}
