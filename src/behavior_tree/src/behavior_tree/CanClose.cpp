#include "behavior_tree/CanClose.hpp"

CanClose::CanClose(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config), node_(rclcpp::Node::make_shared("CanClose"))
{
    // Optionally configure the node or set parameters here
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
}

BT::NodeStatus CanClose::tick()
{
    // Example debug message
    RCLCPP_DEBUG(node_->get_logger(), "Entering CanClose node");

    // Simulate work
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Example debug message
    RCLCPP_DEBUG(node_->get_logger(), "Exiting CanClose node");

    return BT::NodeStatus::FAILURE;
}
