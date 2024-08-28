#include "behavior_tree/OnBlackStrip.hpp"

OnBlackStrip::OnBlackStrip(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), node_(rclcpp::Node::make_shared("OnBlackStrip"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    // Initialize ROS subscriber to the relevant topic
    subscriber_ = node_->create_subscription<std_msgs::msg::Int8>(
        "lineDetectionData", 10, std::bind(&OnBlackStrip::callback, this, std::placeholders::_1));
}

void OnBlackStrip::callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    on_black_strip_ = msg->data;
    RCLCPP_DEBUG(node_->get_logger(), "Received on black strip data: %d", on_black_strip_);

}

BT::NodeStatus OnBlackStrip::tick()
{
    // Ensure the node processes any pending callbacks
    rclcpp::spin_some(node_);

    // Check if the position has been found and act accordingly
    if (on_black_strip_ > 0)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Can on black strip");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_DEBUG(node_->get_logger(), "Can not on black strip");
        // Return RUNNING while waiting for the message to arrive
        return BT::NodeStatus::FAILURE;
    }
}
