#include "behavior_tree/OnStopStrip.hpp"

OnStopStrip::OnStopStrip(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), binary_(0), node_(rclcpp::Node::make_shared("OnStopStrip"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    // Initialize ROS subscriber to the relevant topic
    subscriber_ = node_->create_subscription<std_msgs::msg::Int8>(
        "lineDetectionData", 10, std::bind(&OnStopStrip::callback, this, std::placeholders::_1));
}

void OnStopStrip::callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    on_black_strip_ = msg->data;
    binary_ = std::bitset<5>(on_black_strip_);
    RCLCPP_DEBUG(node_->get_logger(), "Received on black strip data: %s", binary_.to_string().c_str());

}

BT::NodeStatus OnStopStrip::tick()
{
    // Ensure the node processes any pending callbacks
    rclcpp::spin_some(node_);

    // Check if the position has been found and act accordingly
    if (binary_.count() > 4)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Can on stop strip");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_DEBUG(node_->get_logger(), "Can not on stop strip");
        // Return RUNNING while waiting for the message to arrive
        return BT::NodeStatus::FAILURE;
    }
}
