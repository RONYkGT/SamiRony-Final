#include "behavior_tree/CanInView.hpp"

CanInView::CanInView(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), can_position_(item_position::NOT_IN_VIEW), node_(rclcpp::Node::make_shared("CanInView")), position_found(false)
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    // Initialize ROS subscriber to the relevant topic
    subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "can_in_view", 10, std::bind(&CanInView::callback, this, std::placeholders::_1));
}

void CanInView::callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    RCLCPP_DEBUG(node_->get_logger(), "Received can in view data: %d", msg->data);
    can_position_ = msg->data;
    position_found = true;
}

BT::NodeStatus CanInView::tick()
{
    // Ensure the node processes any pending callbacks
    rclcpp::spin_some(node_);

    // Check if the position has been found and act accordingly
    if (position_found)
    {
        if (can_position_ == item_position::CENTER)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Can in center view");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(), "Can not in center view");
            return BT::NodeStatus::FAILURE;
        }
    }
    else
    {
        RCLCPP_DEBUG(node_->get_logger(), "Waiting for can position");
        // Return RUNNING while waiting for the message to arrive
        return BT::NodeStatus::FAILURE;
    }
}
