#include "behavior_tree/QRinView.hpp"

QRinView::QRinView(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), qr_position_(item_position::NOT_IN_VIEW), node_(rclcpp::Node::make_shared("QRinView")), position_found(false)
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    // Initialize ROS subscriber to the relevant topic
    subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "qr_in_view", 10, std::bind(&QRinView::callback, this, std::placeholders::_1));
}

void QRinView::callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    RCLCPP_DEBUG(node_->get_logger(), "Received qr in view data: %d", msg->data);
    qr_position_ = msg->data;
    position_found = true;
}

BT::NodeStatus QRinView::tick()
{
    // Ensure the node processes any pending callbacks
    rclcpp::spin_some(node_);

    // Check if the position has been found and act accordingly
    if (position_found)
    {
        if (qr_position_ == item_position::CENTER)
        {
            RCLCPP_DEBUG(node_->get_logger(), "qr in center view");
            position_found = false;  // Reset for the next tick
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(), "qr not in center view");
            position_found = false;  // Reset for the next tick
            return BT::NodeStatus::FAILURE;
        }
    }
    else
    {
        RCLCPP_DEBUG(node_->get_logger(), "Waiting for qr position");
        // Return RUNNING while waiting for the message to arrive
        return BT::NodeStatus::FAILURE;
    }
}
