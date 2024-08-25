#include "behavior_tree/CanClose.hpp"

CanClose::CanClose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), can_distance_(std::numeric_limits<double>::max()), max_distance(0.5), node_(rclcpp::Node::make_shared("CanClose"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);

    // Initialize ROS subscriber to the laser scan topic
    subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&CanClose::callback, this, std::placeholders::_1));
}

void CanClose::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!msg->ranges.empty())
    {
        can_distance_ = msg->ranges[0];  // Assuming the first range is the one to be checked
        RCLCPP_DEBUG(node_->get_logger(), "Received laser scan data: distance to can = %f", can_distance_);
    }
}

BT::NodeStatus CanClose::tick()
{
    // Ensure the node processes any pending callbacks
    rclcpp::spin_some(node_);

    // Check if the distance is within the threshold
    if (can_distance_ < max_distance)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Can is within close range: %f", can_distance_);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_DEBUG(node_->get_logger(), "Can is not close enough: %f", can_distance_);
        return BT::NodeStatus::FAILURE;
    }
}
