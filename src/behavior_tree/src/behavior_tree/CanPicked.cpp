#include "behavior_tree/CanPicked.hpp"

CanPicked::CanPicked(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("CanPicked"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node_->get_logger(), "CanPicked initialized.");

    // Initialize the publisher for the switch_to_qr topic
    switch_to_qr_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("switch_to_qr", 10);
}

BT::NodeStatus CanPicked::tick()
{
    // Check the blackboard key 'can_picked'
    bool can_picked = false;
    if (config().blackboard->get("can_picked", can_picked) && can_picked)
    {
        // Create a Bool message to publish
        auto msg = std_msgs::msg::Bool();
        msg.data = true;

        // Publish the message
        switch_to_qr_publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published true to switch_to_qr topic.");

        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList CanPicked::providedPorts()
{
    return {
        // Define any input or output ports for your node here
        // For example:
        // BT::InputPort<bool>("some_port_name");
    };
}
