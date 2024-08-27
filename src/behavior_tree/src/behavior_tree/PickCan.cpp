#include "behavior_tree/PickCan.hpp"


PickCan::PickCan(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config), can_picked_(false), node_(rclcpp::Node::make_shared("PickCan"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node_->get_logger(), "PickCan initialized.");
     // Initialize the publisher for the switch_to_qr topic
    switch_to_qr_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("switch_to_qr", 10);
    // Initialize the action client for the gripper
    gripper_action_client_ = rclcpp_action::create_client<robot_hardware_interfaces::action::GripperAction>(node_, "gripper_action");
}

BT::NodeStatus PickCan::tick()
{
    RCLCPP_INFO(node_->get_logger(), "Ticking PickCan action.");

    if (!can_picked_)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        switch_to_qr_publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published true to switch_to_qr topic.");
    }
    // Set the blackboard key 'can_picked' to true
    std::this_thread::sleep_for(std::chrono::seconds(5));
    can_picked_ = true;
    config().blackboard->set("can_picked", can_picked_);
    RCLCPP_INFO(node_->get_logger(), "PickCan action completed successfully.");
    return BT::NodeStatus::SUCCESS;


    // Check if the action server is available
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available.");
        return BT::NodeStatus::FAILURE;
    }

    // Create a goal to close the gripper
    auto goal = robot_hardware_interfaces::action::GripperAction::Goal();

    // Send the goal asynchronously
    auto send_goal_options = rclcpp_action::Client<robot_hardware_interfaces::action::GripperAction>::SendGoalOptions();
    auto goal_future = gripper_action_client_->async_send_goal(goal, send_goal_options);

    // Wait for the goal handle (but not the result)
    if (rclcpp::spin_until_future_complete(node_, goal_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send gripper action.");
        return BT::NodeStatus::FAILURE;
    }

    // Store the goal handle in the blackboard for potential cancellation later
    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server.");
        return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set("gripper_goal_handle", goal_handle);

    // Simulate the time to pick up the can
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Set the blackboard key 'can_picked' to true
    config().blackboard->set("can_picked", true);

    RCLCPP_INFO(node_->get_logger(), "PickCan action completed successfully.");
    return BT::NodeStatus::SUCCESS;
}
