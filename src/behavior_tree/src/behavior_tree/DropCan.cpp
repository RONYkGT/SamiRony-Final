#include "behavior_tree/DropCan.hpp"

DropCan::DropCan(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config), gripper_action_client_(NULL), node_(rclcpp::Node::make_shared("DropCan"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node_->get_logger(), "DropCan initialized.");
}

BT::NodeStatus DropCan::tick()
{
    RCLCPP_INFO(node_->get_logger(), "Ticking DropCan action.");

    // Retrieve the goal handle from the blackboard
    rclcpp_action::Client<robot_hardware_interfaces::action::GripperAction>::GoalHandle::SharedPtr goal_handle;
    if (!config().blackboard->get("gripper_goal_handle", goal_handle))
    {
        RCLCPP_WARN(node_->get_logger(), "No gripper goal handle found to cancel.");
        return BT::NodeStatus::SUCCESS;
    }
    if (!config().blackboard->get("gripper_action_client_", gripper_action_client_))
    {
        RCLCPP_WARN(node_->get_logger(), "No gripper_action_client_ found");
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(node_->get_logger(), "All found");
    if (goal_handle)
    {
        // Cancel the action if a goal handle is available and valid
        auto cancel_future = gripper_action_client_->async_cancel_goal(goal_handle);

        RCLCPP_DEBUG(node_->get_logger(), "Cancel sent, waiting");
        // Wait for the future to complete

        // Check the response of the cancellation request
        RCLCPP_DEBUG(node_->get_logger(), "Done");
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "No gripper goal handle found to cancel.");
    }

    return BT::NodeStatus::SUCCESS;
}
