#include "behavior_tree/MoveForward.hpp"

MoveForward::MoveForward(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config), is_running_(false), node_(rclcpp::Node::make_shared("MoveForward"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
}

BT::NodeStatus MoveForward::tick()
{
    if (!is_running_)
    {
        start_time_ = std::chrono::steady_clock::now();
        is_running_ = true;
    }

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;

    if (elapsed.count() < 3.0)
    {
        RCLCPP_INFO(node_->get_logger(), "Moving forward");
        // Return RUNNING for the first 3 seconds
        return BT::NodeStatus::RUNNING;
    }
    else
    {
        // Return SUCCESS after 3 seconds
        is_running_ = false;
        RCLCPP_INFO(node_->get_logger(), "Moving forward complete");
        return BT::NodeStatus::SUCCESS;
    }
}
void MoveForward::halt()
{
    RCLCPP_DEBUG(node_->get_logger(), "MoveForward node halted");
}