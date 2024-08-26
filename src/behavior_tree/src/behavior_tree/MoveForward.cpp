#include "behavior_tree/MoveForward.hpp"

MoveForward::MoveForward(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config), is_running_(false), node_(rclcpp::Node::make_shared("MoveForward"))
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node_->get_logger(), "MoveForward initialized");

    // Initialize action clients
    move_forward_client_ = rclcpp_action::create_client<bot_behavior_interfaces::action::MoveForward>(node_, "move_forward");

}

BT::NodeStatus MoveForward::tick()
{
    RCLCPP_INFO(node_->get_logger(), "Running MoveForward action.");
    rclcpp::spin_some(node_);

    if (!move_forward_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node_->get_logger(), "Forward Action server not available. Waiting...");
        return BT::NodeStatus::RUNNING;
    }

    if (!move_forward_handle_)
        {
            auto goal = bot_behavior_interfaces::action::MoveForward::Goal();
            auto send_goal_options = rclcpp_action::Client<bot_behavior_interfaces::action::MoveForward>::SendGoalOptions();
            send_goal_options.feedback_callback = [](auto, auto){};
            auto goal_future = move_forward_client_->async_send_goal(goal, send_goal_options);
            rclcpp::spin_until_future_complete(node_, goal_future);
            move_forward_handle_ = goal_future.get();
            is_running_ = true;
        }
    return BT::NodeStatus::RUNNING;
}
void MoveForward::halt()
{
    RCLCPP_DEBUG(node_->get_logger(), "MoveForward node halted");
    if (is_running_)
    {
        if (move_forward_handle_)
        {
            auto cancel_future = move_forward_client_->async_cancel_goal(move_forward_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "MoveForward action canceled.");
            move_forward_handle_.reset();
        }
        is_running_ = false;
    }
}