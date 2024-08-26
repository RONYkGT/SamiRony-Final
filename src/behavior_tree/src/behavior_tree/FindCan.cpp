#include "behavior_tree/FindCan.hpp"

FindCan::FindCan(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared("FindCan")),
      turning_in_progress_(false), can_position_(item_position::NOT_IN_VIEW)
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node_->get_logger(), "FindCan initialized %d", item_position::CENTER);

    // Initialize action clients
    turn_left_client_ = rclcpp_action::create_client<bot_behavior_interfaces::action::TurnLeft>(node_, "turn_left");
    turn_right_client_ = rclcpp_action::create_client<bot_behavior_interfaces::action::TurnRight>(node_, "turn_right");

    // Subscribe to can_in_view topic
    subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "can_in_view", 10, std::bind(&FindCan::callback, this, std::placeholders::_1));
}

void FindCan::callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "Received msg callback: %d", msg->data);
    can_position_ = msg->data;

}


BT::NodeStatus FindCan::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "Starting FindCan action.");

    // If can is already in the center, return SUCCESS
    if (can_position_ == item_position::CENTER)
    {
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FindCan::onRunning()
{
    RCLCPP_INFO(node_->get_logger(), "Running FindCan action.");
    rclcpp::spin_some(node_);
    RCLCPP_INFO(node_->get_logger(), "Node spinned");
 
    // If can is centered, halt ongoing actions and return SUCCESS
    if (can_position_ == item_position::CENTER)
    {
        RCLCPP_INFO(node_->get_logger(), "2");
        if (turning_in_progress_)
        {
            halt();
        }
        return BT::NodeStatus::RUNNING;
    }

    // Check if action server is available before sending a goal
    if (!turn_left_client_->wait_for_action_server(std::chrono::seconds(1)) ||
        !turn_right_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node_->get_logger(), "Action server not available. Waiting...");
        return BT::NodeStatus::RUNNING;
    }

    // Handle LEFT or NOT_IN_VIEW condition
    if (can_position_ == item_position::LEFT || can_position_ == item_position::NOT_IN_VIEW)
    {
        RCLCPP_INFO(node_->get_logger(), "3");
        if (turn_right_goal_handle_)
        {
            auto cancel_future = turn_right_client_->async_cancel_goal(turn_right_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnRight action canceled.1");
            turn_right_goal_handle_.reset();
        }
        if (!turn_left_goal_handle_)
        {
            auto goal = bot_behavior_interfaces::action::TurnLeft::Goal();
            auto send_goal_options = rclcpp_action::Client<bot_behavior_interfaces::action::TurnLeft>::SendGoalOptions();
            send_goal_options.feedback_callback = [](auto, auto){};
            auto goal_future = turn_left_client_->async_send_goal(goal, send_goal_options);
            rclcpp::spin_until_future_complete(node_, goal_future);
            turn_left_goal_handle_ = goal_future.get();
            turning_in_progress_ = true;
        }
        return BT::NodeStatus::RUNNING;
    }

    // Handle RIGHT condition
    if (can_position_ == item_position::RIGHT)
    {
        RCLCPP_INFO(node_->get_logger(), "4");
        if (turn_left_goal_handle_)
        {
            auto cancel_future = turn_left_client_->async_cancel_goal(turn_left_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnLeft action canceled.1");
            turn_left_goal_handle_.reset();
        }
        if (!turn_right_goal_handle_)
        {
            auto goal = bot_behavior_interfaces::action::TurnRight::Goal();
            auto send_goal_options = rclcpp_action::Client<bot_behavior_interfaces::action::TurnRight>::SendGoalOptions();
            send_goal_options.feedback_callback = [](auto, auto){};
            auto goal_future = turn_right_client_->async_send_goal(goal, send_goal_options);
            rclcpp::spin_until_future_complete(node_, goal_future);
            turn_right_goal_handle_ = goal_future.get();
            turning_in_progress_ = true;
        }
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(node_->get_logger(), "5");
    return BT::NodeStatus::RUNNING;
}

void FindCan::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "FindCan action halted.");

    if (turning_in_progress_)
    {
        if (turn_left_goal_handle_)
        {
            auto cancel_future = turn_left_client_->async_cancel_goal(turn_left_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnLeft action canceled.");
            turn_left_goal_handle_.reset();
        }
        if (turn_right_goal_handle_)
        {
            auto cancel_future = turn_right_client_->async_cancel_goal(turn_right_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnRight action canceled.");
            turn_right_goal_handle_.reset();
        }
        turning_in_progress_ = false;
    }
}
