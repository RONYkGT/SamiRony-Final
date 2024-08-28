#include "behavior_tree/CenterLine.hpp"

CenterLine::CenterLine(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config), node_(rclcpp::Node::make_shared("CenterLine")),
      moving_in_progress_(false), binary_(0)
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node_->get_logger(), "CenterLine initialized %d", item_position::CENTER);

    // Initialize action clients
    move_left_client_ = rclcpp_action::create_client<bot_behavior_interfaces::action::TurnLeft>(node_, "move_left");
    move_right_client_ = rclcpp_action::create_client<bot_behavior_interfaces::action::TurnRight>(node_, "move_right");

    // Subscribe to qr_in_view topic
    subscriber_ = node_->create_subscription<std_msgs::msg::Int8>(
        "lineDetectionData", 10, std::bind(&CenterLine::callback, this, std::placeholders::_1));
}

void CenterLine::callback(const std_msgs::msg::Int8::SharedPtr msg)
{

    on_black_strip_ = msg->data;
    binary_ = std::bitset<5>(on_black_strip_);
    RCLCPP_DEBUG(node_->get_logger(), "Received on black strip data: %s", binary_.to_string().c_str());
}

BT::NodeStatus CenterLine::tick()
{
    int line_position_ = 0;
    RCLCPP_INFO(node_->get_logger(), "Running CenterLine action.");
    rclcpp::spin_some(node_);
    RCLCPP_INFO(node_->get_logger(), "Node spinned");

    if (on_black_strip_ = 0)
        line_position_ = item_position::NOT_IN_VIEW;

    else if(binary_[0] == 0 && binary_[4] == 0 && binary_[2] == 1)
        line_position_ = item_position::CENTER;

    else if (on_black_strip_ < 8)
        line_position_ = item_position::RIGHT;

    else
        line_position_ = item_position::LEFT;

    // If line is centered, halt ongoing actions and return RUNNING
    if (line_position_ == item_position::CENTER)
    {
        RCLCPP_INFO(node_->get_logger(), "2");
        if (moving_in_progress_)
        {
            halt();
        }
        return BT::NodeStatus::RUNNING;
    }

    // Check if action server is available before sending a goal
    if (!move_left_client_->wait_for_action_server(std::chrono::seconds(1)) ||
        !move_right_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node_->get_logger(), "Action server not available. Waiting...");
        return BT::NodeStatus::RUNNING;
    }

    // Determine the appropriate speed based on QR code visibility
    float speed = (line_position_ == item_position::NOT_IN_VIEW) ? 0.3f : 0.1f;

    // Handle LEFT or NOT_IN_VIEW condition
    if (line_position_ == item_position::LEFT || line_position_ == item_position::NOT_IN_VIEW)
    {
        RCLCPP_INFO(node_->get_logger(), "3");
        if (move_right_goal_handle_)
        {
            auto cancel_future = move_right_client_->async_cancel_goal(move_right_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnRight action canceled.1");
            move_right_goal_handle_.reset();
        }
        if (!move_left_goal_handle_)
        {
            auto goal = bot_behavior_interfaces::action::TurnLeft::Goal();
            goal.speed = speed; // Set the speed
            auto send_goal_options = rclcpp_action::Client<bot_behavior_interfaces::action::TurnLeft>::SendGoalOptions();
            send_goal_options.feedback_callback = [](auto, auto){};
            auto goal_future = move_left_client_->async_send_goal(goal, send_goal_options);
            rclcpp::spin_until_future_complete(node_, goal_future);
            move_left_goal_handle_ = goal_future.get();
            moving_in_progress_ = true;
        }
        return BT::NodeStatus::RUNNING;
    }

    // Handle RIGHT condition
    if (line_position_ == item_position::RIGHT)
    {
        RCLCPP_INFO(node_->get_logger(), "4");
        if (move_left_goal_handle_)
        {
            auto cancel_future = move_left_client_->async_cancel_goal(move_left_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnLeft action canceled.1");
            move_left_goal_handle_.reset();
        }
        if (!move_right_goal_handle_)
        {
            auto goal = bot_behavior_interfaces::action::TurnRight::Goal();
            goal.speed = speed; // Set the speed
            auto send_goal_options = rclcpp_action::Client<bot_behavior_interfaces::action::TurnRight>::SendGoalOptions();
            send_goal_options.feedback_callback = [](auto, auto){};
            auto goal_future = move_right_client_->async_send_goal(goal, send_goal_options);
            rclcpp::spin_until_future_complete(node_, goal_future);
            move_right_goal_handle_ = goal_future.get();
            moving_in_progress_ = true;
        }
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(node_->get_logger(), "5");
    return BT::NodeStatus::RUNNING;
}

void CenterLine::halt()
{
    RCLCPP_INFO(node_->get_logger(), "CenterLine action halted.");

    if (moving_in_progress_)
    {
        if (move_left_goal_handle_)
        {
            auto cancel_future = move_left_client_->async_cancel_goal(move_left_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnLeft action canceled.");
            move_left_goal_handle_.reset();
        }
        if (move_right_goal_handle_)
        {
            auto cancel_future = move_right_client_->async_cancel_goal(move_right_goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
            RCLCPP_INFO(node_->get_logger(), "TurnRight action canceled.");
            move_right_goal_handle_.reset();
        }
        moving_in_progress_ = false;
    }
}
