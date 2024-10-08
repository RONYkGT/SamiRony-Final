#include "behavior_tree/FindQR.hpp"

FindQR::FindQR(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config), node_(rclcpp::Node::make_shared("FindQR")),
      turning_in_progress_(false), qr_position_(item_position::NOT_IN_VIEW)
{
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node_->get_logger(), "FindQR initialized %d", item_position::CENTER);

    node_->declare_parameter("slow_turn_speed", 0.1);
    node_->get_parameter("slow_turn_speed", slow_turn_speed);
    node_->declare_parameter("fast_turn_speed", 0.5);
    node_->get_parameter("fast_turn_speed", fast_turn_speed);

    // Initialize action clients
    turn_left_client_ = rclcpp_action::create_client<bot_behavior_interfaces::action::TurnLeft>(node_, "turn_left");
    turn_right_client_ = rclcpp_action::create_client<bot_behavior_interfaces::action::TurnRight>(node_, "turn_right");

    // Subscribe to qr_in_view topic
    subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "qr_in_view", 10, std::bind(&FindQR::callback, this, std::placeholders::_1));
}

void FindQR::callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "Received msg callback: %d", msg->data);
    qr_position_ = msg->data;
}

BT::NodeStatus FindQR::tick()
{
    RCLCPP_INFO(node_->get_logger(), "Running FindQR action.");
    rclcpp::spin_some(node_);
    RCLCPP_INFO(node_->get_logger(), "Node spinned");
 
    // If can is centered, halt ongoing actions and return SUCCESS
    if (qr_position_ == item_position::CENTER)
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

    // Determine the appropriate speed based on QR code visibility
    float speed = (qr_position_ == item_position::NOT_IN_VIEW) ? fast_turn_speed : slow_turn_speed;

    // Handle LEFT or NOT_IN_VIEW condition
    if (qr_position_ == item_position::LEFT || qr_position_ == item_position::NOT_IN_VIEW)
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
            goal.speed = speed; // Set the speed
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
    if (qr_position_ == item_position::RIGHT)
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
            goal.speed = speed; // Set the speed
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

void FindQR::halt()
{
    RCLCPP_INFO(node_->get_logger(), "FindQR action halted.");

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
