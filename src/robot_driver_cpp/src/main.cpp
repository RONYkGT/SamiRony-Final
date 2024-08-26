#include "rclcpp/rclcpp.hpp"
#include "robot_driver_cpp/turn_left_action_server.hpp"
#include "robot_driver_cpp/turn_right_action_server.hpp"
#include "robot_driver_cpp/move_forward_action_server.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto turn_left_server = std::make_shared<TurnLeftActionServer>();
    auto turn_right_server = std::make_shared<TurnRightActionServer>();
    auto move_forward_server = std::make_shared<MoveForwardActionServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(turn_left_server);
    executor.add_node(turn_right_server);
    executor.add_node(move_forward_server);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
