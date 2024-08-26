#include "rclcpp/rclcpp.hpp"
#include "robot_driver_cpp/turn_left_action_server.hpp"
#include "robot_driver_cpp/turn_right_action_server.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Create nodes for TurnLeft and TurnRight action servers
    auto turn_left_action_server = std::make_shared<TurnLeftActionServer>();
    auto turn_right_action_server = std::make_shared<TurnRightActionServer>();

    // Spin both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(turn_left_action_server);
    executor.add_node(turn_right_action_server);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}
