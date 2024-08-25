#ifndef MOVEFORWARD_HPP
#define MOVEFORWARD_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <chrono>
#include <thread>

class MoveForward : public BT::ActionNodeBase
{
public:
    MoveForward(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;
    void halt() override;
    
    static BT::PortsList providedPorts() {
        return {};
    }

private:
    std::chrono::steady_clock::time_point start_time_;
    bool is_running_;
    rclcpp::Node::SharedPtr node_;

};

#endif // MOVEFORWARD_HPP
