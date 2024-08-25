#ifndef CANCLOSE_HPP
#define CANCLOSE_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

class CanClose : public BT::ConditionNode
{
public:
    CanClose(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    double can_distance_;
    double max_distance;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};

#endif // CANCLOSE_HPP
