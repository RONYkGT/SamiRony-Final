#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "behavior_tree/FindCan.hpp"
#include "behavior_tree/DropCan.hpp"
#include "behavior_tree/FindQR.hpp"
#include "behavior_tree/MoveForward.hpp"
#include "behavior_tree/PickCan.hpp"
#include "behavior_tree/CanClose.hpp"
#include "behavior_tree/CanInView.hpp"
#include "behavior_tree/CanPicked.hpp"
#include "behavior_tree/QRinView.hpp"
#include "behavior_tree/OnBlackStrip.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_tree_node");

    // Create the BehaviorTree factory
    BT::BehaviorTreeFactory factory;

    // Register custom nodes with the factory
    factory.registerNodeType<FindCan>("FindCan");
    factory.registerNodeType<DropCan>("DropCan");
    factory.registerNodeType<FindQR>("FindQR");
    factory.registerNodeType<MoveForward>("MoveForward");
    factory.registerNodeType<PickCan>("PickCan");
    factory.registerNodeType<CanClose>("CanClose");
    factory.registerNodeType<CanInView>("CanInView");
    factory.registerNodeType<CanPicked>("CanPicked");
    factory.registerNodeType<QRinView>("QRinView");
    factory.registerNodeType<OnBlackStrip>("OnBlackStrip");

    // Load the XML file that defines the behavior tree
    std::string pkgpath = ament_index_cpp::get_package_prefix("behavior_tree");
    std::string xml_file = pkgpath + "/behavior_trees_xml/behavior_tree.xml";
    std::cout << "Package path: " << pkgpath << std::endl;
    std::cout << "XML file path: " << xml_file << std::endl;

    // Build the tree from the XML file
    auto tree = factory.createTreeFromFile(xml_file);
    
    // Creating logger that saves state changes
    BT::FileLogger logger_file(tree, "/tmp/bt_trace.fbl");
    // Creating logger that stores execution time
    BT::MinitraceLogger logger_minitrace(tree, "/tmp/bt_trace.json");

    std::cout << "test1";
    // Run the tree
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    std::cout << "test2";
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    return 0;
}
