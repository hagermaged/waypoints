#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "gps_nav.hpp"
#include "gps_recovery.hpp"
#include "recovery_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  BT::NodeStatus status = BT::NodeStatus::FAILURE;

  {
    // Scoped block: ensures node, factory, and tree are destroyed 
    // BEFORE rclcpp::shutdown() is called.
    auto node = std::make_shared<rclcpp::Node>("gps_bt_test");

    BT::BehaviorTreeFactory factory;

    BT::RosNodeParams nav_params;
    nav_params.nh = node;
    nav_params.default_port_value = "navigate_to_gps";
    factory.registerNodeType<GpsNav>("GpsNav", nav_params);

    BT::RosNodeParams rec_params;
    rec_params.nh = node;
    rec_params.default_port_value = "recover";
    factory.registerNodeType<GpsRecovery>("GpsRecovery", rec_params);

    factory.registerNodeType<RecoveryNode>("RecoveryNode");

    auto dir = ament_index_cpp::get_package_share_directory("bt_recovery_test");
    factory.registerBehaviorTreeFromFile(dir + "/trees/gps_test.xml");

    auto tree = factory.createTree("TestTree");
    status = tree.tickWhileRunning(std::chrono::milliseconds(50));

    RCLCPP_INFO(node->get_logger(), "Tree finished: %s", BT::toStr(status).c_str());
  } // <--- Tree and node are safely destroyed here!

  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}