#pragma once
#include <behaviortree_ros2/bt_action_node.hpp>
#include "gps_interfaces/action/recovery_gps.hpp"

using RecoveryGPS = gps_interfaces::action::RecoveryGPS;

class GpsRecovery : public BT::RosActionNode<RecoveryGPS>
{
public:
  GpsRecovery(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
  : RosActionNode<RecoveryGPS>(name, conf, params) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("target_lat"),
      BT::InputPort<double>("target_lon"),
      BT::InputPort<std::string>("reason")});
  }

  bool setGoal(Goal& goal) override
  {
    auto lat = getInput<double>("target_lat");
    auto lon = getInput<double>("target_lon");
    auto reason = getInput<std::string>("reason");
    if (!lat || !lon || !reason) { return false; }
    goal.target_lat = lat.value();
    goal.target_lon = lon.value();
    goal.reason = reason.value();
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    return wr.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback>) override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override
  {
    return BT::NodeStatus::FAILURE;
  }
};