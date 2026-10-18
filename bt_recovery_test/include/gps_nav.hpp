#pragma once
#include <behaviortree_ros2/bt_action_node.hpp>
#include "gps_interfaces/action/navigate_to_gps.hpp"

using NavigateToGPS = gps_interfaces::action::NavigateToGPS;

class GpsNav : public BT::RosActionNode<NavigateToGPS>
{
public:
  GpsNav(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
  : RosActionNode<NavigateToGPS>(name, conf, params) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("waypoint_lat"),
      BT::InputPort<double>("waypoint_lon"),
      BT::OutputPort<std::string>("fail_reason")});
  }

  bool setGoal(Goal& goal) override
  {
    auto lat = getInput<double>("waypoint_lat");
    auto lon = getInput<double>("waypoint_lon");
    if (!lat || !lon) { return false; }          // missing port value -> node fails
    goal.latitude = lat.value();
    goal.longitude = lon.value();
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    setOutput("fail_reason", wr.result->reason);
    return wr.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> fb) override
  {
    RCLCPP_INFO_THROTTLE(logger(), *node_.lock()->get_clock(), 2000,
                         "distance remaining: %.2f m", fb->distance_remaining);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    (void)error;
    // server not found, goal rejected, timeout, etc.
    setOutput("fail_reason", std::string("action_error"));
    return BT::NodeStatus::FAILURE;
  }
};