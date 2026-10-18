#pragma once
#include <behaviortree_cpp/control_node.h>

class RecoveryNode : public BT::ControlNode
{
public:
  RecoveryNode(const std::string& name, const BT::NodeConfig& config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("number_of_retries", 1, "Recovery attempts before giving up")};
  }

  void halt() override
  {
    retry_count_ = 0;
    in_recovery_ = false;
    ControlNode::halt();
  }

  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 2) {
      throw BT::LogicError("RecoveryNode needs exactly 2 children");
    }
    int max_retries = 1;
    getInput("number_of_retries", max_retries);

    if (status() == BT::NodeStatus::IDLE) { retry_count_ = 0; in_recovery_ = false; }
    setStatus(BT::NodeStatus::RUNNING);

    while (retry_count_ <= max_retries) {
      if (!in_recovery_) {
        switch (children_nodes_[0]->executeTick()) {
          case BT::NodeStatus::SUCCESS:
            return finish(BT::NodeStatus::SUCCESS);
          case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;
          case BT::NodeStatus::FAILURE:
            if (retry_count_ >= max_retries) { return finish(BT::NodeStatus::FAILURE); }
            haltChild(0);
            in_recovery_ = true;
            break;
          default:
            throw BT::LogicError("Unexpected child status");
        }
      }
      switch (children_nodes_[1]->executeTick()) {
        case BT::NodeStatus::SUCCESS:
          haltChild(1);
          in_recovery_ = false;
          ++retry_count_;              // loop again -> main child is retried
          break;
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;
        case BT::NodeStatus::FAILURE:
          return finish(BT::NodeStatus::FAILURE);
        default:
          throw BT::LogicError("Unexpected child status");
      }
    }
    return finish(BT::NodeStatus::FAILURE);
  }

private:
  BT::NodeStatus finish(BT::NodeStatus s)
  {
    haltChildren();
    retry_count_ = 0;
    in_recovery_ = false;
    return s;
  }
  int retry_count_ = 0;
  bool in_recovery_ = false;
};