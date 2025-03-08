#include "wait_for_route_bt_node.hpp"

using namespace std::chrono_literals;

WaitForRoute::WaitForRoute(
  const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node
) : BT::StatefulActionNode(name, config), node_(node) 
{
    RCLCPP_DEBUG(node_->get_logger(), "WaitForRoute constructor called");
    
    action_server_ = rclcpp_action::create_server<luggage_av_msgs::action::SendRoute>(
        node_,
        "wait_for_route",
        std::bind(&WaitForRoute::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        nullptr,  // handle_cancel
        nullptr    // handle_accepted
        );
}

BT::PortsList WaitForRoute::providedPorts() {
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("dropoff_pose")
  };
}

BT::NodeStatus WaitForRoute::onStart() {
  start_time_ = node_->now();
  goal_received_ = false;
  RCLCPP_INFO(node_->get_logger(), "Waiting for route command...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForRoute::onRunning() {
  auto elapsed = node_->now() - start_time_;

  if (goal_received_) {
    setOutput("pickup_pose", pickup_pose_);
    setOutput("dropoff_pose", dropoff_pose_);
    return BT::NodeStatus::SUCCESS;
  } else if (elapsed.seconds() >= timeout_.count()) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout: No route received after 30 minutes");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForRoute::onHalted() {
  // Cleanup if needed
}

rclcpp_action::GoalResponse WaitForRoute::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const luggage_av_msgs::action::SendRoute::Goal> goal
  ) {
    (void)uuid;
    pickup_pose_ = goal->pickup_pose;
    dropoff_pose_ = goal->dropoff_pose;
    goal_received_ = true;
    RCLCPP_INFO(node_->get_logger(), "Route received!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}