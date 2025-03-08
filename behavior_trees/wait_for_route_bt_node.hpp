#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "luggage_av_msgs/action/send_route.hpp"

class WaitForRoute : public BT::StatefulActionNode {
public:
  WaitForRoute(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  std::chrono::seconds timeout_{1800}; // 30 minutes
  bool goal_received_{false};
  geometry_msgs::msg::PoseStamped pickup_pose_;
  geometry_msgs::msg::PoseStamped dropoff_pose_;

  // Action server for SendRoute
  rclcpp_action::Server<luggage_av_msgs::action::SendRoute>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const luggage_av_msgs::action::SendRoute::Goal> goal
  );
};