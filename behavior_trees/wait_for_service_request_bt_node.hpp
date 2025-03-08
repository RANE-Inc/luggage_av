#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "luggage_av_msgs/srv/get_service_request.hpp"

class WaitForServiceRequest : public BT::SyncActionNode {
public:
    WaitForServiceRequest(const std::string& name, 
                        const BT::NodeConfiguration& config,
                        rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<luggage_av_msgs::srv::GetServiceRequest>::SharedPtr client_;  // Add declaration
};