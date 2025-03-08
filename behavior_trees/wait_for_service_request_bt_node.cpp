#include "wait_for_service_request_bt_node.hpp"
#include <chrono> 
using namespace std::chrono_literals; 

WaitForServiceRequest::WaitForServiceRequest(
    const std::string& name, 
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node
) : BT::SyncActionNode(name, config), 
    node_(node),
    client_(node_->create_client<luggage_av_msgs::srv::GetServiceRequest>("get_service_request")) 
{
    // Constructor body can be empty here
}

BT::PortsList WaitForServiceRequest::providedPorts() {
    return {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("dropoff_pose")
    };
}

BT::NodeStatus WaitForServiceRequest::tick() {
    auto request = std::make_shared<luggage_av_msgs::srv::GetServiceRequest::Request>();
    
    if (!client_->wait_for_service(1s)) {
        RCLCPP_WARN(node_->get_logger(), "Service not available");
        return BT::NodeStatus::FAILURE;
    }

    auto result = client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        return BT::NodeStatus::FAILURE;
    }

    auto response = result.get();
    setOutput("pickup_pose", response->pickup_pose);
    setOutput("dropoff_pose", response->dropoff_pose);
    
    return BT::NodeStatus::SUCCESS;
}