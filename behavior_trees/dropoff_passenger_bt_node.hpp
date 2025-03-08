#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
// #include "luggage_av_msgs/srv/get_service_request.hpp"

class DropoffPassenger : public BT::SyncActionNode {
public:
    DropoffPassenger(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }  // No ports needed

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("TestNode"), "[SUCCESS] TestNode executed!");
        return BT::NodeStatus::SUCCESS;
    }
};