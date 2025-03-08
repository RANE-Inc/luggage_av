#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

class DropoffPassenger : public BT::SyncActionNode {
public:
    DropoffPassenger(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }  // No ports needed

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("DropoffPassenger"), "[SUCCESS] DropoffPassenger executed!");
        return BT::NodeStatus::SUCCESS;
    }
};

// Register the node with BehaviorTreeFactory
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<DropoffPassenger>("DropoffPassenger");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rclcpp::Node>("dropoff_passenger_node"));
    rclcpp::shutdown();
    return 0;
}