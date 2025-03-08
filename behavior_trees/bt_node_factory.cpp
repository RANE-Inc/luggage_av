// #include "wait_for_service_request_bt_node.hpp"
#include "wait_for_route_bt_node.hpp"
#include "dropoff_passenger_bt_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/executors.hpp"

void RegisterNodes(BT::BehaviorTreeFactory& factory, rclcpp::Node::SharedPtr node) {
    // Create a non-static executor
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    
    // Spin the executor in a dedicated thread
    std::thread spin_thread([executor]() {
        executor->spin();
    });
    spin_thread.detach();

    // Register node
    factory.registerBuilder<WaitForRoute>("WaitForRoute",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<WaitForRoute>(name, config, node);
        });
    factory.registerBuilder<DropoffPassenger>("Dropoff_Passenger",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<DropoffPassenger>(name, config, node);
        });
    factory.registerNodeType<DropoffPassenger>("DropoffPassenger");
}




// void RegisterNodes(BT::BehaviorTreeFactory& factory, rclcpp::Node::SharedPtr node) {
//     // factory.registerBuilder<WaitForServiceRequest>("WaitForServiceRequest",
//     //     [node](const std::string& name, const BT::NodeConfiguration& config) {
//     //         return std::make_unique<WaitForServiceRequest>(name, config, node);
//     //     });
//     static rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread spin_thread([&executor]() { executor.spin(); });
//     spin_thread.detach();

//     factory.registerBuilder<WaitForRoute>("WaitForRoute",
//         [node](const std::string& name, const BT::NodeConfiguration& config) {
//         return std::make_unique<WaitForRoute>(name, config, node);
//         });
// }

// #include "wait_for_route_bt_node.hpp"
// #include "behaviortree_cpp_v3/bt_factory.h"