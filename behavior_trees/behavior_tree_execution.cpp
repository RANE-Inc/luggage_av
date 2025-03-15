#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/wait_for_route_node.cpp"

using namespace BT;

class BehaviorTreeNode : public rclcpp::Node
{
public:
    BehaviorTreeNode() : Node("behavior_tree_node")
    {
        BehaviorTreeFactory factory;

        auto tree_simplified = factory.createTreeFromText("bt_simple.xml");

        factory.registerNodeType<WaitForRoute>("WaitForRoute");
        // factory.registerSimpleCondition("WaitForRoute", std::bind(&WaitForRoute));
        // factory.registerSimpleAction("NavigateToLocation", std::bind(&NavigateToLocationFunction, std::placeholders::_1));
        // factory.registerSimpleAction("PauseRoute", std::bind(&PauseRouteFunction));

        while (rclcpp::ok() && tree_simplified.tickRoot() == NodeStatus::RUNNING)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviorTreeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
