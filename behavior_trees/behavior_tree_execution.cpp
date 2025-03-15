#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/wait_for_route_node.cpp"

using namespace BT;

class BehaviorTreeNode : public rclcpp::Node
{
public:
    BehaviorTreeNode() : Node("behavior_tree_node")
    {
        // Get the XML file path from the launch arguments
        std::string xml_file_path;

        this->declare_parameter<std::string>("bt_xml_file", "");
        this->get_parameter("bt_xml_file", xml_file_path);

        RCLCPP_INFO(this->get_logger(), "Behavior tree XML file path: %s", xml_file_path.c_str());

        if (xml_file_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Behavior tree XML file path is empty");
            throw std::runtime_error("Behavior tree XML file path is empty");
        }


        // Set up the Behavior Tree
        BehaviorTreeFactory factory;

        auto tree_simplified = factory.createTreeFromFile(xml_file_path);

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
