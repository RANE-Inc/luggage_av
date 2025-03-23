#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/wait_for_route_node.cpp"
#include "nodes/navigate_to_location_bt_node.cpp"
#include "nodes/passenger_confirmation_condition_node.cpp"
// #include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp" 

using namespace BT;

class BehaviorTreeNode : public rclcpp::Node
{
public:
    BehaviorTreeNode() : Node("behavior_tree_node")
    {
        /*
            making this a singleton did not prevent 3 instances from being creates
        */

        // Get the XML file path from the launch arguments
        std::string bt_xml_file_path;
        this->declare_parameter<std::string>("bt_xml_file", "");
        this->get_parameter("bt_xml_file", bt_xml_file_path);

        std::string nav2_tree_nodes_xml_file_path;
        this->declare_parameter<std::string>("nav2_tree_nodes_xml_file", "");
        this->get_parameter("nav2_tree_nodes_xml_file", nav2_tree_nodes_xml_file_path);

        RCLCPP_INFO(this->get_logger(), "Behavior tree XML file path: %s", bt_xml_file_path.c_str());

        if (bt_xml_file_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Behavior tree XML file path is empty");
            throw std::runtime_error("Behavior tree XML file path is empty");
        }


        // Set up the Behavior Tree
        BehaviorTreeFactory factory;

        factory.registerNodeType<WaitForRoute>("WaitForRoute");
        factory.registerNodeType<WaitForPassengerConfirmation>("WaitForPassengerConfirmation");
        // factory.registerBehaviorTreeFromFile(nav2_tree_nodes_xml_file_path);
        factory.registerNodeType<NavigateToLocation>("NavigateToLocation");
        // Register NavigateToPose (assuming Nav2 provides this class)
        // factory.registerNodeType<nav2_behavior_tree::NavigateToPoseAction>("NavigateToPose");
        
        auto tree_simplified = factory.createTreeFromFile(bt_xml_file_path);   //!!here    this creates 3 sinstances of each
        NodeStatus status;

        RCLCPP_INFO(this->get_logger(), "Starting Tree tick loop");
        while (rclcpp::ok() && (status = tree_simplified.tickRoot()) == NodeStatus::RUNNING)
        {
            RCLCPP_DEBUG(this->get_logger(), "Tree tick state: %d", (int)status);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(this->get_logger(), "Tree exited with tick state %d", (int)status);
    }
};

int main(int argc, char **argv)
{
    for (int i = 0; i < argc; ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "argv %s", argv[i]);
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviorTreeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
