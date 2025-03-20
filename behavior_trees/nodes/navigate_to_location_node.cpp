#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_node.cpp"


namespace BT {
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;


class NavigateToLocation : public StatefulActionNode
{
public:
    NavigateToLocation(const std::string &name, const NodeConfiguration &config)
    : StatefulActionNode(name, config), goal_sent_(false)
    {
    }
    ~NavigateToLocation() noexcept override = default; // Explicitly declare the destructor


    static PortsList providedPorts()
    {
        return {InputPort<geometry_msgs::msg::PoseStamped>("goal")};
    }


        // Method overrides
    NodeStatus onStart() override
    {
        if (!getInput<geometry_msgs::msg::PoseStamped>("goal", goal_pose_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation: failed to get goal pose");
            return NodeStatus::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "Goal Pose: [x: %f, y: %f, z: %f]", goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);

        simple_node_ = std::make_shared<SimpleNode>(goal_pose_);
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "Sent Goal to Nav2");
        goal_sent_ = false;
        return NodeStatus::RUNNING;
    }



    NodeStatus onRunning() override
    {
        bool printStatus = false;
        if(navigationStatus != simple_node_.get()->getNavigationStatus()){
            navigationStatus = simple_node_.get()->getNavigationStatus();
            printStatus = true;
        }




        switch (navigationStatus)
        {
            case SimpleNode::NavigationStatus::UNINITIALIZED:
                if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "UNINITIALIZED");
                break;
            case SimpleNode::NavigationStatus::REQUESTED:
            if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "REQUESTED");
            break;

            case SimpleNode::NavigationStatus::ACCEPTED:
            case SimpleNode::NavigationStatus::REJECTED:
            if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "ACCEPTED");
            break;

            case SimpleNode::NavigationStatus::NAVIGATING:
            if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "NAVIGATING");
            break;

            case SimpleNode::NavigationStatus::SUCCEEDED:
            case SimpleNode::NavigationStatus::ABORTED:
            case SimpleNode::NavigationStatus::CANCELED:
            case SimpleNode::NavigationStatus::UNKNOWN_ERROR:
            if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "SUCCEEDED");
            break;

        }
        return BT::NodeStatus::RUNNING;

        // switch (navigationStatus)
        // {
        //     case SimpleNode::NavigationStatus::REQUESTED:
        //         if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation Requested", this->name().c_str());
        //         return BT::NodeStatus::RUNNING;
        //     case SimpleNode::NavigationStatus::ACCEPTED:
        //         if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation Accepted", this->name().c_str());
        //         return BT::NodeStatus::RUNNING;
        //     case SimpleNode::NavigationStatus::SUCCEEDED:
        //         if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Goal reached", this->name().c_str());
        //         return BT::NodeStatus::SUCCESS;
        //     case SimpleNode::NavigationStatus::NAVIGATING:
        //         if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigating...", this->name().c_str());
        //         return BT::NodeStatus::RUNNING;
        //     case SimpleNode::NavigationStatus::REJECTED:
        //         // these ones fall through to UNKNOWN_ERROR which handles all failures
        //     case SimpleNode::NavigationStatus::ABORTED:
        //     case SimpleNode::NavigationStatus::CANCELED:
        //     case SimpleNode::NavigationStatus::UNKNOWN_ERROR:
        //         if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation failed. Status: %d", this->name().c_str(), (int)navigationStatus);
        //         return BT::NodeStatus::FAILURE;
        //     default:
        //         if(printStatus) RCLCPP_ERROR(rclcpp::get_logger("NavigateToLocation"), "[%s] Unknown navigation status", this->name().c_str());
        //         return BT::NodeStatus::FAILURE;
        // }
    }


    // not handled 
    void onHalted() override{};


private:
    std::shared_ptr<SimpleNode> simple_node_;
    bool goal_sent_;    
    geometry_msgs::msg::PoseStamped goal_pose_;
    SimpleNode::NavigationStatus navigationStatus = SimpleNode::NavigationStatus::UNINITIALIZED;
};

}   // BT namespace