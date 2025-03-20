#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navigate_to_pose_node.cpp"
#include <thread>


namespace BT {
using PoseStamped = geometry_msgs::msg::PoseStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;


class NavigateToLocation : public StatefulActionNode
{
public:
    NavigateToLocation(const std::string &name, const NodeConfiguration &config)
    : StatefulActionNode(name, config)
    {
    }
    ~NavigateToLocation() noexcept override = default; // Explicitly declare the destructor


    static PortsList providedPorts()
    {
        return {InputPort<PoseStamped>("goal")};
    }


        // Method overrides
    NodeStatus onStart() override
    {
        if (!getInput<PoseStamped>("goal", goal_pose_))
        {
            goal_pose_ = DEFAULT_DOCKING_POSE;
            RCLCPP_WARN(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation: goal pose not provided. Using DEFAULT_DOCKING_POSE");
        }
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "Goal Pose: [x: %f, y: %f, z: %f]", goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);

        navigate_to_pose_node_ = std::make_shared<NavigateToPoseNode>(goal_pose_);
        spin_thread_ = std::thread([this]() {
            rclcpp::spin(navigate_to_pose_node_);
        });
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "Sent Goal to Nav2");
        return NodeStatus::RUNNING;
    }



    NodeStatus onRunning() override
    {
        bool printStatus = false;
        if(navigationStatus != navigate_to_pose_node_.get()->getNavigationStatus()){
            navigationStatus = navigate_to_pose_node_.get()->getNavigationStatus();
            printStatus = true;
        }

        switch (navigationStatus)
        {
            case NavigateToPoseNode::NavigationStatus::REQUESTED:
                if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation Requested", this->name().c_str());
                return BT::NodeStatus::RUNNING;
            case NavigateToPoseNode::NavigationStatus::ACCEPTED:
                if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation Accepted", this->name().c_str());
                return BT::NodeStatus::RUNNING;
            case NavigateToPoseNode::NavigationStatus::SUCCEEDED:
                if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Goal reached", this->name().c_str());
                return BT::NodeStatus::SUCCESS;
            case NavigateToPoseNode::NavigationStatus::NAVIGATING:
                if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigating...", this->name().c_str());
                return BT::NodeStatus::RUNNING;
            case NavigateToPoseNode::NavigationStatus::REJECTED:
                // these ones fall through to UNKNOWN_ERROR which handles all failures
            case NavigateToPoseNode::NavigationStatus::ABORTED:
            case NavigateToPoseNode::NavigationStatus::CANCELED:
            case NavigateToPoseNode::NavigationStatus::UNKNOWN_ERROR:
                if(printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation failed. Status: %d", this->name().c_str(), (int)navigationStatus);
                return BT::NodeStatus::FAILURE;
            default:
                if(printStatus) RCLCPP_ERROR(rclcpp::get_logger("NavigateToLocation"), "[%s] Unknown navigation status", this->name().c_str());
                return BT::NodeStatus::FAILURE;
        }
    }


    // not handled 
    void onHalted() override{};


private:
    std::shared_ptr<NavigateToPoseNode> navigate_to_pose_node_;
    std::thread spin_thread_;
    PoseStamped goal_pose_;
    NavigateToPoseNode::NavigationStatus navigationStatus = NavigateToPoseNode::NavigationStatus::UNINITIALIZED;



    const PoseStamped DEFAULT_DOCKING_POSE = [] {
        PoseStamped pose;
        pose.header.frame_id = "luggage_av/map";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        return pose;
    }();
};

}   // BT namespace