#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navigate_to_pose_node.cpp"
#include <thread>
#include <atomic>

namespace BT {
using PoseStamped = geometry_msgs::msg::PoseStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigateToLocation : public StatefulActionNode
{
public:
    NavigateToLocation(const std::string &name, const NodeConfiguration &config)
        : StatefulActionNode(name, config), stop_spinning_(false)
    {
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation init");
    }

    ~NavigateToLocation() noexcept override
    {
        cleanup();
    }

    static PortsList providedPorts()
    {
        return {InputPort<PoseStamped>("goal")};
    }

    NodeStatus onStart() override
    {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(rclcpp::get_logger("NavigateToLocation"), "ROS context invalid, skipping start");
            return NodeStatus::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation onStart");

        if (!getInput<PoseStamped>("goal", goal_pose_)) {
            goal_pose_ = DEFAULT_DOCKING_POSE;
            RCLCPP_WARN(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation: goal pose not provided. Using DEFAULT_DOCKING_POSE");
        }
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "Goal Pose: [x: %f, y: %f, z: %f]", 
                    goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);

        // Clean up previous node and thread
        cleanup();

        // Create new node and start spinning
        navigate_to_pose_node_ = std::make_shared<NavigateToPoseNode>(goal_pose_);
        stop_spinning_.store(false);
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation spin");
        spin_thread_ = std::thread([this]() {
            while (rclcpp::ok() && !stop_spinning_.load()) {
                rclcpp::spin_some(navigate_to_pose_node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "Spin thread stopped");
        });
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation onStart finished");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(rclcpp::get_logger("NavigateToLocation"), "ROS context invalid, failing");
            cleanup();
            return NodeStatus::FAILURE;
        }

        bool printStatus = false;
        if (navigationStatus != navigate_to_pose_node_->getNavigationStatus()) {
            navigationStatus = navigate_to_pose_node_->getNavigationStatus();
            printStatus = true;
        }

        switch (navigationStatus) {
            case NavigateToPoseNode::NavigationStatus::REQUESTED:
                if (printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation Requested", this->name().c_str());
                return NodeStatus::RUNNING;
            case NavigateToPoseNode::NavigationStatus::ACCEPTED:
                if (printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation Accepted", this->name().c_str());
                return NodeStatus::RUNNING;
            case NavigateToPoseNode::NavigationStatus::SUCCEEDED:
                if (printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Goal reached", this->name().c_str());
                cleanup(); // Stop spinning when done
                return NodeStatus::SUCCESS;
            case NavigateToPoseNode::NavigationStatus::NAVIGATING:
                if (printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigating...", this->name().c_str());
                return NodeStatus::RUNNING;
            case NavigateToPoseNode::NavigationStatus::REJECTED:
            case NavigateToPoseNode::NavigationStatus::ABORTED:
            case NavigateToPoseNode::NavigationStatus::CANCELED:
            case NavigateToPoseNode::NavigationStatus::UNKNOWN_ERROR:
                if (printStatus) RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "[%s] Navigation failed. Status: %d", this->name().c_str(), (int)navigationStatus);
                cleanup(); // Stop spinning on failure
                return NodeStatus::FAILURE;
            default:
                if (printStatus) RCLCPP_ERROR(rclcpp::get_logger("NavigateToLocation"), "[%s] Unknown navigation status", this->name().c_str());
                cleanup();
                return NodeStatus::FAILURE;
        }
    }

    void onHalted() override
    {
        RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "NavigateToLocation halted");
        cleanup();
    }

private:
    void cleanup()
    {
        if (navigate_to_pose_node_) {
            stop_spinning_.store(true);
            if (spin_thread_.joinable()) {
                RCLCPP_INFO(rclcpp::get_logger("NavigateToLocation"), "Joining spin thread");
                spin_thread_.join();
            }
            navigate_to_pose_node_.reset();
        }
    }

    std::shared_ptr<NavigateToPoseNode> navigate_to_pose_node_;
    std::thread spin_thread_;
    PoseStamped goal_pose_;
    NavigateToPoseNode::NavigationStatus navigationStatus = NavigateToPoseNode::NavigationStatus::UNINITIALIZED;
    std::atomic<bool> stop_spinning_;

    const PoseStamped DEFAULT_DOCKING_POSE = [] {
        PoseStamped pose;
        pose.header.frame_id = "luggage_av/map";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        return pose;
    }();
};

} // namespace BT