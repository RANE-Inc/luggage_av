#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"


namespace BT {
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;


class NavigateToLocation : public StatefulActionNode
{
public:
    NavigateToLocation(const std::string &name, const NodeConfiguration &config)
    : StatefulActionNode(name, config), goal_sent_(false)
    {
        node_ = rclcpp::Node::make_shared("navigate_to_location_node");
        namespace_ = node_->get_namespace();
        if (namespace_ == "/") namespace_ = "";
        std::string topic = namespace_.empty() ? "/navigate_to_pose" : namespace_ + "/navigate_to_pose";

        RCLCPP_INFO(node_->get_logger(), "Action topic: %s", topic.c_str());
        RCLCPP_INFO(node_->get_logger(), "Namespace: %s", namespace_.c_str());

        action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, topic);
        
        executor_.add_node(node_);
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
            RCLCPP_ERROR(node_->get_logger(), "NavigateToLocation: failed to get goal pose");
            return NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "Goal Pose: [x: %f, y: %f, z: %f]", 
            goal_pose_.pose.position.x, 
            goal_pose_.pose.position.y, 
            goal_pose_.pose.position.z);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose_;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&BT::NavigateToLocation::nav_to_pose_result_callback, this, std::placeholders::_1);
        send_goal_options.goal_response_callback = std::bind(&BT::NavigateToLocation::nav_to_pose_goal_response_callback, this, std::placeholders::_1);

        future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(node_->get_logger(), "Sent Goal to Nav2");
        goal_sent_ = false;
        return NodeStatus::RUNNING;
    }



    NodeStatus onRunning() override
    {
        if (goal_sent_){
            RCLCPP_INFO(node_->get_logger(), "[%s] Goal reached", this->name().c_str());
            return BT::NodeStatus::SUCCESS;
        }else{
            RCLCPP_DEBUG(node_->get_logger(), "[%s] Navigating...", this->name().c_str());
            return BT::NodeStatus::RUNNING;
        }
    }


    // not handled 
    void onHalted() override{};




    void nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result){
        // If there is a result, we consider navigation completed.
        // bt_navigator only sends an empty message without status. Idk why though.

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
                goal_sent_ = true;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                setStatus(NodeStatus::FAILURE);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                setStatus(NodeStatus::FAILURE);
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                break;
        }
    }

    void nav_to_pose_goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle){
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server");
            setStatus(NodeStatus::FAILURE);
        } else {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by the server, waiting for result");
        }
    }


private:
    rclcpp::Node::SharedPtr node_;      //!!Here rename this
    std::string namespace_;
    geometry_msgs::msg::PoseStamped ppose_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    bool goal_sent_;    
    geometry_msgs::msg::PoseStamped goal_pose_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_future<GoalHandleNav::SharedPtr> future_goal_handle_;


};

}   // BT namespace