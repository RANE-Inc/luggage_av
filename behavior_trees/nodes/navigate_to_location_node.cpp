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
        //!!here add namespace
        action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
        
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

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose_;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&BT::NavigateToLocation::nav_to_pose_result_callback, this, std::placeholders::_1);
        send_goal_options.goal_response_callback = std::bind(&BT::NavigateToLocation::nav_to_pose__goal_response_callback, this, std::placeholders::_1);

        future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(node_->get_logger(), "Sent Goal to Nav2\n");
        goal_sent_ = true;
        return NodeStatus::RUNNING;
    }



    NodeStatus onRunning() override
    {
        if (goal_sent_){
            RCLCPP_INFO(node_->get_logger(), "[%s] Goal reached\n", this->name().c_str());
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::RUNNING;
        }
    }


    // not handled 
    void onHalted() override{};




    void nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result){
        // If there is a result, we consider navigation completed.
        // bt_navigator only sends an empty message without status. Idk why though.

        if (result.result) goal_sent_ = true;
    }

    void nav_to_pose__goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle){
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "NavigateToLocation: goal was rejected by server");
            setStatus(NodeStatus::FAILURE);
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "NavigateToLocation: goal accepted by server");
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