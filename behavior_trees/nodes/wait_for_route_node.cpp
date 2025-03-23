#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "luggage_av_msgs/action/send_route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>
#include <atomic>



namespace BT {

class WaitForRoute : public StatefulActionNode
{
public:
    using SendRoute = luggage_av_msgs::action::SendRoute;
    using GoalHandleSendRoute = rclcpp_action::ServerGoalHandle<SendRoute>;

    WaitForRoute(const std::string& name, const NodeConfiguration& config)
        : StatefulActionNode(name, config), route_received_(false)
    {
        // Initialize the ROS node
        node_ = rclcpp::Node::make_shared("wait_for_route");
        namespace_ = node_->get_namespace();
        if (namespace_ == "/") namespace_ = "";

        // Set up the action server
        std::string action_name = namespace_.empty() ? "/send_route" : namespace_ + "/send_route";
        RCLCPP_INFO(node_->get_logger(), "Creating action server for %s action.", action_name.c_str());

        action_server_ = rclcpp_action::create_server<SendRoute>(
            node_,
            action_name,
            std::bind(&WaitForRoute::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&WaitForRoute::handle_cancel, this, std::placeholders::_1),
            std::bind(&WaitForRoute::handle_accepted, this, std::placeholders::_1));

        // Start spinning the node in a separate thread
        spinning_ = true;
        spin_thread_ = std::thread([this]() {
            while (spinning_ && rclcpp::ok()) {
                rclcpp::spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    ~WaitForRoute() noexcept override
    {
        // Stop spinning and clean up the thread

        if (node_) {
            node_->get_node_base_interface()->get_context()->shutdown("NavigateToLocation node halted"); // Signal spin to stop for this specific node
            if (spin_thread_.joinable()) {
                spinning_ = false;
                spin_thread_.join();
            }
        }
    }

    static PortsList providedPorts()
    {
        return {
            OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),
            OutputPort<geometry_msgs::msg::PoseStamped>("dropoff_pose")
        };
    }

    NodeStatus onStart() override
    {
        route_received_ = false;
        RCLCPP_INFO(node_->get_logger(), "Waiting for route...");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (route_received_)
        {
            route_received_ = false; // Reset for potential reuse
            setOutput("pickup_pose", pickup_pose_);
            setOutput("dropoff_pose", dropoff_pose_);
            RCLCPP_INFO(node_->get_logger(), "Route received and processed!");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_INFO(node_->get_logger(), "WaitForRoute halted.");
        // No need to stop spinning here; destructor will handle cleanup
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /* uuid */,
        std::shared_ptr<const SendRoute::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request");
        pickup_pose_ = goal->pickup_pose;
        dropoff_pose_ = goal->dropoff_pose;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleSendRoute> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleSendRoute> goal_handle)
    {
        std::thread{std::bind(&WaitForRoute::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleSendRoute> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        auto result = std::make_shared<SendRoute::Result>();

        // Simulate some work
        loop_rate.sleep();

        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            route_received_ = true;
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    rclcpp_action::Server<SendRoute>::SharedPtr action_server_;
    bool route_received_;
    geometry_msgs::msg::PoseStamped pickup_pose_;
    geometry_msgs::msg::PoseStamped dropoff_pose_;
    std::atomic<bool> spinning_{false};
    std::thread spin_thread_;
};

} // namespace BT