#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "luggage_av/msg/RoutePoses.hpp" // Include the header for PickupDropoffPoses

/// :TESTED:USING:      ros2 topic pub -1 /luggage_av/route std_msgs/msg/String "{data: 'Your message here'}"


const geometry_msgs::msg::PoseStamped DEFAULT_PICKUP_POSE = [] {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "luggage_av/map";
    pose.pose.position.x = 2.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}();

const geometry_msgs::msg::PoseStamped DEFAULT_DROPOFF_POSE = [] {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "luggage_av/map";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}();



namespace BT {

class WaitForRoute : public ConditionNode
{
public:
    WaitForRoute(const std::string &name, const NodeConfiguration &config)
        : ConditionNode(name, config), route_received_(false)
    {
        node_ = rclcpp::Node::make_shared("wait_for_route");
        namespace_ = node_->get_namespace();
        if (namespace_ == "/") namespace_ = "";

        std::string topic = namespace_.empty() ? "/route" : namespace_ + "/route";

        RCLCPP_INFO(node_->get_logger(), "Creating subscriber for %s topic.", topic.c_str());

        subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            topic, 10, [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(node_->get_logger(), "[%s] Received message: %s", namespace_.c_str(), msg->data.c_str());
                // For demonstration purposes, we set our poses to some filler ones
                this->pickup_pose_ = DEFAULT_PICKUP_POSE;
                this->dropoff_pose_ = DEFAULT_DROPOFF_POSE;
                this->route_received_ = true;
            });
        // subscriber_ = node_->create_subscription<luggage_av::msg::RoutePoses>(
        //     topic, 10, [this](const luggage_av::msg::RoutePoses::SharedPtr msg)
        //     {
        //         RCLCPP_INFO(node_->get_logger(), "[%s] Received Route", namespace_.c_str());
        //         this->pickup_pose_ = msg->pickup_pose;
        //         this->dropoff_pose_ = msg->dropoff_pose;
        //         this->route_received_ = true;
        //     });

        executor_.add_node(node_);
    }

    ~WaitForRoute() noexcept override = default; // Explicitly declare the destructor

    static PortsList providedPorts() { 
        return {
            OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),
            OutputPort<geometry_msgs::msg::PoseStamped>("dropoff_pose")
        };
    }

    NodeStatus tick() override
    {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] Waiting for route...", namespace_.c_str());
        executor_.spin_some();
        if (route_received_)
        {
            route_received_ = false; // Reset the flag

            setOutput("pickup_pose", pickup_pose_);
            setOutput("dropoff_pose", dropoff_pose_);

            RCLCPP_INFO(node_->get_logger(), "Route Recieved!");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    // rclcpp::Subscription<luggage_av::msg::RoutePoses>::SharedPtr subscriber_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    bool route_received_;
    geometry_msgs::msg::PoseStamped pickup_pose_;
    geometry_msgs::msg::PoseStamped dropoff_pose_;
};

} // namespace BT