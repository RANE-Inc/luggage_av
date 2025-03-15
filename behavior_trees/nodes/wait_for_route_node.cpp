#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "luggage_av/msg/RoutePoses.hpp" // Include the header for PickupDropoffPoses

using namespace std::chrono_literals;

namespace BT {

class WaitForRoute : public ConditionNode
{
public:
    WaitForRoute(const std::string &name, const NodeConfiguration &config)
        : ConditionNode(name, config), route_received_(false)
    {
        node_ = rclcpp::Node::make_shared("wait_for_route");
        namespace_ = node_->get_namespace(); // Auto-detect namespace
        if (namespace_ == "/") namespace_ = ""; // Avoid double slashes

        std::string topic = namespace_.empty() ? "/route" : namespace_ + "/route";

        subscriber_ = node_->create_subscription<luggage_av::msg::PickupDropoffPoses>(
            topic, 10, [this](const luggage_av::msg::RoutePoses::SharedPtr msg)
            {
                RCLCPP_INFO(node_->get_logger(), "[%s] Received Route", namespace_.c_str());
                this->pickup_pose_ = msg->pickup_pose;
                this->dropoff_pose_ = msg->dropoff_pose;
                this->route_received_ = true;
            });

        executor_.add_node(node_);
    }

    ~WaitForRoute() noexcept override = default; // Explicitly declare the destructor

    static PortsList providedPorts() { 
        return {OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),
                OutputPort<geometry_msgs::msg::PoseStamped>("dropoff_pose")};
    }

    NodeStatus tick() override
    {
        executor_.spin_some();
        if (route_received_)
        {
            route_received_ = false; // Reset the flag

            setOutput("pickup_pose", pickup_pose_);
            setOutput("dropoff_pose", dropoff_pose_);

            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    rclcpp::Subscription<luggage_av::msg::PickupDropoffPoses>::SharedPtr subscriber_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    bool route_received_;
    geometry_msgs::msg::PoseStamped pickup_pose_;
    geometry_msgs::msg::PoseStamped dropoff_pose_;
};

} // namespace BT