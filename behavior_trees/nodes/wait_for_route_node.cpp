#include "behaviortree_cpp_v3/condition_node.h"
 
namespace BT
{

ConditionNode::ConditionNode(const std::string& name, const NodeConfiguration& config) :
  LeafNode::LeafNode(name, config)
{}

SimpleConditionNode::SimpleConditionNode(const std::string& name,
                                         TickFunctor tick_functor,
                                         const NodeConfiguration& config) :
  ConditionNode(name, config), tick_functor_(std::move(tick_functor))
{}

NodeStatus SimpleConditionNode::tick()
{
  return tick_functor_(*this);
}

}   // namespace BT




//--------------------------------//!!here      version difference      ----------------------------------------



#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;


namespace BT{

class WaitForRoute : public ConditionNode::ConditionNode
{
public:
    WaitForRoute(const std::string &name, const NodeConfiguration &config)
        : LeafNode::LeafNode(name, config), route_received_(false)
    {
        node_ = rclcpp::Node::make_shared("wait_for_route");
        namespace_ = node_->get_namespace(); // Auto-detect namespace
        if (namespace_ == "/") namespace_ = ""; // Avoid double slashes

        std::string topic = namespace_.empty() ? "/route" : namespace_ + "/route";

        subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            topic, 10, [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(node_->get_logger(), "[%s] Received Route: %s",
                            namespace_.c_str(), msg->data.c_str());
                this->route_received_ = true;
            });

        executor_.add_node(node_);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("pickup_pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("dropoff_pose")};
    }

    NodeStatus tick() override
    {
        executor_.spin_some(); // Process messages

        if (route_received_)
        {
            geometry_msgs::msg::PoseStamped pickup, dropoff;
            pickup.pose.position.x = 1.0;  // Dummy value
            dropoff.pose.position.x = 5.0; // Dummy value

            setOutput("pickup_pose", pickup);
            setOutput("dropoff_pose", dropoff);

            return NodeStatus::SUCCESS;
        }

        return NodeStatus::FAILURE; // change to RUNNING if you want to wait for the route?
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    bool route_received_;
    std::string namespace_;
};

}   // namespace BT