#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace BT;
using namespace std::chrono_literals;



class WaitForPickupConfirmation : public WaitForConfirmation
{
public:
    WaitForPickupConfirmation(const std::string &name, const NodeConfiguration &config)
        : WaitForConfirmation(name, config, "/pickup_confirmation") {}
};

class WaitForDropOffConfirmation : public WaitForConfirmation
{
public:
    WaitForDropOffConfirmation(const std::string &name, const NodeConfiguration &config)
        : WaitForConfirmation(name, config, "/dropoff_confirmation") {}
};



class WaitForConfirmation : public BT::ConditionNode
{
public:
    ~ConditionNode() override = default;
    WaitForConfirmation(const std::string &name, const NodeConfiguration &config, const std::string &topic_suffix)
        : ConditionNode(name, config), confirmation_received_(false), topic_suffix_(topic_suffix)
    {
        node_ = rclcpp::Node::make_shared("wait_for_confirmation");
        namespace_ = node_->get_namespace();
        if (namespace_ == "/") namespace_ = ""; // Avoid double slashes

        std::string topic = namespace_.empty() ? topic_suffix_ : namespace_ + topic_suffix_;

        subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            topic, 10, [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(node_->get_logger(), "[%s] Confirmation received!", 
                            namespace_.c_str());
                this->confirmation_received_ = true;
            });

        executor_.add_node(node_);
    }

    static BT::PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        executor_.spin_some(); // Process messages

        return confirmation_received_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    bool confirmation_received_;
    std::string namespace_;
    std::string topic_suffix_;
};
