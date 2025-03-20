#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>

namespace BT {

class WaitForPassengerConfirmation : public StatefulActionNode
{
public:
    WaitForPassengerConfirmation(const std::string &name, const NodeConfiguration &config)
        : StatefulActionNode(name, config), confirmation_received_(false), node_initialized_(false)
    {
    }

    ~WaitForPassengerConfirmation() noexcept override
    {
        cleanup();
    }

    static PortsList providedPorts()
    {
        return {};
    }

    NodeStatus onStart() override
    {
        if (!node_initialized_)
        {
            node_ = rclcpp::Node::make_shared("wait_for_passenger_confirmation");
            namespace_ = node_->get_namespace();
            if (namespace_ == "/") namespace_ = "";

            std::string topic = namespace_.empty() ? "/passenger_confirmation" : namespace_ + "/passenger_confirmation";

            RCLCPP_INFO(node_->get_logger(), "Creating subscriber for %s topic.", topic.c_str());

            subscriber_ = node_->create_subscription<std_msgs::msg::String>(
                topic, 10, [this](const std_msgs::msg::String::SharedPtr msg)
                {
                    RCLCPP_INFO(node_->get_logger(), "[%s] Received message: %s", namespace_.c_str(), msg->data.c_str());
                    confirmation_received_ = true;
                });

            spin_thread_ = std::thread([this]() {
                rclcpp::spin(node_);
            });

            node_initialized_ = true;
        }

        confirmation_received_ = false;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (confirmation_received_)
        {
            RCLCPP_INFO(node_->get_logger(), "Passenger confirmed");
            confirmation_received_ = false;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        cleanup();
    }

private:
    void cleanup()
    {
        if (node_initialized_)
        {
            RCLCPP_INFO(node_->get_logger(), "Cleaning up node and thread");
            if (spin_thread_.joinable())
            {
                spin_thread_.join();
            }
            node_initialized_ = false;
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    std::string namespace_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    bool confirmation_received_;
    bool node_initialized_;
};

} // namespace BT
