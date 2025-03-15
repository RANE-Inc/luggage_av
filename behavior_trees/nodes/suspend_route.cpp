#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace BT;
using namespace std::chrono_literals;

class PauseRoute : public BT::SyncActionNode
{
public:
    PauseRoute(const std::string &name, const NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("pause_route");
        namespace_ = node_->get_namespace();
        if (namespace_ == "/") namespace_ = ""; // Avoid double slashes
    }

    NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] Pausing route...", namespace_.c_str());
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
};

class ResumeRoute : public BT::SyncActionNode
{
public:
    ResumeRoute(const std::string &name, const NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("resume_route");
        namespace_ = node_->get_namespace();
        if (namespace_ == "/") namespace_ = ""; // Avoid double slashes
    }

    NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] Resuming route...", namespace_.c_str());
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
};
