#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

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

class SimpleNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    SimpleNode()
        : Node("simple_node")
    {
        namespace_ = this->get_namespace();     // I have debugged this to death. the namespace is exactly what you think
        if (namespace_ == "/") namespace_ = "";
        std::string action_name = namespace_.empty() ? "/navigate_to_pose" : namespace_ + "/navigate_to_pose";
        std::string frame_id = namespace_.empty() ? "map" : namespace_.substr(1) + "/map";

        RCLCPP_INFO(this->get_logger(), "Action topic: %s", action_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Namespace: %s", namespace_.c_str());

        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, action_name);

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = DEFAULT_PICKUP_POSE;

        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&SimpleNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&SimpleNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&SimpleNode::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    std::string namespace_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was successful");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}