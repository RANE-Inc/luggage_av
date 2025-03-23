#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <atomic>

class NavigateToPoseNode : public rclcpp::Node
{
public:
    enum class NavigationStatus
    {
        UNINITIALIZED,
        REQUESTED,
        ACCEPTED,
        REJECTED,
        NAVIGATING,
        SUCCEEDED,
        ABORTED,
        CANCELED,
        UNKNOWN_ERROR,
    };

    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using NavigationStatusCallback = std::function<void(NavigationStatus)>;

    NavigateToPoseNode(PoseStamped goal_pose) : Node("navigate_to_pose_node")
    {
        namespace_ = this->get_namespace();
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
        goal_msg.pose = goal_pose;

        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigateToPoseNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&NavigateToPoseNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&NavigateToPoseNode::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        navigation_status_.store(NavigationStatus::REQUESTED);
    }

    NavigationStatus getNavigationStatus() {
        return navigation_status_.load();
    }


    
private:
    std::string namespace_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    std::atomic<NavigationStatus> navigation_status_{NavigationStatus::UNINITIALIZED};
    

    void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            navigation_status_.store(NavigationStatus::REJECTED);
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            navigation_status_.store(NavigationStatus::ACCEPTED);
        }
    }

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_DEBUG(this->get_logger(), "Current position: (%.2f, %.2f)", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
        navigation_status_.store(NavigationStatus::NAVIGATING);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was successful");
                navigation_status_.store(NavigationStatus::SUCCEEDED);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                navigation_status_.store(NavigationStatus::ABORTED);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                navigation_status_.store(NavigationStatus::CANCELED);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                navigation_status_.store(NavigationStatus::UNKNOWN_ERROR);
                break;
        }
    }
};