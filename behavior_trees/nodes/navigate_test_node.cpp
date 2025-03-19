#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/*
ros2 action send_goal /luggage_av/navigate_to_pose nav2_msgs/action/NavigateToPose 
"{
    pose: {
        header: {frame_id: 'luggage_av/map'}, 
        pose:   {
            position: {x: 2.0, y: 0.0, z: 0.0}, 
            orientation: {z: 0.0, w: 1.0}
        }
    }
}"
*/

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

class NavigateToLocationNode : public rclcpp::Node
{
public:
    NavigateToLocationNode()
        : Node("navigate_to_location_node"), goal_sent_(false)
    {


        namespace_ = this->get_namespace();     // I have debugged this to death. the namespace is exactly what you think
        if (namespace_ == "/") namespace_ = "";
        std::string topic = namespace_.empty() ? "/navigate_to_pose" : namespace_ + "/navigate_to_pose";

        RCLCPP_INFO(this->get_logger(), "Action topic: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Namespace: %s", namespace_.c_str());

        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, topic);

        sendGoal(DEFAULT_PICKUP_POSE);
    }

private:
    void sendGoal(const geometry_msgs::msg::PoseStamped pose)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal pose: [x: %f, y: %f, z: %f]",
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavigateToLocationNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&NavigateToLocationNode::resultCallback, this, std::placeholders::_1);

        future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Sent Goal to Nav2");
        goal_sent_ = false;
    }



    void goalResponseCallback(std::shared_ptr<GoalHandleNav> goal_handle){
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
        }
    }

    void resultCallback(const GoalHandleNav::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            goal_sent_ = true;
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
    }


    std::string namespace_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    bool goal_sent_;
    std::shared_future<GoalHandleNav::SharedPtr> future_goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToLocationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}