#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "luggage_av_msgs/action/send_route.hpp"

class SendRouteActionServer : public rclcpp::Node
{
public:
    using SendRoute = luggage_av_msgs::action::SendRoute;
    using GoalHandleSendRoute = rclcpp_action::ServerGoalHandle<SendRoute>;

    explicit SendRouteActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("send_route_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<SendRoute>(
            this,
            "send_route",
            std::bind(&SendRouteActionServer::handle_goal, this, _1, _2),
            std::bind(&SendRouteActionServer::handle_cancel, this, _1),
            std::bind(&SendRouteActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<SendRoute>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SendRoute::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleSendRoute> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleSendRoute> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&SendRouteActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleSendRoute> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<SendRoute::Result>();

        // Simulate some work
        loop_rate.sleep();

        // Check if goal is done
        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendRouteActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}