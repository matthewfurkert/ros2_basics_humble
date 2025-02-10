#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"
#include "example_interfaces/msg/bool.hpp"

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;
using namespace std::placeholders;

class MoveRobotClientNode : public rclcpp::Node {
public:
    MoveRobotClientNode() : Node("move_robot_client") {
        move_robot_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");
        move_robot_subscriber_ = this->create_subscription<example_interfaces::msg::Bool>("cancel_move", 10,
            std::bind(&MoveRobotClientNode::cancel_callback, this, _1));
    }

    void send_goal(int position, int velocity) {
        move_robot_client_->wait_for_action_server();

        auto goal = MoveRobot::Goal();
        goal.position = position;
        goal.velocity = velocity;

        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MoveRobotClientNode::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&MoveRobotClientNode::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&MoveRobotClientNode::goal_result_callback, this, _1);

        RCLCPP_INFO(this->get_logger(), "Sending a goal...");
        move_robot_client_->async_send_goal(goal, options);
    }

private:
    void cancel_callback(const example_interfaces::msg::Bool::SharedPtr msg) {
        if (msg->data && goal_handle_) {
            RCLCPP_INFO(this->get_logger(), "Cancel the goal");
            move_robot_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
    }   
    void goal_response_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Goal got rejected");
        }
        else {
            this-> goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    void feedback_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const MoveRobot::Feedback> feedback) {
        (void)goal_handle;
        int position = feedback->current_position;
        RCLCPP_INFO(this->get_logger(), "Robot is at: %d", position);
    }

    void goal_result_callback(const MoveRobotGoalHandle::WrappedResult &result) {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Cancelled");
        }

        int robot_position = result.result->position;
        RCLCPP_INFO(this->get_logger(), "Final Position: %d", robot_position);
    }

    rclcpp_action::Client<MoveRobot>::SharedPtr move_robot_client_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr move_robot_subscriber_;
    MoveRobotGoalHandle::SharedPtr goal_handle_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotClientNode>();
    node->send_goal(77, 7);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}