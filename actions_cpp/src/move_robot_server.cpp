#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using namespace std::placeholders;

class MoveRobotServerNode : public rclcpp::Node {
public:
    MoveRobotServerNode() : Node("move_robot_server") {
        robot_position_ = 50;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot",
            std::bind(&MoveRobotServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveRobotServerNode::cancel_callback, this, _1),
            std::bind(&MoveRobotServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
    RCLCPP_INFO(this->get_logger(), "Action server has been started");
    }

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveRobot::Goal> goal) {
        
        (void)uuid;
        (void)goal;

        // Policy: Abort current goal if new goal is received
        { std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_) {
                if (goal_handle_->is_active()) {
                    RCLCPP_WARN(this->get_logger(), "New goal accepted, existing goal aborted");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
        
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
        execute(goal_handle);
    }

    void execute(const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }
        
        int goal_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;
        auto feedback = std::make_shared<MoveRobot::Feedback>();
        auto result = std::make_shared<MoveRobot::Result>();
        rclcpp::Rate loop_rate(1.0);

        while (rclcpp::ok()) {
            int difference = goal_position - robot_position_;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->position = robot_position_;
                    goal_handle->abort(result);
                    return;
                }
            }

            if (goal_handle->is_canceling()) {
                result->position = robot_position_;
                result->message = "Goal cancelled";
                goal_handle->canceled(result);
                return;
            }

            if (!difference) {
                RCLCPP_INFO(this->get_logger(), "Robot has reach it's goal of: %d", goal_position);
                result->position = goal_position;
                goal_handle->succeed(result);
                return;
            }
            else if (difference > 0) {
                robot_position_ += std::min(difference, velocity);
            }
            else {
                robot_position_ += std::max(difference, -velocity);
            }
            RCLCPP_INFO(this->get_logger(), "Robot is at: %d", robot_position_);
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
        
    }

    rclcpp_action::Server<MoveRobot>::SharedPtr move_robot_server_;
    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    int robot_position_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}