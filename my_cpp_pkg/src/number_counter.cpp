#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node {
public:
    NumberCounterNode() : Node("number_counter") {
        total_ = 0;
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                                    std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        server_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
                    std::bind(&NumberCounterNode::callbackResetNumber, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Number counter has been started");
    }

private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg) {
        total_+= msg->data;
        RCLCPP_INFO(this->get_logger(), "Received %ld, the total is now %d.", msg->data, total_);
        auto num = example_interfaces::msg::Int64();
        num.data = total_;
        publisher_->publish(num);
    }

    void callbackResetNumber(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                             const example_interfaces::srv::SetBool::Response::SharedPtr response) {
        if (request->data) {
            total_ = 0;
            response->success = true;
            response->message = "Counter reset";
        }
        else {
            response->success = false;
            response->message = "Counter not reset";
        }
    }

    int total_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}