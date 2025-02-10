#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h>  // 用于 getpid()

// Node1: 每隔1秒发布 "Hello from Node 1" 并打印进程 ID
class Node1 : public rclcpp::Node {
public:
    Node1() : Node("node1") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("test", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto message = std_msgs::msg::String();
                message.data = "Hello from Node 1";
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
                RCLCPP_INFO(this->get_logger(), "Node1 running in process ID: %d", getpid());
            }
        );
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

// Node2: 订阅 "test" 主题，每次收到消息时打印，并打印进程 ID
class Node2 : public rclcpp::Node {
public:
    Node2() : Node("node2") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "test", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
                RCLCPP_INFO(this->get_logger(), "Node2 running in process ID: %d", getpid());
            }
        );
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<Node1>();
    auto node2 = std::make_shared<Node2>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
