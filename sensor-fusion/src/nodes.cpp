#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h> // 用于 getpid()
#include "image_publisher.h"
#include "driver_node.h"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<ImagePublisherNode>();
    auto node2 = std::make_shared<livox_ros::DriverNode>(rclcpp::NodeOptions());


    RCLCPP_INFO(node1->get_logger(), "Adding Node1 to executor...");
    RCLCPP_INFO(node2->get_logger(), "Adding Node2 to executor...");


    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);

    RCLCPP_INFO(node1->get_logger(), "Starting executor...");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

