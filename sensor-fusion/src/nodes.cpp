#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h> // 用于 getpid()
#include "image_publisher.h"
#include "driver_node.h"
#include "analysis/analysis.h"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<ImagePublisherNode>();
    auto node2 = std::make_shared<livox_ros::DriverNode>(rclcpp::NodeOptions());
    auto node3 = std::make_shared<TimestampSyncNode>();

    RCLCPP_INFO(node1->get_logger(), "Adding Node1 to executor...");
    RCLCPP_INFO(node2->get_logger(), "Adding Node2 to executor...");
    RCLCPP_INFO(node3->get_logger(), "Adding Node3 to executor...");


    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.add_node(node3);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

