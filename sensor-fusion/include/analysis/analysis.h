// TimestampSyncNode.hpp
#ifndef TIMESTAMP_SYNC_NODE_HPP
#define TIMESTAMP_SYNC_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <vector>
#include <string>

#define CAPACITY 100

class TimestampSyncNode : public rclcpp::Node {
public:
    TimestampSyncNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void pc2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void calculate_and_reset();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub;

    std::vector<uint64_t> image_timestamps;
    std::vector<uint64_t> pc2_timestamps;
    std::vector<uint64_t> image_arrive_times;
    std::vector<uint64_t> pc2_arrive_times;

    int image_index;
    int pc2_index;
};

std::string nanosec2date2(uint64_t nanoseconds);

#endif // TIMESTAMP_SYNC_NODE_HPP
