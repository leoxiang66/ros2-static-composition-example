
#include "analysis/analysis.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

using namespace std::chrono_literals;

std::string nanosec2date2(uint64_t nanoseconds) {
    std::time_t seconds = nanoseconds / 1'000'000'000;
    uint64_t microseconds = (nanoseconds % 1'000'000'000) / 1'000;
    
    std::tm tm_utc = *std::gmtime(&seconds);
    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%d %H:%M:%S") << "." << std::setw(6) << std::setfill('0') << microseconds;
    return oss.str();
}

TimestampSyncNode::TimestampSyncNode() : Node("timestamp_sync_node"), image_index(0), pc2_index(0) {
    image_timestamps.resize(CAPACITY, 0);
    pc2_timestamps.resize(CAPACITY, 0);
    image_arrive_times.resize(CAPACITY, 0);
    pc2_arrive_times.resize(CAPACITY, 0);

    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/hikvision/camera/image_raw", 10,
        std::bind(&TimestampSyncNode::image_callback, this, std::placeholders::_1));

    pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&TimestampSyncNode::pc2_callback, this, std::placeholders::_1));
}

void TimestampSyncNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (image_index >= CAPACITY || pc2_index >= CAPACITY) {
        calculate_and_reset();
        return;
    }
    image_arrive_times[image_index] = this->now().nanoseconds();
    image_timestamps[image_index] = static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000'000 + msg->header.stamp.nanosec;
    image_index++;
}

void TimestampSyncNode::pc2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (pc2_index >= CAPACITY || image_index >= CAPACITY) {
        calculate_and_reset();
        return;
    }
    pc2_arrive_times[pc2_index] = this->now().nanoseconds();
    pc2_timestamps[pc2_index] = static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000'000 + msg->header.stamp.nanosec;
    pc2_index++;
}

void TimestampSyncNode::calculate_and_reset() {
    uint64_t sum_diff = 0;
    int valid_pairs = std::min(image_index, pc2_index);

    for (int i = 0; i < valid_pairs; i++) {
        RCLCPP_INFO(this->get_logger(), "Image timestamp: %s", nanosec2date2(image_timestamps[i]).c_str());
        RCLCPP_INFO(this->get_logger(), "PC2 timestamp: %s", nanosec2date2(pc2_timestamps[i]).c_str());
        RCLCPP_INFO(this->get_logger(), "Image arrive timestamp: %s", nanosec2date2(image_arrive_times[i]).c_str());
        RCLCPP_INFO(this->get_logger(), "PC2 arrive timestamp: %s", nanosec2date2(pc2_arrive_times[i]).c_str());
        
        uint64_t diff = std::abs(static_cast<int64_t>(image_timestamps[i]) - static_cast<int64_t>(pc2_timestamps[i]));
        sum_diff += diff;
        RCLCPP_INFO(this->get_logger(), "Diff[%d]: %lu ns", i, diff);
    }

    if (valid_pairs > 0) {
        RCLCPP_INFO(this->get_logger(), "Average timestamp difference: %lu ns", sum_diff / valid_pairs);
        int64_t arrive_diff_sum = 0;
        for (int i = 0; i < valid_pairs; i++) {
            arrive_diff_sum += static_cast<int64_t>(pc2_arrive_times[i]) - static_cast<int64_t>(image_arrive_times[i]);
        }
        RCLCPP_INFO(this->get_logger(), "Average arrive time diff: %ld ns", arrive_diff_sum / valid_pairs);
    }

    image_index = 0;
    pc2_index = 0;
}


