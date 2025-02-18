#ifndef IMAGE_PUBLISHER_NODE_HPP
#define IMAGE_PUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "utils.h"
#include <thread> // 为了使用std::thread

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode();
    ~ImagePublisherNode();  // 析构函数用于确保线程结束

private:
    // 发布器主题名称
    std::string image_topic_ = "hikvision/camera/image_raw";
    
    // 图像发布器
    image_transport::Publisher image_pub_;
    
    // 用于运行 camera_work 的线程
    std::thread camera_thread_;

    // 在新线程中启动 camera_work
    void start_camera_work(int frequency, int sync_point, float exposure_time, image_transport::Publisher image_pub);
};

#endif // IMAGE_PUBLISHER_NODE_HPP
