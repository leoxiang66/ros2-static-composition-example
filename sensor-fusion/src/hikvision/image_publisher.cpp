#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "utils.h"
#include "image_publisher.h"
#include <thread> // 引入std::thread

ImagePublisherNode::ImagePublisherNode() : Node("hikvision_image_publisher")
{
    // Create a shared_ptr to this node
    auto node_shared_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    // Initialize image_transport with the shared_ptr
    image_transport::ImageTransport it(node_shared_ptr);

    // Read parameters
    declare_parameter<float>("exposure_time", 10000.0);
    declare_parameter<int>("frequency", 10);
    declare_parameter<int>("sync_point", 10000000);
    declare_parameter<float>("gamma", 0.3);
    declare_parameter<float>("gain", 15.0);

    float exposure_time = get_parameter("exposure_time").get_value<float>();
    int frequency = get_parameter("frequency").get_value<int>();
    int sync_point = get_parameter("sync_point").get_value<int>();
    float gamma = get_parameter("gamma").get_value<float>();
    float gain = get_parameter("gain").get_value<float>();

    std::cout << "Exposure Time: " << exposure_time << std::endl;
    std::cout << "Frequency: " << frequency << std::endl;
    std::cout << "Sync point: " << sync_point << std::endl;
    std::cout << "Gamma correction: " << gamma << std::endl;
    std::cout << "Gain: " << gain << std::endl;

    // Advertise the image topic
    image_pub_ = it.advertise(image_topic_, 1);

    RCLCPP_INFO(this->get_logger(), "Starting camera work...");

    // 启动一个新的线程来运行camera_work
    camera_thread_ = std::thread(&ImagePublisherNode::start_camera_work, this, frequency, sync_point, exposure_time,gamma, gain ,image_pub_);
}

void ImagePublisherNode::start_camera_work(int frequency, int sync_point, float exposure_time ,float gamma, float gain,image_transport::Publisher image_pub)
{
    // RCLCPP_INFO(this->get_logger(), "Node3 running in process ID: %d", getpid());
    camera_work(
        0,                    // Camera index
        frequency,            // Publish frequency
        sync_point,           // Sync point
        exposure_time,        // Exposure time
        gamma, 
        gain,
        image_pub            // Image publisher
    );
}

ImagePublisherNode::~ImagePublisherNode()
{
    // 在节点析构时确保线程正确结束
    if (camera_thread_.joinable()) {
        camera_thread_.join();  // 等待线程结束
    }
}
