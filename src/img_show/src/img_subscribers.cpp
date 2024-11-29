#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include <functional>

class ImageSubscriberNode : public rclcpp::Node {
public:
    ImageSubscriberNode() : Node("image_subscriber_node") {
        // 使用 image_transport 创建订阅者
        image_transport::ImageTransport it(shared_from_this());

        // 使用 std::bind 绑定成员函数
        image_subscriber_ = it.subscribe("/image_raw", 10,
            std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received image with width: %d and height: %d", 
                    msg->width, msg->height);
    }

    image_transport::Subscriber image_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
