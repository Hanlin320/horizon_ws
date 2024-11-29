// #include <sys/types.h>
// #include <sys/ipc.h>
// #include <sys/shm.h>
// #include <stdio.h>
// #include <string.h>
// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include <image_transport/image_transport.hpp>
// #include "sensor_msgs/msg/image.hpp"

// int main() {
//     // 初始化 ROS 2 节点
//     rclcpp::init(0, nullptr);
//     auto node = rclcpp::Node::make_shared("shared_memory_image_publisher");

//     // 使用 image_transport 创建发布者
//     image_transport::ImageTransport it(node);
//     image_transport::Publisher publisher = it.advertise("/shared_image", 10);

//     key_t key = ftok("/home", 0x666);
//     if (key == -1) {
//         perror("ftok error");
//         return -1;
//     }
//     std::cout << "ftok : " << key << std::endl;

//     int shmId = shmget(key, 4096, IPC_CREAT | 0600);
//     if (shmId == -1) {
//         perror("shmget error");
//         return -1;
//     }
//     std::cout << "shmget : " << shmId << std::endl;

//     while (rclcpp::ok()) {
//         // 获取共享内存地址
//         void *shm_addr = shmat(shmId, NULL, 0);
//         if (shm_addr == (void*)-1) {
//             perror("shmat error");
//             return -1;
//         }

// 		std::cout << "Shared memory address: " << shm_addr << std::endl;

//         // 取出数据
//         size_t dataSize = 614989; // 确保数据大小是正确的
//         unsigned char *data = static_cast<unsigned char*>(malloc(dataSize));
// 		if (!data) {
//     	perror("malloc error");
//     	return -1;
// 		}
//         memcpy(data, shm_addr, dataSize);
//         std::cout << "The data has been copied into the buffer." << std::endl;

// 		// 打印数据地址（仅调试）
//         std::cout << "The data address: " << (void*)data << std::endl;

//         // 创建 ROS 图像消息
//         sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
//         image_msg->header.stamp = node->get_clock()->now();
//         image_msg->height = 480;  // 请根据实际图像的高度设置
//         image_msg->width = 640;   // 请根据实际图像的宽度设置
//         image_msg->encoding = "nv12"; // 这里的编码要与共享内存中的编码一致
//         image_msg->is_bigendian = false;
//         image_msg->step = 640; // 一行图像的字节数，根据图像宽度设置

//         // 将共享内存中的数据放入图像消息
//         image_msg->data.insert(image_msg->data.end(), data, data + dataSize);

//         // 发布图像消息
//         publisher.publish(*image_msg);

//         // 断开共享内存
//         shmdt(shm_addr);

//         // 休息一会，确保发布频率
//         rclcpp::spin_some(node);
//     }

//     // 清理共享内存
//     int sh = shmctl(shmId, IPC_RMID, NULL);
//     if (sh == -1) {
//         perror("shmctl error");
//         return -1;
//     }

//     std::cout << "Shared memory cleaned up.\n";
//     rclcpp::shutdown();
//     return 0;
// }
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"

int main() {
    // 初始化 ROS 2 节点
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("shared_memory_image_publisher");

    // 使用 image_transport 创建发布者
    image_transport::ImageTransport it(node);
    image_transport::Publisher publisher = it.advertise("/shared_image", 10);

    // 获取共享内存的唯一标识符
    key_t key = ftok("/home", 0x666);
    if (key == -1) {
        perror("ftok error");
        return -1;
    }
    std::cout << "ftok : " << key << std::endl;

    // 共享内存的实际大小（确保与图像数据一致）
    size_t dataSize = 614989; // 数据的大小，需要与共享内存中的数据大小一致

    // 获取共享内存的 ID
    int shmId = shmget(key, dataSize, IPC_CREAT | 0600);
    if (shmId == -1) {
        perror("shmget error");
        return -1;
    }
    std::cout << "shmget : " << shmId << std::endl;

    // 映射共享内存
    void *shm_addr = shmat(shmId, NULL, 0);
    if (shm_addr == (void*)-1) {
        perror("shmat error");
        return -1;
    }

    // 创建缓冲区
    unsigned char *data = static_cast<unsigned char*>(malloc(dataSize));
    if (!data) {
        perror("malloc error");
        return -1;
    }

    // 循环读取共享内存并发布图像
    while (rclcpp::ok()) {
        // 从共享内存中复制数据到缓冲区
        memcpy(data, shm_addr, dataSize); // 拷贝共享内存中的数据到缓冲区
        std::cout << "The data has been copied into the buffer." << std::endl;

        // 创建 ROS 图像消息
        sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_msg->header.stamp = node->get_clock()->now();
        image_msg->height = 480;  // 图像高度
        image_msg->width = 640;   // 图像宽度
        image_msg->encoding = "nv12"; // 编码格式
        image_msg->is_bigendian = false;
        image_msg->step = 640; // 每行的字节数，NV12 格式时可能需要调整
        image_msg->data.insert(image_msg->data.end(), data, data + dataSize);

        // 发布图像消息
        publisher.publish(*image_msg);

        // 调用 rclcpp::spin_some，确保其他 ROS 2 的回调函数能执行
        rclcpp::spin_some(node);
    }

    // 清理共享内存和动态分配的内存
    free(data);
    int sh = shmctl(shmId, IPC_RMID, NULL);  // 删除共享内存
    if (sh == -1) {
        perror("shmctl error");
        return -1;
    }

    std::cout << "Shared memory cleaned up.\n";

    // 断开共享内存
    shmdt(shm_addr);

    rclcpp::shutdown();
    return 0;
}
