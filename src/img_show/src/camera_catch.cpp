// #include "rclcpp/rclcpp.hpp"
// #include <stdio.h>
// #include <sys/types.h>
// #include <sys/ipc.h>
// #include <sys/shm.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <boost/asio.hpp>
// #include <sys/ioctl.h>
// #include <linux/videodev2.h>
// #include <string.h>
// #include <sys/mman.h>
// #include "sensor_msgs/msg/compressed_image.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include <image_transport/image_transport.hpp> // 使用 image_transport

// int main(void) 
// {
//     rclcpp::init(0, nullptr);
//     auto node = rclcpp::Node::make_shared("image_publisher_node");

//     // 使用 image_transport 创建发布者
//     image_transport::ImageTransport it(node);
//     image_transport::Publisher publisher = it.advertise("/image_raw", 10);

//     // 打开摄像头设备
//     int fd = open("/dev/video0", O_RDWR);
//     if (fd < 0) {
//         perror("打开设备失败");
//         return -1;
//     }

//     // 获取摄像头支持格式
//     struct v4l2_format vfmt;
//     vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 摄像头采集
//     vfmt.fmt.pix.width = 640; // 设置摄像头采集参数
//     vfmt.fmt.pix.height = 480;
//     vfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12; // 设置为 NV12
//     std::cout << "format: " << vfmt.fmt.pix.pixelformat << std::endl;
//     if (vfmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12) {
//         std::cout << "format: NV12" << std::endl;
//     }

//     int ret = ioctl(fd, VIDIOC_S_FMT, &vfmt);
//     if (ret < 0) {
//         perror("设置格式失败1");
//     }

//     // 申请缓冲区
//     struct v4l2_requestbuffers reqbuffer;
//     reqbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//     reqbuffer.count = 4; // 申请4个缓冲区
//     reqbuffer.memory = V4L2_MEMORY_MMAP; // 映射方式
//     ret = ioctl(fd, VIDIOC_REQBUFS, &reqbuffer);
//     if (ret < 0) {
//         perror("申请空间失败");
//     }

//     // 映射到用户空间
//     unsigned char *mptr[4]; // 保存映射后用户空间的首地址
//     unsigned int size[4];
//     struct v4l2_buffer mapbuffer;
//     mapbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

//     for (int i = 0; i < 4; i++) {
//         mapbuffer.index = i;
//         ret = ioctl(fd, VIDIOC_QUERYBUF, &mapbuffer); // 从内核空间中查询一个空间作映射
//         if (ret < 0) {
//             perror("查询内核空间失败");
//         }
//         // 映射到用户空间
//         mptr[i] = (unsigned char *)mmap(NULL, mapbuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mapbuffer.m.offset);
//         size[i] = mapbuffer.length; // 保存映射长度用于后期释放
//         // 查询后通知内核已经放回
//         ret = ioctl(fd, VIDIOC_QBUF, &mapbuffer);
//         if (ret < 0) {
//             perror("放回失败");
//         }
//     }

//     int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//     ret = ioctl(fd, VIDIOC_STREAMON, &type); 
//     if (ret < 0) {
//         perror("开启失败");
//     }

//     // 发布图像
//     while (true) {
//         struct v4l2_buffer readbuffer;
//         readbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//         ret = ioctl(fd, VIDIOC_DQBUF, &readbuffer);
//         if (ret < 0) {
//             perror("读取数据失败");
//             break;
//         }

//         // 创建一个 ROS 2 图像消息
//         sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
//         image_msg->header.stamp = node->get_clock()->now();
//         image_msg->height = 480;  // 图像的高度
//         image_msg->width = 640;   // 图像的宽度
//         image_msg->encoding = "nv12"; // 图像编码格式
//         image_msg->is_bigendian = false;
//         image_msg->step = 640;  // 每一行的字节数

//         size_t dataSize = 614989; // 确保数据大小是正确的
//         image_msg->data.resize(dataSize);
//         memcpy(image_msg->data.data(), mptr[readbuffer.index], dataSize); // 将数据复制到 ROS 2 消息

//         // 发布图像消息
//         publisher.publish(*image_msg);

//         // 释放队列
//         ret = ioctl(fd, VIDIOC_QBUF, &readbuffer);
//         if (ret < 0) {
//             perror("放回队列失败");
//         }
//     }

//     // 停止采集
//     ret = ioctl(fd, VIDIOC_STREAMOFF, &type);

//     // 释放映射
//     for (int i = 0; i < 4; i++) {
//         munmap(mptr[i], size[i]);
//     }

//     close(fd); // 关闭文件
//     rclcpp::shutdown();
//     return 0;
// }


#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <string.h>
#include <sys/mman.h>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp> // 使用 image_transport

int main(void) 
{
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("image_publisher_node");

    // 使用 image_transport 创建发布者
    image_transport::ImageTransport it(node);
    image_transport::Publisher publisher = it.advertise("/image_raw", 10);

    // 打开摄像头设备
    int fd = open("/dev/video0", O_RDWR);
    if (fd < 0) {
        perror("打开设备失败");
        return -1;
    }

    // 获取摄像头支持格式
    struct v4l2_format vfmt;
    vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 摄像头采集
    vfmt.fmt.pix.width = 640; // 设置摄像头采集参数
    vfmt.fmt.pix.height = 480;
    vfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12; // 设置为 NV12
    vfmt.fmt.pix.field = V4L2_FIELD_NONE; // 不使用字段模式
    vfmt.fmt.pix.sizeimage = (vfmt.fmt.pix.width * vfmt.fmt.pix.height * 3) / 2; // NV12格式的图像大小
    vfmt.fmt.pix.bytesperline = vfmt.fmt.pix.width; // 每行的字节数

    // 配置帧率为30fps
    struct v4l2_streamparm stream_params;
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &stream_params) == 0) {
        stream_params.parm.capture.timeperframe.numerator = 1;
        stream_params.parm.capture.timeperframe.denominator = 30;  // 设置帧率为30fps
        ioctl(fd, VIDIOC_S_PARM, &stream_params);
    }

    int ret = ioctl(fd, VIDIOC_S_FMT, &vfmt);
    if (ret < 0) {
        perror("设置格式失败");
        close(fd);
        return -1;
    }

    // 申请缓冲区
    struct v4l2_requestbuffers reqbuffer;
    reqbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuffer.count = 4; // 申请4个缓冲区
    reqbuffer.memory = V4L2_MEMORY_MMAP; // 映射方式
    ret = ioctl(fd, VIDIOC_REQBUFS, &reqbuffer);
    if (ret < 0) {
        perror("申请空间失败");
        close(fd);
        return -1;
    }

    // 映射到用户空间
    unsigned char *mptr[4]; // 保存映射后用户空间的首地址
    unsigned int size[4];
    struct v4l2_buffer mapbuffer;
    mapbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    for (int i = 0; i < 4; i++) {
        mapbuffer.index = i;
        ret = ioctl(fd, VIDIOC_QUERYBUF, &mapbuffer); // 从内核空间中查询一个空间作映射
        if (ret < 0) {
            perror("查询内核空间失败");
            close(fd);
            return -1;
        }
        // 映射到用户空间
        mptr[i] = (unsigned char *)mmap(NULL, mapbuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mapbuffer.m.offset);
        size[i] = mapbuffer.length; // 保存映射长度用于后期释放
        // 查询后通知内核已经放回
        ret = ioctl(fd, VIDIOC_QBUF, &mapbuffer);
        if (ret < 0) {
            perror("放回失败");
            close(fd);
            return -1;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMON, &type); 
    if (ret < 0) {
        perror("开启失败");
        close(fd);
        return -1;
    }

    // 发布图像
    while (rclcpp::ok()) {
        struct v4l2_buffer readbuffer;
        readbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ret = ioctl(fd, VIDIOC_DQBUF, &readbuffer);
        if (ret < 0) {
            perror("读取数据失败");
            break;
        }

        // 创建一个 ROS 2 图像消息
        sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_msg->header.stamp = node->get_clock()->now();
        image_msg->height = 480;  // 图像的高度
        image_msg->width = 640;   // 图像的宽度
        image_msg->encoding = "nv12"; // 图像编码格式
        image_msg->is_bigendian = false;
        image_msg->step = 640;  // 每一行的字节数

        size_t dataSize = vfmt.fmt.pix.sizeimage; // 确保数据大小是正确的
        image_msg->data.resize(dataSize);
        memcpy(image_msg->data.data(), mptr[readbuffer.index], dataSize); // 将数据复制到 ROS 2 消息

        // 发布图像消息
        publisher.publish(*image_msg);

        // 释放队列
        ret = ioctl(fd, VIDIOC_QBUF, &readbuffer);
        if (ret < 0) {
            perror("放回队列失败");
            break;
        }
    }

    // 停止采集
    ret = ioctl(fd, VIDIOC_STREAMOFF, &type);

    // 释放映射
    for (int i = 0; i < 4; i++) {
        munmap(mptr[i], size[i]);
    }

    close(fd); // 关闭文件
    rclcpp::shutdown();
    return 0;
}
