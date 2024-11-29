# ROS 2 图像发布者与订阅者

本项目包含两个 ROS 2 节点：一个用于从摄像头设备（`/dev/video0`）发布图像，另一个用于订阅已发布的图像数据。发布者节点通过 Video4Linux2 (V4L2) API 获取摄像头图像，并将其转化为 `sensor_msgs::msg::Image` 消息格式发布。订阅者节点接收这些图像并记录每个图像的宽度和高度。

---

## 项目概述

### 1. **图像发布者节点** (`image_publisher_node`)
- 从摄像头设备（`/dev/video0`）通过 V4L2 获取图像。
- 将图像转换为 ROS 2 `sensor_msgs::msg::Image` 消息格式。
- 将图像发布到 `/image_raw` 话题。

### 2. **图像订阅者节点** (`image_subscriber_node`)
- 订阅 `/image_raw` 话题。
- 记录接收到的每个图像的宽度和高度。

---

## 前提条件

在运行节点之前，请确保以下条件已经满足：

- 已安装并正确配置 **ROS 2 (Foxy 或更高版本)**。
- 有一个可用的摄像头设备 `/dev/video0`（如果设备路径不同，可以在代码中修改）。
- 安装了必要的依赖：
  - `sensor_msgs`
  - `image_transport`
  - `rclcpp`
  - `boost`
  - `v4l-utils`（用于处理 V4L2 设备）
  - `libboost` 和 `libv4l-dev`（用于支持 V4L2）

如果缺少这些依赖，可以使用 `apt` 安装：

```bash
sudo apt-get install libboost-all-dev libv4l-dev v4l-utils
```

---

## 安装步骤

1. **克隆代码仓库** 到你的 ROS 2 工作空间（例如 `~/ros2_ws/src`）：

    ```bash
    cd ~/ros2_ws/src
    git clone <仓库地址>
    ```

2. **安装 ROS 2 包依赖**：

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **构建工作区**：

    ```bash
    colcon build --symlink-install
    ```

4. **配置环境**：

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

---

## 运行节点

### 1. **启动图像发布者节点**

运行图像发布者节点，该节点从摄像头捕获图像并发布：

```bash
ros2 run <你的包名> image_publisher_node
```

该节点将从 `/dev/video0` 捕获图像，并将图像发布到 `/image_raw` 话题。

### 2. **启动图像订阅者节点**

运行图像订阅者节点，该节点订阅 `/image_raw` 话题并记录图像的宽度和高度：

```bash
ros2 run <你的包名> image_subscriber_node
```

该订阅者节点将在接收到图像时，打印图像的宽度和高度。

---

## 代码讲解

### **图像发布者节点** (`image_publisher_node`)

1. **打开摄像头设备**：
    - 使用 `open()` 系统调用打开 `/dev/video0` 设备。

2. **配置摄像头格式**：
    - 使用 V4L2 设置摄像头的采集格式为 `V4L2_PIX_FMT_NV12`，这是一个常用的 YUV 格式。

3. **配置帧率**：
    - 使用 `VIDIOC_S_PARM` ioctl 调用设置帧率为 30fps。

4. **内存映射**：
    - 使用 `mmap()` 将摄像头的缓冲区映射到用户空间，这样可以直接读取图像数据。

5. **开始流式传输**：
    - 使用 `VIDIOC_STREAMON` 启动视频流，然后进入循环读取图像数据，并将其打包成 ROS 2 消息进行发布。

6. **发布图像**：
    - 使用 `image_transport::Publisher` 将图像发布到 `/image_raw` 话题，图像的编码格式为 `nv12`。

7. **清理资源**：
    - 在停止流式传输后，解除内存映射并关闭摄像头设备。

### **图像订阅者节点** (`image_subscriber_node`)

1. **订阅话题**：
    - 使用 `image_transport::Subscriber` 订阅 `/image_raw` 话题，接收到图像时触发回调函数。

2. **回调函数**：
    - 每当接收到新图像时，节点将记录并打印图像的宽度和高度。

---

## 故障排查

### 1. **找不到摄像头设备**：
   - 确保摄像头设备已连接并被系统识别。
   - 使用以下命令检查 `/dev/video0` 是否存在：
     ```bash
     ls /dev/video*
     ```

### 2. **打开摄像头失败**：
   - 如果 `open()` 调用失败，检查 `/dev/video0` 的权限，确保当前用户有访问摄像头的权限。
   - 使用以下命令查看设备权限：
     ```bash
     ls -l /dev/video0
     ```

### 3. **图像显示异常**：
   - 确保摄像头支持 `NV12` 格式。如果使用不同的像素格式，可以根据需要调整代码。

### 4. **ROS 2 话题**：
   - 确保 `/image_raw` 话题已由发布者节点发布，并且订阅者节点已正确订阅。
   - 使用以下命令查看当前可用的 ROS 2 话题：
     ```bash
     ros2 topic list
     ```

---

## 后续改进

- **图像显示**：可以将订阅者节点扩展为使用 `rqt_image_view` 或其他工具显示接收到的图像。
- **支持多种格式**：添加对更多图像格式（例如 `YUYV`、`MJPEG`）的支持。
- **增强错误处理**：对摄像头设备、内存分配等各类可能出现的错误进行更全面的处理。
- **摄像头参数化**：将摄像头参数（分辨率、帧率、像素格式）设置为 ROS 参数，以便动态调整。

---

## 许可证

本项目遵循 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。