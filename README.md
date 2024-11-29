# README: ROS 2 图像展示与 Web 服务器

本项目包含一个 ROS 2 包 `img_show`，用于展示从摄像头获取的图像，并通过 Web 服务器 (`FastAPI` 和 `Uvicorn`) 提供图像流接口。此包包括一个 ROS 2 节点，它订阅图像话题，并通过 HTTP 提供图像流。

---

## 项目概述

### **功能**
- **图像展示**：通过订阅 ROS 2 图像话题，接收并显示摄像头图像。
- **Web 服务器**：使用 `FastAPI` 和 `Uvicorn` 启动一个 Web 服务器，提供图像流接口。
  - 客户端可以通过浏览器访问 `/image` 路径，查看实时图像。

### **组成部分**
1. **ROS 2 节点**：订阅摄像头图像并处理图像数据。
2. **Web 服务器**：使用 `FastAPI` 提供一个 HTTP 接口，允许通过浏览器查看图像。
3. **图像流**：支持图像流式传输，通过浏览器查看每一帧图像。

---

## 前提条件

- **ROS 2 (Foxy 或更高版本)** 已安装并配置。
- 已安装以下 Python 依赖：
  - `FastAPI`
  - `Uvicorn`
  - `rclpy`
  - `setuptools`
- 摄像头设备（`/dev/video0` 或其他设备）已连接并正常工作。
- ROS 2 包已编译并安装。

---

## 安装步骤

1. **克隆代码仓库** 到 ROS 2 工作空间（例如 `~/ros2_ws/src`）：

    ```bash
    cd ~/ros2_ws/src
    git clone <仓库地址>
    ```

2. **安装 ROS 2 依赖**：

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **构建 ROS 2 工作空间**：

    ```bash
    colcon build --symlink-install
    ```

4. **安装 Python 依赖**：

    使用 `pip` 安装 Python 依赖：

    ```bash
    pip install -r requirements.txt
    ```

5. **安装包**：

    在 ROS 2 工作空间内安装该包：

    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

---

## 运行节点

### 1. **启动图像发布者节点**

首先启动图像发布者节点，它将从摄像头捕获图像并发布到 ROS 2 话题（如 `/image_raw`）。

```bash
ros2 run img_show image_publisher_node
```

该节点会从摄像头读取图像并通过 ROS 2 的 `image_transport` 发布图像。

### 2. **启动 Web 服务器**

启动 `FastAPI` 和 `Uvicorn` Web 服务器，提供图像流接口。浏览器可以通过 `/image` 路径访问实时图像。

```bash
ros2 run img_show web_server
```

Web 服务器将在 `http://localhost:8000/image` 路径上提供图像流。

---

## 代码结构

### `setup.py`

此文件用于配置 ROS 2 包的安装和依赖关系。它包含以下主要部分：

- **`data_files`**：
  - 包含 ROS 2 包的资源文件，例如 `package.xml` 文件和 `web_server.py` 脚本。
  
- **`install_requires`**：
  - 列出此包所需的 Python 库，如 `FastAPI`、`Uvicorn` 和 `rclpy`。

- **`zip_safe`**：
  - 设置为 `True`，表示该包是可以被压缩的。

### `web_server.py`

该 Python 脚本通过 `FastAPI` 启动一个 Web 服务器，提供图像流功能。通过访问 `/image` 路径，浏览器可以查看来自摄像头的实时图像流。

### `package.xml`

ROS 2 包的元数据文件，描述该包的依赖关系和其他信息。

---

## 使用方法

### 1. **启动图像发布节点**

如果尚未启动图像发布节点，请执行以下命令：

```bash
ros2 run img_show image_publisher_node
```

图像发布者节点将开始从摄像头设备采集图像并发布到 ROS 2 话题。

### 2. **启动 Web 服务器**

启动 Web 服务器：

```bash
ros2 run img_show web_server
```

访问 `http://localhost:8000/image`，即可在浏览器中查看实时图像流。

---

## 故障排查

### 1. **摄像头设备无法打开**：
   - 确保 `/dev/video0` 存在，并且当前用户有访问权限。
   - 使用以下命令查看设备是否存在：
     ```bash
     ls /dev/video*
     ```

### 2. **Web 服务器无法启动**：
   - 确保所有必要的 Python 依赖项已安装。
   - 检查是否有其他进程占用了 `8000` 端口。你可以通过以下命令查看哪些进程占用了该端口：
     ```bash
     lsof -i:8000
     ```

### 3. **图像流显示不正常**：
   - 检查 `image_transport` 话题是否正在发布。使用以下命令查看当前的 ROS 2 话题：
     ```bash
     ros2 topic list
     ```

---

## 后续改进

- **支持多种图像格式**：支持不同的图像编码格式（如 `MJPEG`、`YUYV` 等）。
- **优化 Web 服务器性能**：提高图像流的传输效率，支持更高的帧率。
- **增加认证和权限管理**：为 Web 服务器添加用户认证和权限控制，以提高安全性。

---

## 许可证

本项目遵循 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。

