import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Nv12ToMjpegNode(Node):
    def __init__(self):
        super().__init__('nv12_to_mjpeg_converter_py')

        # 订阅 "/nv12_image" 主题
        self.subscription = self.create_subscription(
            Image,
            '/nv12_image',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告

        # 创建 CvBridge 实例
        self.bridge = CvBridge()

        # MJPEG 发布者
        self.mjpeg_publisher = self.create_publisher(Image, '/mjpeg_image', 10)

    def listener_callback(self, msg):
        try:
            # 将接收到的消息转换为 OpenCV 格式的图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # 假设数据是 NV12 格式, 然后进行 MJPEG 编码
            if cv_image is not None:
                # 将 NV12 数据转换为 BGR（MJPEG 需要 RGB/BGR 格式）
                height, width = cv_image.shape
                nv12_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height + height // 2, width))
                yuv_image = cv2.cvtColor(nv12_data, cv2.COLOR_YUV2BGR_NV12)

                # 将 BGR 图像转换为 MJPEG 格式
                ret, jpeg_image = cv2.imencode('.jpg', yuv_image)

                if ret:
                    # 将 JPEG 编码图像转换回 ROS 图像消息
                    mjpeg_msg = self.bridge.cv2_to_imgmsg(jpeg_image, encoding="jpeg")

                    # 发布 MJPEG 图像消息
                    self.mjpeg_publisher.publish(mjpeg_msg)
                else:
                    self.get_logger().error("JPEG 编码失败")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge 错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    nv12_to_mjpeg_node = Nv12ToMjpegNode()
    rclpy.spin(nv12_to_mjpeg_node)

    # 清理节点
    nv12_to_mjpeg_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
