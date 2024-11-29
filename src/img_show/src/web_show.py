# import rclpy
# from fastapi import FastAPI
# from fastapi.responses import StreamingResponse
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from io import BytesIO

# # FastAPI应用
# app = FastAPI()

# # ROS 2 节点初始化
# rclpy.init()
# node = Node('image_subscriber')
# latest_image = None

# def image_callback(msg: Image):
#     global latest_image
#     latest_image = msg

# # 订阅 /image_raw 话题
# image_subscription = node.create_subscription(
#     Image,
#     '/image_raw',
#     image_callback,
#     10
# )

# @app.on_event("startup")
# async def startup():
#     # 确保 ROS 2 节点正在运行
#     node.create_timer(0.1, lambda: None)  # Ensure the node is spinning

# @app.get("/image")
# async def get_image():
#     global latest_image
#     if latest_image is not None:
#         # 将 ROS 2 图像数据转换为字节流
#         image_data = BytesIO(latest_image.data)
#         return StreamingResponse(image_data, media_type="image/jpeg")
#     else:
#         return {"message": "No image available"}

# if __name__ == "__main__":
#     import uvicorn
#     uvicorn.run(app, host="0.0.0.0", port=8000)
import rclpy
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from rclpy.node import Node
from sensor_msgs.msg import Image
from io import BytesIO
from PIL import Image as PILImage
import time

# FastAPI应用
app = FastAPI()

# ROS 2 节点初始化
rclpy.init()
node = Node('image_subscriber')
latest_image = None
frame_rate = 30  # 设置帧率（每秒帧数）

def image_callback(msg: Image):
    global latest_image
    latest_image = msg

# 订阅 /image_raw 话题
image_subscription = node.create_subscription(
    Image,
    '/image_raw',
    image_callback,
    10
)

@app.on_event("startup")
async def startup():
    # 确保 ROS 2 节点正在运行
    node.create_timer(0.1, lambda: None)  # Ensure the node is spinning

def generate_video_stream():
    global latest_image
    while True:
        if latest_image is not None:
            # 从ROS 2图像消息转换为PIL图像
            np_image = np.array(latest_image.data, dtype=np.uint8)
            height = latest_image.height
            width = latest_image.width
            np_image = np_image.reshape((height, width, 3))  # 假设是RGB格式

            pil_image = PILImage.fromarray(np_image)

            # 将PIL图像编码为JPEG格式
            with BytesIO() as byte_io:
                pil_image.save(byte_io, format='JPEG')
                byte_io.seek(0)
                byte_img = byte_io.read()

            # 返回帧
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + byte_img + b'\r\n\r\n')

            # 控制帧率
            time.sleep(1 / frame_rate)
        else:
            # 如果没有图像数据，则继续等待
            time.sleep(0.1)

@app.get("/video")
async def get_video():
    return StreamingResponse(generate_video_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
