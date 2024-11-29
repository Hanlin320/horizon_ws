from flask import Flask, render_template, Response
import cv2
import numpy as np
 
app = Flask(__name__)

def gen_frames():  # 使用生成器来源源不断地提供画面帧
    cap = cv2.VideoCapture(0)  # 打开摄像头，这里为了示例我们用0代表摄像头编号
    while True:
        ret, frame = cap.read()  # 读取一帧画面
        #cap.set(cv2.CAP_PROP_FPS, 30)
        # 设置帧宽度和高度，如果需要的话
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not ret:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)  # 将帧编码为JPEG格式
            frame = buffer.tobytes()  # 将帧转换为字节流
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # 发送帧数据
 
@app.route('/')
def index():
    return render_template('index.html')
 
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
 
if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)