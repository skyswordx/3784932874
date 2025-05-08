import cv2
import psutil
import time
import threading

def print_system_usage(interval=1):
    while True:
        # 获取CPU使用率
        cpu_usage = psutil.cpu_percent(interval=0.5)
        # 获取内存使用情况
        memory_info = psutil.virtual_memory()
        memory_usage = memory_info.percent

        print(f"CPU Usage: {cpu_usage}%")
        print(f"Memory Usage: {memory_usage}%")

        time.sleep(interval)

def start_camera():
    # 打开摄像头，0 表示默认摄像头
    cap = cv2.VideoCapture(0)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return

    # 设置摄像头分辨率和帧率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # 主循环，读取并显示视频帧
    while True:
        # 丢弃缓冲区中的旧帧，只保留最新帧
        for _ in range(150):  # 根据实际情况调整抓取次数
            cap.grab()

        # 读取最新帧
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # 显示当前帧
        cv2.imshow('Camera Feed', frame)

        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头和关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # 启动系统资源监控线程
    monitoring_thread = threading.Thread(target=print_system_usage, daemon=True)
    monitoring_thread.start()

    # 启动摄像头
    start_camera()
