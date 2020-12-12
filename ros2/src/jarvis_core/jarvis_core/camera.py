
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
import rclpy


class JarvisCameraHandle(Node):
    def __init__(self):
        super().__init__('jarvis_camera')
        timer_period = 0.15  # 15 hz
        self.width = 320
        self.height = 240
        self.pub = self.create_publisher(CompressedImage, 'camera_image', 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
        #self.cap.set(3, self.width)
        #self.cap.set(4, self.height)

    def timer_callback(self):
        r, frame = self.cap.read()
        # covert frame to image
        resize_frame = cv2.resize(frame, (self.width, self.height))
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', resize_frame)[1]).tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = JarvisCameraHandle()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
