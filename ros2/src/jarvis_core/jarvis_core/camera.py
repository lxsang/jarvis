
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
import rclpy
import nanocamera as nano
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile



class JarvisCameraHandle(Node):
    def __init__(self):
        super().__init__('jarvis_camera')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('width', 640),
                ('height', 480),
                ('camera_topic', "camera_image"),
                ('time_period', 0.2),
            ]
        )
        timer_period = self.get_parameter("time_period").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(CompressedImage, self.get_parameter("camera_topic").value, qos_profile)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame = None
        self.has_frame = False
        # self.bridge = CvBridge()
        

        #cv2.VideoCapture(
        #    "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
        self.get_logger().info("topic: %s, w: %d, h: %d, period: %.3f"%(self.get_parameter("camera_topic").value, self.width, self.height, timer_period))
        #self.cap.set(3, self.width)
        #self.cap.set(4, self.height)

    def timer_callback(self):
        if self.has_frame == True:
            # covert frame to image
            #resize_frame = cv2.resize(frame, (self.width, self.height))
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
            # result, encimg = cv2.imencode('.jpg', img, encode_param)
            msg.data = np.array(cv2.imencode('.jpg', self.frame, encode_param)[1]).tobytes()
            self.pub.publish(msg)
            self.has_frame = False


def main(args=None):
    rclpy.init(args=args)
    node = JarvisCameraHandle()
    cap = nano.Camera(flip=2, width=node.width, height=node.height, fps=25)
    while rclpy.ok():
        try:
            if cap.isReady():
                node.frame = cap.read()
                node.has_frame = True
            rclpy.spin_once(node)
        except KeyboardInterrupt:
            cap.release()
            break
    cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
