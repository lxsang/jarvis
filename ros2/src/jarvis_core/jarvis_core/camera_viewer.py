from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('time_period', 0.033),
                ('camera_topic', '/jarvis/camera')
            ]
        )
        
        self.timer = self.create_timer(self.get_parameter("time_period").value, self.timer_callback)
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.sub = self.create_subscription(
            CompressedImage,
            self.get_parameter("camera_topic").value,
            self.callback,
            qos_profile)
        self.frame = None
        self.has_frame = False

    def callback(self, msg):
        self.has_frame = True
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        # np.frombytes(msg.data, np.uint8)
        self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def timer_callback(self):
        if self.has_frame:
            cv2.imshow("camera", self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    node = CameraViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()