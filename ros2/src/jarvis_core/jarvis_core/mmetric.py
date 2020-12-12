import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


class JarvisMetric(Node):
    def __init__(self):
        super().__init__('jarvis_mmetric')
        # timer_period = 0.1  # 1 hz
        self.tick_per_m = 4495
        self.motor_ratio = 1.0
        self.last_ltick = None
        self.last_rtick = None
        self.mlpub = self.create_publisher(Int16, 'left_motor_pwm', 10)
        self.mrpub = self.create_publisher(Int16, 'right_motor_pwm', 10)
        self.lwsub = self.create_subscription(
            Int16,
            'left_tick',
            self.lw_call_back,
            10)
        self.rwsub = self.create_subscription(
            Int16,
            'right_tick',
            self.rw_call_back,
            10)

    def lw_call_back(self, msg):
        if self.last_ltick == None:
            self.last_ltick = msg.data
        else:
            if (msg.data - self.last_ltick > self.tick_per_m):
                vmsg = Int16()
                vmsg.data = 0
                self.mlpub.publish(vmsg)
                print("left wheel finish at: ", msg.data - self.last_ltick)
                self.last_ltick = None

    def rw_call_back(self, msg):
        if self.last_rtick == None:
            self.last_rtick = msg.data
        else:
            if (msg.data - self.last_rtick > self.tick_per_m):
                vmsg = Int16()
                vmsg.data = 0
                self.mrpub.publish(vmsg)
                print("right wheel finish at: ", msg.data - self.last_rtick)
                self.last_rtick = None

    def set_speed(self, sp):
        msg = Int16()
        self.get_logger().info("Set two motors speeds at %d" % sp)
        msg.data = sp
        self.mrpub.publish(msg)
        msg.data = round(msg.data*self.motor_ratio)
        self.mlpub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = JarvisMetric()

    node.set_speed(150)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
