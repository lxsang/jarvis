import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class T2MVelocity(Node):
    def __init__(self):
        super().__init__('t2mvel')
        # space between two wheels in meter
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_space', 0.15),
                ('time_period', 0.02),
                ('lwheel_vel_topic', "lwheel_vel"),
                ('rwheel_vel_topic', "rwheel_vel"),
                ('cmd_vel_topic', "/vel")
            ]
        )
        
        self.wheel_space = self.get_parameter("wheel_space").value
        # rate of the node in Hz
        time_period = self.get_parameter("time_period").value
        # left wheel velocity topic name
        lwheel_vel_topic = self.get_parameter("lwheel_vel_topic").value
        #right wheel velocity topic name
        rwheel_vel_topic = self.get_parameter("rwheel_vel_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        
        self.timer = self.create_timer(time_period, self.timer_callback)
        
        self.lwheel_vel_pub = self.create_publisher(Float32, lwheel_vel_topic, 10)
        self.rwheel_vel_pub = self.create_publisher(Float32, rwheel_vel_topic, 10)
        # twist topic name
        
        self.sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.twist_callback,
            10)
        
        self.dx = 0
        self.dy = 0
        self.theta = 0
        self.has_data = False
        self.get_logger().info("Wheel space: %.3f, period: %.3f, lwheel_topic: %s, rwheel_topic: %s, cmd_vel_topic: %s" %(self.wheel_space, time_period, lwheel_vel_topic, rwheel_vel_topic, cmd_vel_topic))
        
        
    def twist_callback(self, msg):
        self.dx = msg.linear.x
        self.dy = msg.linear.y
        self.theta = msg.angular.z
        self.has_data = True
    
    #def loop(self):
    #    hz = rospy.Rate(self.rate)
    #    while rospy.is_shutdown() == False:
    #        self.loopOnce()
    #        hz.sleep()
    
    def timer_callback(self):
        self.loopOnce()
    
    def loopOnce(self):
        ml_speed = Float32()
        ml_speed.data = (self.dx - self.theta*self.wheel_space/2.0)*1000.0
        mr_speed = Float32()
        mr_speed.data = (self.dx + self.theta*self.wheel_space/2.0)*1000.0
        #publish the speed
        #rospy.loginfo("left speed %.2f, right speed: %2f", ml_speed, mr_speed)
        if self.has_data:
            self.lwheel_vel_pub.publish(ml_speed)
            self.rwheel_vel_pub.publish(mr_speed)
            self.has_data = False
        
def main(args=None):
    rclpy.init(args=args)

    node = T2MVelocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()