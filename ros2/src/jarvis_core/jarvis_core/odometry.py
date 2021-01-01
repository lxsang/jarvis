import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
# from tf.broadcaster import TransformBroadcaster


class Ticks2Odometry(Node):
    def __init__(self):
        super().__init__('odometry')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lelf_wheel_tick_topic', "lwheel_tick"),
                ('right_wheel_tick_topic', "rwheel_tick"),
                ('ticks_per_meter', 4495),
                ('wheels_space', 0.192),
                ('odom_frame', "odom"),
                ('odom_topic', "odom"),
                ('base_frame', 'base_footprint'),
                ('time_period', 0.03),
                ('left_wheel_variance_coef', 1e-6),
                ('right_wheel_variance_coef', 1e-6),
                ('publish_tf', False),
                ('slip_compensate', 1.0)
            ]
        )
        
        ltick_topic = self.get_parameter("lelf_wheel_tick_topic").value
        rtick_topic = self.get_parameter("right_wheel_tick_topic").value
        self.ticks_per_m = self.get_parameter("ticks_per_meter").value
        self.wheels_space = self.get_parameter("wheels_space").value
        self.odom_frame = self.get_parameter("odom_frame").value
        odom_topic = self.get_parameter("odom_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        time_period = self.get_parameter("time_period").value
        self.lwheel_var = self.get_parameter("left_wheel_variance_coef").value
        self.rwheel_var = self.get_parameter("right_wheel_variance_coef").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.slip_compensate = self.get_parameter("slip_compensate").value
        
        self.get_logger().info(
            "left_tick: %s, right_tick: %s, tick/m: %d, wheel space: %.4f, odom frame: %s, odom topic: %s, base frame: %s, period: %.3f, left wheel variance: %f, right wheel variance: %f, slip_compensate: %.3f"
            % (
                ltick_topic,
                rtick_topic,
                self.ticks_per_m,
                self.wheels_space,
                self.odom_frame,
                odom_topic,
                self.base_frame,
                time_period,
                self.lwheel_var,
                self.rwheel_var,
                self.slip_compensate
            ))
        
        self.timer = self.create_timer(time_period, self.timer_callback)
        
        self.lwheel_tick_sub = self.create_subscription(
                            Int16,
                            ltick_topic,
                            self.ltick_callback,
                            10)
        self.rwheel_tick_sub = self.create_subscription(
                            Int16,
                            rtick_topic,
                            self.rtick_callback,
                            10)
        
        self.odompub = self.create_publisher(Odometry, odom_topic, 1)
        
         # encoder specific
        self.encoder_min = -32768
        self.encoder_max = 32768
        self.lowrap = (self.encoder_max - self.encoder_min)*0.3 + self.encoder_min
        self.highwrap = (self.encoder_max - self.encoder_min)*0.7 + self.encoder_min
        self.n_left_wrap = 0.0
        self.n_right_wrap = 0.0
        self.left_travel = 0.0
        self.right_travel = 0.0
        self.last_left_travel = 0.0
        self.last_right_travel = 0.0
        self.last_ltick = 0.0
        self.last_rtick = 0.0
        # odometry data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lasttime = None
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.last_imu = None
        self.start_imu = None
        self.covariance_matrix = np.matrix( [[0,0,0], [0,0,0], [0,0,0]] )
        self.lasttime = self.get_clock().now()


    def ltick_callback(self, msg):
        data = msg.data
        # calculate the input
        if self.last_ltick > self.highwrap and data < self.lowrap:
            self.n_left_wrap = self.n_left_wrap + 1
        
        if self.last_ltick < self.lowrap and data > self.highwrap:
            self.n_left_wrap = self.n_left_wrap - 1
        
        self.left_travel = self.n_left_wrap*(self.encoder_max - self.encoder_min) + data
        if self.last_left_travel == 0:
            self.last_left_travel = self.left_travel
        self.last_ltick = data
        #rospy.loginfo("LEFT: %f ", self.left_travel)

    def rtick_callback(self, msg):
        data = msg.data
        # calculate the input
        if self.last_rtick > self.highwrap and data < self.lowrap:
            self.n_right_wrap = self.n_right_wrap + 1
        
        if self.last_rtick < self.lowrap and data > self.highwrap:
            self.n_right_wrap = self.n_right_wrap - 1

        self.right_travel = self.n_right_wrap*(self.encoder_max - self.encoder_min) + data
        if self.last_right_travel == 0:
            self.last_right_travel = self.right_travel
        self.last_rtick = data
        #rospy.loginfo("RIGHT: %f", self.right_travel)
    
    def timer_callback(self):
        self.update()
    

    def update(self):
        now = self.get_clock().now()
        duration = now - self.lasttime
        dt = duration.nanoseconds / 1e9
        dleft = 1.0*(self.left_travel - self.last_left_travel) / self.ticks_per_m
        dright = 1.0*(self.right_travel - self.last_right_travel) / self.ticks_per_m
        vx = 0
        va = 0
        self.lasttime = now
        self.last_left_travel = self.left_travel
        self.last_right_travel = self.right_travel

        dif = dright - dleft
        delta_s = (dleft + dright) / 2.0
        # error = pi / 180.0
        dtheta = (dif / self.wheels_space) * self.slip_compensate
        
       
        #now calculate the covariance
        wheel_covar = np.matrix([
            [ self.rwheel_var*abs(dright), 0],
            [0, self.lwheel_var*abs(dleft) ]
        ])
        pos_jacobian = np.matrix([
            [1, 0, -delta_s*sin(self.theta + dtheta / 2.0)],
            [0, 1, delta_s*cos(self.theta + dtheta / 2.0)],
            [0, 0 , 1]
        ]
        )
        wheel_jacobian = np.matrix([
            [cos(self.theta + dtheta /2.0)/2.0 - delta_s*sin(self.theta + dtheta/2.0)/ (2.0*self.wheels_space), cos(self.theta + dtheta/ 2.0)/2.0 + delta_s*sin(self.theta + dtheta/2.0)/ (2.0*self.wheels_space)],
            [sin(self.theta + dtheta /2.0)/2.0 + delta_s*cos(self.theta + dtheta/2.0)/ (2.0*self.wheels_space), sin(self.theta + dtheta/ 2.0)/2.0 - delta_s*cos(self.theta + dtheta/2.0)/ (2.0*self.wheels_space)],
            [1.0/ self.wheels_space, -1.0/ self.wheels_space]
        ])
        self.covariance_matrix = pos_jacobian*self.covariance_matrix* pos_jacobian.transpose() + wheel_jacobian*wheel_covar*wheel_jacobian.transpose()
        #print(self.covariance_matrix)

        # calculate the position
        # the simple way
        self.x = self.x + delta_s*cos(self.theta + dtheta / 2.0)
        self.y = self.y + delta_s*sin(self.theta + dtheta / 2.0)
        self.theta = self.theta + dtheta

        """
        if abs(dif) < 1e-6:
            self.x = self.x + delta_s*cos(self.theta)
            self.y = self.y + delta_s*sin(self.theta)
        else:
            new_theta = self.theta + dtheta * 0.75
            coef = self.wheels_space*delta_s / dif
            self.x = self.x  + coef*( sin(new_theta) - sin(self.theta) )
            self.y = self.y - coef*( cos(new_theta) - cos(self.theta) )
            self.theta = new_theta
        """
        vx = delta_s / dt
        va =  dtheta / dt
        quat = Quaternion()

    
        q =  self.euler_to_quaternion(0.0,0.0, self.theta)
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        # publish this information

        odom_msg = Odometry()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation =  quat
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = va


        # the covariance matrix
        odom_msg.pose.covariance[0] = self.covariance_matrix[0,0]
        odom_msg.pose.covariance[1] = self.covariance_matrix[0,1]
        odom_msg.pose.covariance[5] = self.covariance_matrix[0,2]
        odom_msg.pose.covariance[6] = self.covariance_matrix[1,0]
        odom_msg.pose.covariance[7] = self.covariance_matrix[1,1]
        odom_msg.pose.covariance[11] = self.covariance_matrix[1,2]
        odom_msg.pose.covariance[30] = self.covariance_matrix[2,0]
        odom_msg.pose.covariance[31] = self.covariance_matrix[2,1]
        odom_msg.pose.covariance[35] = self.covariance_matrix[2,2]

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = self.odom_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf.sendTransform(t)
        self.odompub.publish(odom_msg)

    def  euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)

    node = Ticks2Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()