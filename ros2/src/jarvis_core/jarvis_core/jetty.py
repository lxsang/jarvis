import rclpy
from rclpy.node import Node
from .common import JarvisSerial, CMDFrame, LogFrame, LOG_WARNING
from .imu import QuaternionFilter
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import Point
import math
import threading


class JettyNode(Node):
    """
    docstring
    """

    def __init__(self):
        super().__init__('jetty')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu_topic', "jetty_imu"),
                ('battery_topic', "jetty_battery"),
                ('left_tick_topic', "jetty_left_tick"),
                ('right_tick_topic', "jetty_right_tick"),
                ('left_motor_pwm_topic', "jetty_left_pwm"),
                ('right_motor_pwm_topic', "jetty_right_pwm"),
                ('time_period', 0.03),
            ]
        )
        imu_topic = self.get_parameter("imu_topic").value
        battery_topic = self.get_parameter("battery_topic").value
        left_tick_topic = self.get_parameter("left_tick_topic").value
        right_tick_topic = self.get_parameter("right_tick_topic").value
        self.left_motor_pwm_topic = self.get_parameter("left_motor_pwm_topic").value
        
        self.right_motor_pwm_topic = self.get_parameter("right_motor_pwm_topic").value
        
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        # publisher
        self.bat_pub = self.create_publisher(Float32, battery_topic, 10)
        self.ltick_pub = self.create_publisher(Int16, left_tick_topic, 10)
        self.rtick_pub = self.create_publisher(Int16, right_tick_topic, 10)
        

        timer_period = self.get_parameter("time_period").value
        self.imu = Imu()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.filter = QuaternionFilter(self)
        self.mag_bias = Point()
        self.mag_bias.x = -189.98
        self.mag_bias.y = -34.275
        self.mag_bias.z = 546.105

        self.mag_scale = Point()
        self.mag_scale.x = 1.04
        self.mag_scale.y = 1.03
        self.mag_scale.z = 0.935

        self.battery = Float32()
        self.left_tick = Int16()
        self.right_tick = Int16()
        self.synchronised = False

        self.cmd = CMDFrame()
        self.cmd.l_pwm = 0
        self.cmd.r_pwm = 0
        self.endpoint = JarvisSerial()
        self.has_data = False
        
        self.get_logger().info(
            "imu: %s, left_tick: %s, right_tick: %s, left_pwm: %s, right_pwm: %s, time_period: %.3f"
                %(  
                    imu_topic,
                    left_tick_topic,
                    right_tick_topic,
                    self.left_motor_pwm_topic,
                    self.right_motor_pwm_topic,
                    timer_period))

    def rpwm_listener_callback(self, msg):
        self.cmd.r_pwm = msg.data
        if self.synchronised:
            self.endpoint.send_frame(self.cmd)

    def lpwm_listener_callback(self, msg):
        # set log level
        # log_setting = LogFrame()
        # log_setting.level = LOG_WARNING
        # self.endpoint.send_frame(log_setting)
        self.cmd.l_pwm = msg.data
        if self.synchronised:
            self.endpoint.send_frame(self.cmd)

    def get_sensors_data(self):
        while rclpy.ok():
            frame = self.endpoint.read_frame()
            if frame != None:
                if frame.type == 0:
                    if not self.synchronised:
                        self.synchronised = True
                        # create subscriber
                        self.lpwm_sub = self.create_subscription(
                            Int16,
                            self.left_motor_pwm_topic,
                            self.lpwm_listener_callback,
                            10)
                
                        self.lpwm_sub = self.create_subscription(
                            Int16,
                            self.right_motor_pwm_topic,
                            self.rpwm_listener_callback,
                            10)
                    self.update(frame)
                else:
                    frame.log(self.get_logger())

    def update(self, frame):
        self.has_data = True
        self.battery.data = frame.battery
        self.left_tick.data = frame.left_tick
        self.right_tick.data = frame.right_tick

        gx = frame.gyro.x * (math.pi/180.0)
        gy = frame.gyro.y * (math.pi/180.0)
        gz = frame.gyro.z * (math.pi/180.0)

        ax = frame.accel.x
        ay = frame.accel.y
        az = frame.accel.z

        mx = (frame.mag.x - self.mag_bias.x)*self.mag_scale.x
        my = (frame.mag.y - self.mag_bias.y)*self.mag_scale.y
        # z point down, now we need to make it up
        mz = (frame.mag.y - self.mag_bias.z)*self.mag_scale.z

        # update the filter
        # Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
        # the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
        # We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
        # For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
        # in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
        # This is ok by aircraft orientation standards!
        # Pass gyro rate as rad/s
        self.filter.update(ax,  ay,  az,  gx,  gy,  gz,  my,  mx,  mz)

        self.imu = Imu()
        self.imu.orientation.w = self.filter.quaternion[0]
        self.imu.orientation.x = self.filter.quaternion[1]
        self.imu.orientation.y = self.filter.quaternion[2]
        self.imu.orientation.z = self.filter.quaternion[3]
        # angular velocity
        self.imu.angular_velocity.x = gx
        self.imu.angular_velocity.y = gy
        self.imu.angular_velocity.z = gz

        self.imu.linear_acceleration.x = ax * 9.81
        self.imu.linear_acceleration.y = ay * 9.81
        self.imu.linear_acceleration.z = az * 9.81

        self.imu.header.stamp = self.get_clock().now().to_msg()
        self.imu.header.frame_id = "imu_link"
        # covariance
        self.imu.orientation_covariance[0] = 1e-6
        self.imu.orientation_covariance[4] = 1e-6
        self.imu.orientation_covariance[8] = 1e-6
        self.imu.angular_velocity_covariance[0] = 0.015
        self.imu.angular_velocity_covariance[4] = 0.015
        self.imu.angular_velocity_covariance[8] = 0.015
        self.imu.linear_acceleration_covariance[0] = 0.03
        self.imu.linear_acceleration_covariance[4] = 0.03
        self.imu.linear_acceleration_covariance[8] = 0.03

    def timer_callback(self):
        if self.has_data:
            self.imu_pub.publish(self.imu)
            self.bat_pub.publish(self.battery)
            self.ltick_pub.publish(self.left_tick)
            self.rtick_pub.publish(self.right_tick)
            self.has_data = False
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = JettyNode()
    thread = threading.Thread(target=node.get_sensors_data)
    try:
        thread.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
