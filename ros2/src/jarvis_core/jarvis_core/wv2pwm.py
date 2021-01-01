import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from numpy import array

class PID:
    def __init__(self,kp, ki, kd,node):
        self.tune(kp,ki,kd)
        self.node = node
        self.input = 0.0
        self.setpoint = 0.0
        self.output = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.max_output = 200
        self.min_output = -200
        self.lasttime = self.node.get_clock().now()


    def reset(self):
        self.last_error = 0
        self.integral = 0

    def compute(self):
        now = self.node.get_clock().now()
        delta = now - self.lasttime
        self.lasttime = self.node.get_clock().now()
        dt = delta.nanoseconds / 1e9
        error = self.setpoint - self.input
        self.integral = self.integral + (error*dt)
        de = 1.0*(error - self.last_error)/dt
        self.output = self.kp*error + self.ki*self.integral + self.kd*de
        self.last_error = error
        if self.output > self.max_output:
            self.output  = self.max_output
            self.integral = self.integral - (error*dt)
        if self.output  < self.min_output:
            self.output  = self.min_output
            self.integral = self.integral - (error*dt)
    
    def tune(self, kp,ki,kd):
        self.ki = ki
        self.kd = kd
        self.kp = kp

class WV2PWM(Node):
    def __init__(self):
        super().__init__('wv2pwm')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 10),
                ('ki', 10),
                ('kd', 1e-4),
                ('ticks_per_meter', 4495),
                ('time_period', 0.02),
                ('wheel_vel_setpoint_topic', 'wheel_vel_setpoint'),
                ('wheel_tick_topic', 'wheel_tick'),
                ('motor_pwm_topic', 'motor_pwm'),
                ('real_wheel_vel_topic', 'wheel_vel')
            ]
        )
        
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.get_logger().info("Kp: %f, Kd: %f, Ki: %f"%(kp, ki, kd))

        # encoder specific
        self.encoder_min = -32768
        self.encoder_max = 32768
        self.lowrap = (self.encoder_max - self.encoder_min)*0.3 + self.encoder_min
        self.highwrap = (self.encoder_max - self.encoder_min)*0.7 + self.encoder_min
        self.nwrap = 0
        self.last_tick = 0
        self.wheel_travel = 0
        self.last_wheel_travel = 0
        self.laspwm = 0
        #self.arr = [0.0]*2
        #self.max_velocity = 

        self.pid = PID(kp, ki, kd, self)
        
        time_period = self.get_parameter("time_period").value
        
        self.timer = self.create_timer(time_period, self.timer_callback)
        
        self.ticks_per_meter = self.get_parameter("ticks_per_meter").value
        
        wheel_vel_topic = self.get_parameter("wheel_vel_setpoint_topic").value
        
        self.wheel_vel_sub = self.create_subscription(
            Float32,
            wheel_vel_topic,
            self.wheel_vel_callback,
            10)

        wheel_tick_topic = self.get_parameter("wheel_tick_topic").value
        
        self.wheel_tick_sub = self.create_subscription(
            Int16,
            wheel_tick_topic,
            self.wheel_tick_callback,
            10)
            

        motor_topic = self.get_parameter("motor_pwm_topic").value
        
        
        self.motor_pub =  self.create_publisher(Int16, motor_topic, 1)

        output_vel_topic = self.get_parameter("real_wheel_vel_topic").value
        
        self.wheel_vel_pub =  self.create_publisher(Float32, output_vel_topic, 1)
        self.pid.lasttime = self.get_clock().now()
        self.lasttime = self.get_clock().now()
        self.get_logger().info("wheel_vel: %s, wheel_tick: %s, motor: %s, period: %.3f, ticks/m: %d, output_vel_topic: %s"%( wheel_vel_topic, wheel_tick_topic, motor_topic, time_period, self.ticks_per_meter, output_vel_topic))
    
    def wheel_vel_callback(self, msg):
        vel = msg.data
        #if msg.data > self.max_velocity:
        #    vel = self.max_velocity
        self.pid.setpoint = vel
        if vel == 0:
            self.pid.reset()
        
    
    def wheel_tick_callback(self, msg):
        # calculate the input
        if self.last_tick > self.highwrap and msg.data < self.lowrap:
            self.nwrap = self.nwrap + 1
        
        if self.last_tick < self.lowrap and msg.data > self.highwrap:
            self.nwrap = self.nwrap - 1
        
        self.total_tick = self.nwrap*(self.encoder_max - self.encoder_min) + msg.data
        self.wheel_travel = (1.0* self.total_tick / self.ticks_per_meter) * 1000.0
        if self.last_wheel_travel == 0:
            self.last_wheel_travel = self.wheel_travel
        self.last_tick = msg.data
    
    def calculate_velocity(self):
        now = self.get_clock().now()
        delta = now - self.lasttime
        dt = delta.nanoseconds / 1e9
        self.lasttime = self.get_clock().now()
        if self.last_wheel_travel != self.wheel_travel:
        #     curr_vel = (1.0/self.ticks_per_meter)/dt
        #     if abs(curr_vel) < 1e-4:
        #         self.arr.append(0)
        #         del self.arr[0]
        #     elif abs(curr_vel*1000.0 < self.pid.input):
        #         self.arr.append(curr_vel*1000.0)
        #         del self.arr[0]
        # else:
            self.pid.input = ((self.wheel_travel - self.last_wheel_travel) / dt) 

            self.last_wheel_travel = self.wheel_travel
            #del self.arr[0]
        else:
            if self.pid.setpoint == 0:
                #self.arr.append(0)
                #del self.arr[0]
                self.pid.input = 0
        #self.pid.input = array(self.arr).mean()

    
    def timer_callback(self):
        self.loopOnce()
        
    
    def loopOnce(self):
        #hz = rospy.Rate(self.rate)
        int_msg = Int16()
        float_msg = Float32()
        self.calculate_velocity()
        self.pid.compute()
        pwm = round(self.pid.output)
        #delta = pwm - self.laspwm
        #change too quick
        #if abs(delta) > self.pid.max_output / 2:
        #    pwm = self.laspwm + delta / 3
        if self.laspwm != pwm:
            int_msg.data = pwm
            self.motor_pub.publish(int_msg)
            float_msg.data = self.pid.input*1.0
            self.wheel_vel_pub.publish(float_msg)
        self.laspwm = pwm
        #rospy.loginfo("PWM : %d", pwm)

def main(args=None):
    rclpy.init(args=args)

    node = WV2PWM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()