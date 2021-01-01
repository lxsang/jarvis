import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
import adafruit_ssd1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from board import SCL, SDA
import subprocess
import busio


class Display:
    def __init__(self):
        # Create the I2C interface.
        i2c = busio.I2C(SCL, SDA)
        self.disp = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)
        # self.disp.begin()
        # Clear display.
        self.disp.fill(0)
        self.disp.show()
        # Clear display.
        # self.disp.clear()
        # self.disp.display()
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))
        # Get drawing object to draw on image.
        self.drawobj = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default()
        self.x = 0
        self.top = -2
        self.bottom = self.height - self.top

    def clear(self):
        self.drawobj.rectangle(
            (0, 0, self.width, self.height), outline=0, fill=0)

    def log(self, pos, text):
        self.drawobj.text(
            (self.x + pos[0], self.top + pos[1]), text, font=self.font, fill=255)

    def show(self):
        self.disp.image(self.image.rotate(180))
        self.disp.show()


class JarvisStat(Node):
    def __init__(self):
        super().__init__('stat')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bat_min_voltage', 3300.0),
                ('bat_max_voltage', 3400.0),
                ('battery_topic', "jarvis_battery"),
                ('time_period', 0.5),
            ]
        )
        timer_period = self.get_parameter("time_period").value
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bat_sub = self.create_subscription(
            Float32,
            self.get_parameter("battery_topic").value,
            self.bat_callback,
            10)

        self.bat_min_voltage = self.get_parameter("bat_min_voltage").value
        self.bat_max_voltage = self.get_parameter("bat_max_voltage").value
        self.is_online = False
        self.bat_safe_zone = self.bat_max_voltage - self.bat_min_voltage
        # get my IP address
        cmd = "hostname -I | cut -d\' \' -f1"
        self.ip = subprocess.check_output(cmd, shell=True)
        self.display = Display()
        self.display.clear()

        self.voltage = 0.0

    def timer_callback(self):
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        CPU = subprocess.check_output(cmd, shell=True)
        cmd = "free -m | awk 'NR==2{printf \"M.:%s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
        MemUsage = subprocess.check_output(cmd, shell=True)
        cmd = "df -h | awk '$NF==\"/\"{printf \"Disk:%d/%dGB %s\", $3,$2,$5}'"
        Disk = subprocess.check_output(cmd, shell=True)
        cmd = "cat /sys/class/thermal/thermal_zone0/temp"
        cpu_temp = int(subprocess.check_output(cmd, shell=True)) / 1000
        try:
            self.display.clear()

            self.display.log((0, 0),       "IP: " + self.ip.decode("utf-8"))
            self.display.log((0, 9),     CPU.decode("utf-8"))
            self.display.log((0, 18),    MemUsage.decode("utf-8"))
            self.display.log((0, 27),    Disk.decode("utf-8"))
            self.display.log((0, 36),    "CPU temp.(C): " + str(cpu_temp))
            self.display.log((0, 45), "Battery: " +
                             "{:.2f}".format(self.map(self.voltage)) + "%")

            # Display image.
            self.display.show()

        except OSError:
            pass

        """ file = "/var/wfifo_notification"
        if os.path.exists(file):
            # write data to notification fifo
            try:
                json = "{\"ip\":\"" + self.ip.decode("utf-8").strip() +"\","
                json += "\"mem\":\"" + MemUsage.decode("utf-8").strip() + "\","
                json += "\"cpu\":\"" + CPU.decode("utf-8").strip() + "\","
                json += "\"disk\":\"" + Disk.decode("utf-8").strip() + "\","
                json += "\"battery\":" + str(self.voltage) + ","
                json += "\"battery_percent\":" + str(self.map(self.voltage)) + ","
                json += "\"temp\":" + str(cpu_temp) + "}"
                with open(file,"w") as fp:
                    fp.write(json)
            except IOError:
                pass
        
        #run control command
        file = "/var/jarvis_control"
        if os.path.exists(file):
            fifo = os.open(file, os.O_RDONLY | os.O_NONBLOCK)
            try:
                buffer = os.read(fifo, 255)
            except OSError as err:
                #if err.errno != errno.EAGAIN and err.errno != errno.EWOULDBLOCK:
                buffer = None
            
            if buffer is not None: 
                os.system(buffer.decode("utf-8"))
            os.close(fifo) """

    def map(self, volt):
        if(volt < self.bat_min_voltage):
            return 0
        result = 101 - (101 / pow(1 + pow(1.33 * (volt - self.bat_min_voltage) /
                                          (self.bat_max_voltage - self.bat_min_voltage), 4.5), 3))
        if result > 100:
            return 100
        return result

    def bat_callback(self, msg):
        self.voltage = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = JarvisStat()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
