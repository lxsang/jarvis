# code inspired and modified from
# https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Imu
"""
PyTeapot module for drawing rotating cube using OpenGL as per
quaternion or yaw, pitch, roll angles received over serial port.

Dependencies:
 - pygame: pip3 install pygame
 - openGL: pip install PyOpenGL PyOpenGL_accelerate
"""
class IMUViewer(Node):
    def __init__(self):
        super().__init__('imu_viewer')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu_topic', '/jarvis/imu')
            ]
        )
        self.sub = self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.callback,
            10)
        video_flags = OPENGL | DOUBLEBUF
        pygame.init()
        screen = pygame.display.set_mode((640, 480), video_flags)
        pygame.display.set_caption("IMU orientation visualization")
        self.resizewin(640, 480)
        self.setup()
        self.running = True
        self.imu = Imu()

    def callback(self, msg):
        self.imu = msg

    def resizewin(self, width, height):
        """
        For resizing window
        """
        if height == 0:
            height = 1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0*width/height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()


    def setup(self):
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


    def read_data(self):
        w  = self.imu.orientation.w
        nx = self.imu.orientation.x
        ny = self.imu.orientation.y
        nz = self.imu.orientation.z
        return [w, nx, ny, nz]


    def draw(self,w, nx, ny, nz):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        glTranslatef(0, 0.0, -7.0)

        self.drawText((-2.6, 1.8, 2), "PyTeapot", 18)
        self.drawText((-2.6, 1.6, 2), "Module to visualize quaternion", 16)
        self.drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

        [yaw, pitch , roll] = self.quat_to_ypr([w, nx, ny, nz])
        self.drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        
        glRotatef(2 * math.acos(w) * 180.00/math.pi,  nx, nz, -1*ny)
        glRotated( -90, 1.0, 0.0, 0.0 )
        #glRotatef(yaw, 1.0, 0.0, 0.0)
        #glRotatef(pitch, 0.0, 1.0, 0.0)
        #glRotatef(roll, 0.0, 0.0, 1.0)
        # move the axes to the screen corner
        #glTranslatef(-3.0, -2.0, 0.0)
        # draw our axes
        glLineWidth( 3.0 )
        glBegin(GL_LINES)
        # draw line for x axis
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(2.0, 0.0, 0.0)
        # draw line for y axis
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 2.0, 0.0)
        # draw line for Z axis
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 2.0)
        glEnd()
        
        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd() 

        # save previous matrix
        #glPushMatrix()
        # clear matrix
        # glLoadIdentity()
        # apply rotations
        # ROT
        # load the previous matrix
        # glPopMatrix()

    def drawText(self,position, textString, size):
        font = pygame.font.SysFont("Courier", size, True)
        textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
        textData = pygame.image.tostring(textSurface, "RGBA", True)
        glRasterPos3d(*position)
        glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

    def quat_to_ypr(self,q):
        yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
        pitch = -math.sin(2.0 * (q[1] * q[3] - q[0] * q[2]))
        roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
        pitch *= 180.0 / math.pi
        yaw   *= 180.0 / math.pi
        yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
        roll  *= 180.0 / math.pi
        return [yaw, pitch, roll]
    
    def step(self):
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            self.running = False
        
        [w, nx, ny, nz] = self.read_data()
        self.draw(w, nx, ny, nz)
        pygame.display.flip()


def main(args=None):
    rclpy.init(args=args)
    
    node = IMUViewer()
    while rclpy.ok() and node.running:
        rclpy.spin_once(node)
        node.step()
    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()