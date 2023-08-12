
import rclpy
import adafruit_pca9685
import digitalio
import time
import busio
import board

from rclpy.node import Node

from std_msgs.msg import *
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('listener_diff_controll')
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.pca.frequency = 500
        
        self.pwm0 = self.pca.channels[0]
        self.pwm1 = self.pca.channels[1]

        self.a1 = digitalio.DigitalInOut(board.D17)
        self.a1.direction = digitalio.Direction.OUTPUT
        self.b1 = digitalio.DigitalInOut(board.D27)
        self.b1.direction = digitalio.Direction.OUTPUT
        self.a2 = digitalio.DigitalInOut(board.D10)
        self.a2.direction = digitalio.Direction.OUTPUT
        self.b2 = digitalio.DigitalInOut(board.D9)
        self.b2.direction = digitalio.Direction.OUTPUT
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        
    

    def listener_callback(self, msg):
        self.pwm0.duty_cycle = abs(int(65535*(msg.linear.x)))
        self.pwm1.duty_cycle = abs(int(65535*(msg.linear.x)))
        if(msg.linear.x >0.05):
            self.a1.value = True
            self.b1.value = False
            self.a2.value = True
            self.b2.value = False
        elif(msg.linear.x <0.05):
            self.a1.value = False
            self.b1.value = True
            self.a2.value = False
            self.b2.value = True
        else: 
            self.a1.value = False
            self.b1.value = False
            self.a2.value = False
            self.b2.value = False




def main(args=None):
    rclpy.init(args=args)

    listener_diff_controll = MinimalSubscriber()

    rclpy.spin(listener_diff_controll)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener_diff_controll.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
