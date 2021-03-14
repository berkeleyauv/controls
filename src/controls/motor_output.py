#!/usr/bin/env python3

import sys

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import ManualControl, OverrideRCIn


class SetMotors(rclpy.Node):
    ''' Generates and publishes ROS messages
    '''
    def __init__(self):
        self.pub = self.create_publisher("/sub/cmd_vel", Twist, 10)

    def send(self, msg):
        """
        Assuming that 
            forward is +y
            lateral right is +x
            depth up is +z
        """
        self.pub.publish(Twist(msg))

    def stop(self):
        self.send(Twist())


if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rclpy.init(sys.argv)
    setMotor = SetMotors()
    rclpy.spin(setMotor)
    rclpy.shutdown()

