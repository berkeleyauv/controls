#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from getIMUData import 
from PIDController import PID

'''
A python script to practice receiving ROS messages
'''

class HeadingController():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):

        self.chatter_sub = rospy.Subscriber("/control/heading", Float64, self.chatter_callback)
        self.pid = PID(0.5, 0.0, 0.0)

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        self.accelX.append(msg.linear_acceleration.x)
        self.accelY.append(msg.linear_acceleration.y)
        self.accelZ.append(msg.linear_acceleration.z)
        self.velX.append(integrate(accelX))
        self.velY.append(integrate(accelY))
        self.velZ.append(integrate(accelZ))
        print("Linear vel: {}".format((self.velX[-1], self.velY[-1], self.velZ[-1])))
        print("Position: {}".format(integrate(self.velX), integrate(self.velY), integrate(self.velZ)))
        print("Linear accel: {}".format(msg.linear_acceleration))
        print("Angular vel: {}".format(msg.angular_velocity))
        print("Orientation: {}".format(msg.orientation)) 

def integrate(arr):
    total = 0
    prevTime, prevVal = arr[0]
    for i in range(1, len(arr)):
        time, val = arr[i]
        total += (time-prevTime) * (prevVal + val) / 2
    return total

class SetHeading():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        # publishing objects
        self.chatter_pub = rospy.Publisher("/control/heading", Float64, queue_size=1)
        self.chat_frequency = rospy.Rate(chat_frequency)

    def send(self, msg):
        ''' Send messages on chatter topic at regular rate
            Vector3: 
                float x
                float y
                float z
        '''
        self.chatter_pub.publish(msg)


if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('HeadingController')
    controller = HeadingController()
    print("Heading controller node running")
    sender = SetHeading()
    rospy.spin()
