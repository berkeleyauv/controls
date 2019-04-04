#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from getIMUData import 
from PIDController import PID
import sys

'''
A python script to practice receiving ROS messages
'''

class VelocityController():
    ''' Subscribes to ROS messages
    '''
    def __init__(self, axis):

        self.chatter_sub = rospy.Subscriber("/control/velocity", Vector3, self.chatter_callback)
        self.axis = axis
        self.output = 
        self.accelX = []
        self.accelY = []
        self.accelZ = []
        self.velX = []
        self.velY = []
        self.velZ = []
        self.forward = PID(0.5, 0.0, 0.0)
        self.lateral = PID(0.5, 0.0, 0.0)
        self.thrust = PID(0.5, 0.0, 0.0)

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
        self.forward.setSetpoint(msg.x)
        self.thrust.setSetpoint(msg.y)
        self.lateral.setSetpoint(msg.z)

def integrate(arr):
    total = 0
    prevTime, prevVal = arr[0]
    for i in range(1, len(arr)):
        time, val = arr[i]
        total += (time-prevTime) * (prevVal + val) / 2
    return total

class SetVelocity():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        # publishing objects
        self.chatter_pub = rospy.Publisher("/control/velocity", Vector3, queue_size=1)
        self.chat_frequency = rospy.Rate(chat_frequency)

    def send(self, msg):
        ''' Send messages on chatter topic at regular rate
            Vector3: 
                float x
                float y
                float z
        '''
        self.chatter_pub.publish(msg)

rospy.init_node('VelocityController')
velControl = VelocityController()
print("Velocity controller node running")
rospy.spin()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    print(sys.argv)
    sender = SetVelocity()
    msg = Vector3(0,0,0)
    sender.send(msg)