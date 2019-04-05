#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

'''
A python script to practice receiving ROS messages
'''

class IMUListener():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):

        # publishing objects
        self.chatter_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.chatter_callback)
        self.accelX = []
        self.accelY = []
        self.accelZ = []
        self.velX = []
        self.velY = []
        self.velZ = []

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        self.accelX.append(msg.linear_acceleration.x)
        self.accelY.append(msg.linear_acceleration.y)
        self.accelZ.append(msg.linear_acceleration.z)
        self.velX.append(integrate(accelX))
        self.velY.append(integrate(accelY))
        self.velZ.append(integrate(accelZ))
        self.position = (integrate(self.velX), integrate(self.velY), integrate(self.velZ))
        print("Linear vel: {}".format((self.velX[-1], self.velY[-1], self.velZ[-1])))
        print("Position: {}".format(self.position))
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

rospy.init_node('imu_listener')
imu = IMUListener()
print("IMU Listener node running")

if __name__ == '__main__':
    rospy.spin()
