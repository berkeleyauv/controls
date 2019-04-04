#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

'''
A python script to practice receiving ROS messages
'''

class Listener():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):

        # publishing objects
        self.chatter_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.chatter_callback)

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        print("Linear accel: {}".format(msg.linear_acceleration))
        print("Angular vel: {}".format(msg.angular_velocity))
        print("Orientation: {}".format(msg.orientation))


if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('imu_listener')
    l_obj = Listener()
    print("Listener node running")
    rospy.spin()
