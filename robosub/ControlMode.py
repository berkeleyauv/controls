#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu
import HeadingController
import VelocityController
import PositionController
import setRCOutput


'''
A python script to practice receiving ROS messages
'''

class ControlMode():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):

        # publishing objects
        self.chatter_sub = rospy.Subscriber("/control/mode", String, self.chatter_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode = 'output'

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        if self.mode == msg.data:
            return
        self.mode = msg.data
        self.out.stop()
        print("Switching to mode: {}".format(self.mode))
        if self.mode == 'output':
            self.out = setRCOutput.setMotor
            zero = [1500]*8
        elif self.mode == 'velocity':
            self.out = VelocityController.sender
            msg = (0.0,0.0,0.0)
        elif self.mode == 'position':
            self.out = PositionController.sender
            msg = (0.0,0.0,0.0)
        elif self.mode == 'heading':
            self.out = HeadingController.sender
            msg = 0.0
        elif self.mode == 'disarm':
            self.arming(False)
            return
        else:
            return
        self.out.send(msg)  

class SetControlMode():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        # publishing objects
        self.chatter_pub = rospy.Publisher("/control/mode", String, queue_size=1)
        # rate of publishing
        self.chat_frequency = rospy.Rate(chat_frequency)

    def send(self, msg):
        ''' Send messages on chatter topic at regular rate
            String: 
                string data
        '''
        self.chatter_pub.publish(msg)
        
      
rospy.init_node('ControlMode')
mode = ControlMode()
print("ControlMode listener node running")
sender = SetControlMode()
print("ControlMode publisher running")

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''

    msg = String('output')
    sender.send(msg)
    rospy.spin()
