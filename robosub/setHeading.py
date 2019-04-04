#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import ManualControl, OverrideRCIn

'''
A python script to practice sending ROS messages
'''

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
        #self.chat_frequency.sleep()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('SetVelocity')
    td = SetVelocity()
    print("Velocity setter running")
    channels = [1500]*8
    
    # msg = ManualControl()
    # msg.header.frame_id = 'Manual control'
    # msg.x = 100
    # msg.y = 0
    # msg.z = 0
    # msg.r = 0
    # msg.buttons = 0

    """
        Channel	Meaning
    1	    Pitch
    2       Roll
    3	    Throttle
    4	    Yaw
    5	    Forward
    6	    Lateral
    7	    Reserved
    8	    Camera Tilt
    9	    Lights 1 Level
    10	    Lights 2 Level
    """

    channels[0]  =  1500 # Pitch
    channels[1]  =  1500 # Roll
    channels[2]  =  1500 # Throttle
    channels[3]  =  1500 # Yaw
    channels[4]  =  1500 # Forward
    channels[5]  =  1500 # Lateral

    msg = OverrideRCIn(channels)

    # start the chatter
    td.send(msg)
