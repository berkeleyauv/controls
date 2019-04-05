#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import ManualControl, OverrideRCIn

'''
A python script to practice sending ROS messages
'''

class SetOutput():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        # publishing objects
        self.chatter_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
        #self.chatter_pub = rospy.Publisher("/mavros/manual_control/send", ManualControl, queue_size=1)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        # rate of publishing
        self.chat_frequency = rospy.Rate(chat_frequency)

    def send(self, msg):
        ''' Send messages on chatter topic at regular rate
            int[8]:
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
        '''
        # i = 0
        # while (not rospy.is_shutdown()):
        #     i = i + 1
            
        #     #msg.header.seq = i
        #     #msg.header.stamp = rospy.Time.now()
        #     self.chatter_pub.publish(msg)
        #     self.chat_frequency.sleep()
        msg = OverrideRCIn(msg)
        self.mode(0, "MANUAL")
        self.arming(True)
        self.chatter_pub.publish(msg)

rospy.init_node('RC_override')
setMotor = SetOutput()
print("SetMotor node running")

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
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

    channels[0]  =  1500 # Pitch, won't work since no pitch control
    channels[1]  =  1500 # Roll
    channels[2]  =  1500 # Throttle
    channels[3]  =  1500 # Yaw
    channels[4]  =  1500 # Forward
    channels[5]  =  1500 # Lateral

    #msg = OverrideRCIn(channels)

    # start the chatter
    setMotor.send(channels)
