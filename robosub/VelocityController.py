#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from getIMUData import imu
from PIDController import PID
import sys
from threading import Thread

'''
A python script to practice receiving ROS messages
'''

class VelocityController():
    ''' Subscribes to ROS messages
    '''
    def __init__(self, axis):

        self.chatter_sub = rospy.Subscriber("/control/velocity", Vector3, self.chatter_callback)
        self.forward = PID(0.5, 0.0, 0.0)
        self.lateral = PID(0.5, 0.0, 0.0)
        self.thrust = PID(0.5, 0.0, 0.0)

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        if self.thread:
            self.thread.stop()
        self.forward.setSetpoint(msg.x)
        self.thrust.setSetpoint(msg.y)
        self.lateral.setSetpoint(msg.z)
        self.thread = PIDThread()
        self.thread.start(forward, lateral, thrust)

    def stop(self):
        if self.thread:
            self.thread.stop()

    class PIDThread(Thread):

        def start(self, forward, lateral, thrust):
            self.stop = False
            self.rate = rospy.Rate(50)
            self.forward = forward
            self.lateral = lateral
            self.thrust = thrust
            self.imu = imu
            Thread.start(self)
        
        def stop(self):
            self.stop = True

        def run(self):
            self.forward.reset()
            self.lateral.reset()
            self.thrust.reset()
            start = time.time()
            while not self.stop:
                x = self.forward.pidLoop(self.imu.velX[-1], time.time()-start)
                y = self.thrust.pidLoop(self.imu.velY[-1], time.time()-start)
                z = self.lateral.pidLoop(self.imu.velZ[-1]. time.time()-start)
                msg = [1500]*8
                msg[2] += y
                msg[4] += x
                msg[5] += z
                setMotor.send(msg)
                self.rate.sleep()

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
        msg = Vector3(*msg)
        self.chatter_pub.publish(msg)

rospy.init_node('VelocityController')
velControl = VelocityController()
print("Velocity controller node running")
sender = SetVelocity()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    print(sys.argv)
    msg = Vector3(0,0,0)
    sender.send(msg)
    rospy.spin()