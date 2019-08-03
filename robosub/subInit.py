#!/usr/bin/env python

from __future__ import print_function
import sys
import time

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'comp':
        import os
        os.spawnl(os.P_NOWAIT, 'roscore')
        time.sleep(3)
        os.spawnl(os.P_NOWAIT, 'roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp://@192.168.137.1:14550')
        time.sleep(3)

import rospy

rospy.init_node("SubControls")
rospy.sleep(0.5)

import ControlMode
from getIMUData import imu, writeVideo, releaseVideo
from getYaw import yawl
import setRCOutput
import VelocityController
import HeadingController
#import PositionController
import traceback
import sys
import time
from datetime import datetime 
import cv2

MODES = ['power', 'velocity', 'heading', 'disarm', 'arm', 'stabilize', 'manual', 'depth']


START_DELAY = 10
RUN_TIME = 3


class Main:

    def __init__(self):
        self.mode = ControlMode.sender
        self.out = setRCOutput.setMotor
        zero = [1500]*8
        self.mode.send('power')
        self.out.send(zero)

    def run(self, option, output):
        if option in MODES:
            code = self.mode.send(option)
        if option in ['power', 'stabilize', 'depth']:
            self.out = setRCOutput.setMotor
            msg = [int(i) for i in output] + [1500, 1500]
        elif option == 'vision':            
           return
        elif option == 'velocity':
            self.out = VelocityController.sender
            msg = output[:3]
        # elif mode == 'position':
        #     self.out = PositionController.sender
        #     msg = output
        elif option == 'heading':
            self.out = HeadingController.sender
            msg = output[0]
        elif option == 'imu':
            print("Linear accel: {}".format(imu.linear_acceleration))
            print("Angular vel: {}".format(imu.angular_velocity))
            print("Orientation: {}".format(imu.orientation))
            print("Linear vel: {}".format((imu.velX[-1], imu.velY[-1], imu.velZ[-1])))
            return
        elif option == 'yaw':
            print("Current yaw:", yawl.yaw)
            return
        elif option == 'stop':
            self.mode.send('power')
            self.out = setRCOutput.setMotor
            msg = [1500]*8
        else:
            # if code:
            #     print("Invalid option:", option)
            return
        self.out.send(msg)

def processInput():
    try:
        m = Main()
        print()
        print("Instructions:")
        print("Options are: power, velocity, heading, imu, yaw, arm, disarm, stop, stabilize, depth, manual")
        print("Outputs are needed for: power, velocity, and heading. Need to be in space delimited format")
        print("Keyboard interrupt(Ctrl+C) to exit")
        while not rospy.is_shutdown():
            try:
                inputString = raw_input("Provide [option] [output] \n").split()
                if inputString == 'exit':
                    raise KeyboardInterrupt()
                if inputString:
                    option = inputString[0]
                    output = [float(item) for item in inputString[1:]]
                    m.run(option, output)
                rospy.sleep(0.1)
            except Exception as e:
                print("Error occurred: " + str(e))
                traceback.print_exc()
    except KeyboardInterrupt:
        print("Sub shutting down...")
    finally:
        m.out = setRCOutput.setMotor
        m.out.stop()
        m.mode.send('disarm')

if __name__ == '__main__':
    if len(sys.argv) == 1:
        processInput()
    elif sys.argv[1] == 'auto' or sys.argv[1] == 'comp':
        try:
            start_time = time.time()
            # while ((time.time() - start_time) < START_DELAY):
            #     writeVideo()
            main = Main()
            main.mode.send('arm')
            time.sleep(1)
            main.mode.send('power')
            start_time = time.time()
            while ((time.time() - start_time) < RUN_TIME*2.5):
                main.out.send([1500,1500,1500,1500,1600,1500,1500,1500])
                writeVideo()
            start_time = time.time()
            while ((time.time() - start_time) < RUN_TIME):
                main.out.send([1500,1500,1500,1500,1500,1600,1500,1500])
                writeVideo()
            start_time = time.time()
            while ((time.time() - start_time) < RUN_TIME*2.5):
                main.out.send([1500,1500,1500,1500,1400,1500,1500,1500])
                writeVideo()
            while ((time.time() - start_time) < RUN_TIME):
                main.out.send([1500,1500,1500,1500,1500,1400,1500,1500])
                writeVideo()
            # Release everything if job is finished
            main.out.send([1500,1500,1500,1500,1500,1500,1500,1500])
        except KeyboardInterrupt as i:
            pass
        finally:
            releaseVideo()
