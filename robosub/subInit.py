#!/usr/bin/env python

from __future__ import print_function
import rospy

rospy.init_node("SubControls")
rospy.sleep(0.5)

import ControlMode
from getIMUData import imu
from getYaw import yawl
import setRCOutput
import VelocityController
import HeadingController
#import PositionController
import traceback
import sys

class Main:

    def __init__(self):
        self.mode = ControlMode.sender
        self.out = setRCOutput.setMotor
        zero = [1500]*8
        self.mode.send('power')
        self.out.send(zero)

    def run(self, option, output):
        code = self.mode.send(option)
        if option == 'power':
            self.out = setRCOutput.setMotor
            msg = [int(i) for i in output] + [1500, 1500]
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
            self.mode.send('output')
            self.out = setRCOutput.setMotor
            msg = [1500]*8
        else:
            # if code:
            #     print("Invalid option:", option)
            return
        self.out.send(msg)
        

if __name__ == '__main__':
    try:
        m = Main()
        print()
        print("Instructions:")
        print("Options are: power, velocity, heading, imu, yaw, arm, disarm, stop, stabilize, depth, manual")
        print("Outputs are needed for: power, velocity, and heading need to be in space delimited format")
        print("Keyboard interrupt(Ctrl+C) to exit")
        while not rospy.is_shutdown():
            try:
                inputString = raw_input("Provide [option] [output] \n").split()
                if inputString:
                    option = inputString[0]
                    output = [float(item) for item in inputString[1:]]
                    m.run(option, output)
                rospy.sleep(0.1)
            except Exception:
                traceback.print_exc()   
    except KeyboardInterrupt:
        print("Sub shutting down...")
        self.out = setRCOutput.setMotor
        self.out.stop()
    sys.exit(0)
