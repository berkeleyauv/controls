import ControlMode
import getIMUData
import getYaw
import setRCOutput
import VelocityController
import HeadingController
import PositionController
import traceback
import sys


class Main:

    def __init__(self):
        rospy.init_node("SubInit")
        self.mode = ControlMode.sender
        self.out = setRCOutput.setMotor
        zero = [1500]*8
        self.mode.send('output')
        self.out.send(zero)

    def run(self, mode, output):
        self.mode.send(mode)
        if mode == 'output':
            self.out = setRCOutput.setMotor
            msg = output + [1500, 1500]
        elif mode == 'velocity':
            self.out = VelocityController.sender
            msg = output
        elif mode == 'position':
            self.out = PositionController.sender
            msg = output
        elif mode == 'heading':
            self.out = HeadingController.sender
            msg = output[0]
        else:
            return
        self.out.send(msg)
        

if __name__ == '__main__':
    try:
        m = Main()
        while not rospy.is_shutdown():
            inputString = input("Provide [mode] [output]").split()
            if inputString:
                mode = inputString[0]
                output = [int(item) for item in inputString[1:]]
                a.run(mode, output)
    except KeyboardInterrupt:
        print("Sub shutting down...")
    except Exception as e:
        traceback.print_exc()
    sys.exit(0)