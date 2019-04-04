from setMode import SetControlMode
from setRCOutput import SetOutput
from setVelocity import SetVelocity
from setPosition import SetPosition
import rospy
import traceback
import sys


class Main:

    def __init__(self):
        rospy.init_node("SubInit")
        self.mode = SetControlMode()
        self.out = SetOutput()
        zero = [1500]*8
        self.mode.send('output')
        self.out.send(zero)

    def run(self, mode, output):
        self.mode.send(mode)
        if mode == 'output':
            self.out = SetOutput()
        elif mode == 'velocity':
            self.out = SetVelocity()
        elif mode == 'position':
            self.out = SetPosition()
        else:
            return
        self.out.send(msg)
        

if __name__ == '__main__':
    try:
        m = Main()
        while True:
            inputString = input("Provide [mode] [output]").split()
            mode = inputString[0]
            output = [int(item) for item in inputString[1:]]
            a.run(mode, output)
    except KeyboardInterrupt:
        print("Sub shutting down...")
    except Exception as e:
        traceback.print_exc()
    sys.exit(0)