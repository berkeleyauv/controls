#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from threading import Thread


class RazorIMU(Thread):

    def __init__(self, read_frequency=10, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.serial = serial.Serial('/dev/ttyACM0')
        self.pub = rospy.Publisher('/controls/razor', String, queue_size=10)
        ## TODO: Add more topics that we publish to individually
        self.rate = rospy.Rate(read_frequency) # 10hz
  
    def run(self):
        output = str(self.serial.readline())
        rospy.loginfo(output)
        self.pub.publish(output)


razor = RazorIMU(daemon=True)
razor.start()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.spin()
