#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyACM0')

pub = rospy.Publisher('razor-imu', String, queue_size=10)

def imu():
  rospy.init_node('razor-imu', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    output = str(ser.readline())
    rospy.loginfo(output)
    pub.publish(output)
    rate.sleep()

if __name__ == '__main__':
  try:
    imu()
  except rospy.ROSInterruptException:
    pass
