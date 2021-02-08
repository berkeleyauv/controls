import rclpy
from rclpy.node import Node

import sys

import tf_quaternion.transformations as transf

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class PublishPose(Node):
    ''' Generates and publishes ROS messages
    '''
    def __init__(self):
        super().__init__("pose_publisher")
        self.pub = self.create_publisher(PoseStamped, "/sub/cmd_pose", 10)

    def send(self, msg):
        """
        Assuming that 
            forward is +y
            lateral right is +x
            depth up is +z
        """
        data = PoseStamped()
        data.header.stamp = self.get_clock().now().to_msg()
        data.pose.position = Point(x=msg[0], y=msg[1], z=msg[2])
        ori = transf.quaternion_from_euler(msg[3], msg[4], msg[5], 'sxyz')
        data.pose.orientation = Quaternion(x=ori[0], y = ori[1], z=ori[2], w=ori[3])
        self.pub.publish(data)

    def stop(self):
        self.send(PoseStamped())


if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rclpy.init(args=sys.argv)
    publisher = PublishPose()
    while rclpy.ok():
        pose = input("Enter 6 values for pose: ")
        pose = [float(val) for val in pose.split(' ')]
        print(pose)
        publisher.send(pose)
    rclpy.shutdown()
