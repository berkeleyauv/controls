import math
import numpy
import rclpy

from rcl_interfaces.msg import ParameterDescriptor

from uuv_PID import PIDRegulator

import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import tf_quaternion.transformations as transf

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.node import Node

from plankton_utils.time import time_in_float_sec_from_msg
from plankton_utils.time import is_sim_time

class OrientationControllerNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.get_logger().info('OrientationControllerNode: initializing node')

        self.config = {}

        self.quat_des = numpy.array([0, 0, 0, 1])

        self.initialized = False

        # Initialize pids with default parameters
        self.pid_rot = PIDRegulator(1, 0, 0, 1)

        self._declare_and_fill_map("rot_p", 1., "p component of pid for orientation.", self.config)
        self._declare_and_fill_map("rot_i", 0.0, "i component of pid for orientation.", self.config)
        self._declare_and_fill_map("rot_d", 0.0, "d component of pid for orientation.", self.config)
        self._declare_and_fill_map("rot_sat", 3.0, "saturation of pid for orientation.", self.config)

        self.set_parameters_callback(self.callback_params)

        self.create_pids(self.config)

        # ROS infrastructure
        self.sub_cmd_pose = self.create_subscription(geometry_msgs.PoseStamped, 'cmd_pose', self.cmd_pose_callback, 10)
        self.sub_odometry = self.create_subscription(Imu, 'imu', self.odometry_callback, 10)
        self.pub_cmd_vel = self.create_publisher(geometry_msgs.Twist, 'cmd_vel', 10)        

    #==============================================================================
    def cmd_pose_callback(self, msg):
        """Handle updated set pose callback."""
        # Just store the desired pose. The actual control runs on odometry callbacks
        q = msg.pose.orientation
        self.quat_des = numpy.array([q.x, q.y, q.z, q.w])
        print("orientation is {}".format(q))

    #==============================================================================
    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return

        q = msg.orientation
        q = numpy.array([q.x, q.y, q.z, q.w])

        if not self.initialized:
            # If this is the first callback: Store and hold latest pose.
            self.quat_des = q
            self.initialized = True

        # Compute control output:
        t = time_in_float_sec_from_msg(msg.header.stamp)

        # Error quaternion wrt body frame
        e_rot_quat = transf.quaternion_multiply(transf.quaternion_conjugate(q), self.quat_des)

        # Error angles
        e_rot = numpy.array(transf.euler_from_quaternion(e_rot_quat))

        v_angular = self.pid_rot.regulate(e_rot, t)

        # Convert and publish vel. command:
        cmd_vel = geometry_msgs.Twist()
        cmd_vel.angular = geometry_msgs.Vector3(x=v_angular[0], y=v_angular[1], z=v_angular[2])
        self.pub_cmd_vel.publish(cmd_vel)

    #==============================================================================
    def callback_params(self, data):
        """Handle updated configuration values."""
        for parameter in data:
            #if parameter.name == "name":
            #if parameter.type_ == Parameter.Type.DOUBLE:
            self.config[parameter.name] = parameter.value

        # Config has changed, reset PID controllers
        self.create_pids(self.config)

        self.get_logger().warn("Parameters dynamically changed...")
        return SetParametersResult(successful=True)

    #==============================================================================
    def create_pids(self, config):
        self.pid_rot = PIDRegulator(config['rot_p'], config['rot_i'], config['rot_d'], config['rot_sat'])

    #==============================================================================
    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value

#==============================================================================
def main():
    print('Starting orientation_control.py')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = OrientationControllerNode('orientation_control', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        node.pub_cmd_vel.publish(Twist())
        if rclpy.ok():
            rclpy.shutdown()
        print('Exiting')

#==============================================================================
if __name__ == '__main__':
    main()
