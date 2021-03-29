#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy import Node
from rclpy.clock import ClockType
from rcl_interfaces.msg import ParameterDescriptor
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry

from PIDController import PID

# modules from plankton repo
from plankton_utils.time import time_in_float_sec_from_msg
from plankton_utils.time import is_sim_time

class OrientationController(Node):
    def __init__(self, node_name, **kwargs):
        print('VelocityControllerNode: initializing node')
        super().__init__(node_name, **kwargs)

        self.config = {}

        self.v_angular_des = np.zeros(3)

        # Initialize pids with default parameters
        # TODO: correctly initialize PID parameters
        self.pid_angular = PID(1, 0, 0, 1)

        # Declared parameters are overriden with yaml values
        self._declare_and_fill_map("angular_p", 1., "p component of pid for angular vel.", self.config)
        self._declare_and_fill_map("angular_i", 0.0, "i component of pid for angular vel.", self.config)
        self._declare_and_fill_map("angular_d", 0.0, "d component of pid for angular vel.", self.config)
        self._declare_and_fill_map("angular_sat", 3.0, "saturation of pid for angular vel.", self.config)
        
        self._declare_and_fill_map(
            "odom_vel_in_world", True, "Is odometry velocity supplied in world frame? (gazebo)", self.config)
        
        self.set_parameters_callback(self.callback_params) # idk what this does
        
        self.create_pid(self.config)

        # ROS infrastructure
        # TODO: find which topics to subscribe/publish to
        self.sub_cmd_vel = self.create_subscription(geometry_msgs.Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_odometry = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.pub_cmd_accel = self.create_publisher(geometry_msgs.Accel, 'cmd_accel', 10)

    #==============================================================================
    def cmd_vel_callback(self, msg):
        """Handle updated set velocity callback."""
        # Just store the desired velocity. The actual control runs on odometry callbacks
        v_a = msg.angular
        self.v_angular_des = np.array([v_a.x, v_a.y, v_a.z])

    #==============================================================================
    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return

        +
        angular = msg.twist.twist.angular
        v_angular = np.array([angular.x, angular.y, angular.z])
        
        # Compute compute control output:
        t = time_in_float_sec_from_msg(msg.header.stamp)
        
        # TODO: make sure this is correctly using our own PID
        a_angular = self.pid_angular.calculate(v_angular, t)

        # Convert and publish accel. command:
        cmd_accel = geometry_msgs.Accel()
        cmd_accel.angular = geometry_msgs.Vector3(x=a_angular[0], y=a_angular[1], z=a_angular[2])
        self.pub_cmd_accel.publish(cmd_accel)

    #==============================================================================
    def callback_params(self, data):
        for parameter in data:
            self.config[parameter.name] = parameter.value
        
        # config has changed, reset PID controllers
        self.create_pid(self.config)

        self.get_logger().warn("Parameters dynamically changed...")
        return SetParametersResult(successful=True) #msg type

    #==============================================================================
    def create_pid(self, config):
        self.pid_angular = PID(
            config['angular_p'], config['angular_i'], config['angular_d'], config['angular_sat'])

    #==============================================================================
    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value

def main() -> None:
    print('Starting OrientationController.py')
    rclpy.init()

    try:
        # TODO: figure out if we can use plankton's is_sim_time
        sim_time_param = is_sim_time()

        node = OrientationController("orientation_controller", parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        rclpy.shutdown()
    print('Exiting')

if __name__ == "__main__":
    main()