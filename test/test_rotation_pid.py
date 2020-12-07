#!/usr/bin/env python3

import numpy as np
import rclpy
import matplotlib.pyplot as plt

from controls.controllers.PIDController import PID

def simple_system(val, u):
    return val + u

def linear_system(val, u):
    return .3 * val + 2 * u

def neg_linear_system(val, u):
    return -val + u

def sine_system(val, u):
    return val * np.sin(val) + u

def sine_sys2(val, u):
    return val * np.sin(val * u)

def sqrt_system(val, u):
    return (val) ** .5 + u

def log_system(val, u):
    return np.log(val + u)

# unstable system?
def cubic_system(val, u):
    return (val + u)**3

# unstable system?
def exponential_sys(val, u):
    return 2**(val - u)

def test(system, p, i, d, sat):
    goal = 100
    pid = PID(p, i, d, sat, thresh=.1, setpoint=goal)

    hz = 50
    val = 5
    t = 0
    max_time = 5

    val_arr = [val]

    while not pid.onTarget(val):
        if np.isnan(val):
            print("Value overflowed")
            plt.plot(val_arr)
            plt.plot(np.full(len(val_arr), goal))
            plt.legend(['state values', 'setpoint'])
            plt.show()
            return
        if t > max_time:
            print("PID did not converge to setpoint in time")
            l = len(val_arr)
            plt.plot(val_arr[:l])
            plt.plot(np.full(l, goal))
            plt.legend(['state values', 'setpoint'])
            plt.show()
            return

        u = pid.calculate(val, t)
        val = system(val, u)
        t += 1/hz
        val_arr.append(val)

    print("yay PID worked")
    print(f"error: {abs(pid.setpoint - val)}")
    plt.plot(val_arr)
    plt.plot(np.full(len(val_arr), goal))
    plt.legend(['state values', 'setpoint'])
    plt.show()

if __name__ == "__main__":
    # test(simple_system, 1.5, 0, 0, 15)
    test(linear_system, .5, .1, 0, 1000)
    # test(neg_linear_system, -.1, 0, -100, 1000)
    # test(sqrt_system, 10000, 0, 0, 1)
    # test(sine_system, 1, 0, 0, 15)
    # test(sine_sys2, 10, 0, 0, 15)
    
# TODO: tune PID gains for cubic and exponential system
