#!/usr/bin/env python3

import numpy as np
import rclpy
import matplotlib.pyplot as plt

from controls.controllers.PIDController import PID

def simple_system(val, u):
    return val + u

def quadratic_system(val, u):
    return val**2 + u

def cubic_system(val, u):
    return (val + u)**3

def exponential_sys(val, u):
    return 2**(val - u)

def test(system, p, i, d, sat):
    goal = 100
    pid = PID(p, i, d, 15, thresh=.1, setpoint=goal)

    val = 5
    t = 1
    big_number = 1000

    val_arr = [val]

    while not pid.onTarget(val):
        if np.isnan(val):
            print("Value overflowed")
            plt.plot(val_arr)
            plt.plot(np.full(len(val_arr), goal))
            plt.show()
            return
        if t > big_number:
            print("PID did not converge to setpoint in time")
            plt.plot(val_arr)
            plt.plot(np.full(len(val_arr), goal))
            plt.show()
            return

        # system: x[t+1] = x[t] + u[t]
        u = pid.calculate(val, t)
        val = system(val, u)
        t += 1
        val_arr.append(val)

    print("yay PID worked")
    print(f"error: {abs(pid.setpoint - val)}")
    plt.plot(val_arr)
    plt.plot(np.full(len(val_arr), goal))
    plt.show()

if __name__ == "__main__":
    # test(simple_system, 1.5, 0, 0, 15)
    test(quadratic_system, -100, 0, 0, 1000)
    # test(cubic_system, -1, 0, 0, 15)
    # test(exponential_sys, 0, 0, 0, 15)
    
# TODO: tune PID gains for cubic and exponential system
