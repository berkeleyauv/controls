#!/usr/bin/env python3

import numpy as np
import rclpy
import matplotlib.pyplot as plt

from controls.controllers.PIDController import PID

def simple_system(val, u):
    return val + u

def cubic_system(val, u):
    return (val + u)**3

def exponential_sys(val, u):
    return 2**(val - u)

def test(system, p, i, d):
    pid = PID(p, i, d, 15, thresh=.1, setpoint=100)

    val = 5
    t = 1
    big_number = 1000

    val_arr = [val]

    while not pid.onTarget(val):
        if t > big_number:
            print("PID did not converge to setpoint in time")
            return

        # system: x[t+1] = x[t] + u[t]
        u = pid.calculate(val, t)
        val = system(val, u)
        t += 1
        val_arr.append(val)

    print("yay PID worked")
    print(f"error: {abs(pid.setpoint - val)}")
    plt.plot(val_arr)
    plt.show()

if __name__ == "__main__":
    test(simple_system, 1.5, 0, 0)
    test(cubic_system, 11, 11, 11)
    # test(exponential_sys)
    
# TODO: write tests for complicated systems
# TODO: plot state vs. time with matplotlib
