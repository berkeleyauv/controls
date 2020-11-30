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

def test(system):
    pid = PID(2, 0, 0, 15, thresh=.1, setpoint=100)

    val = 5
    t = 1
    big_number = 1000
    while not pid.onTarget(val):
        if t > big_number:
            print("PID did not converge to setpoint in time")
            return

        # system: x[t+1] = x[t] + u[t]
        u = pid.calculate(val, t)
        val = system(val, u)
        t += 1

    print("yay PID worked")
    print(f"error: {abs(pid.setpoint - val)}")

if __name__ == "__main__":
    test(simple_system)
    test(cubic_system)
    test(exponential_sys)
    
# TODO: write tests for complicated systems
# TODO: plot state vs. time with matplotlib