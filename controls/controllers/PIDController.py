import numpy as np

class PID:
    """Class that handles PID looping and outputs."""

    def __init__(self, p, i , d, sat, thresh=1.0, setpoint=0.0):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat
        self.setpoint = setpoint
        self.prevIntegral = 0.0
        self.prevError = 0.0
        self.prevTime = 0.0
        self.PID_THRESH = thresh
        self.bias = 0.0

    def reset(self):
        self.prevError = 0.0
        self.prevIntegral = 0.0
        self.prevTime = 0.0

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def calculate(self, curVal, curTime):
        error = self.setpoint - curVal
        print("setpoint:", self.setpoint)
        print("curVal:", curVal)
        print("error:", error)
        print()
        p_term = self.p*error
        d_term = self.d*(error - self.prevError)/(curTime - self.prevTime)
        integral = self.prevIntegral + error*(curTime - self.prevTime)
        self.prevError = error
        self.prevTime = curTime
        self.prevIntegral = integral

        u = p_term + self.i*integral + d_term
        if np.linalg.norm(u) > self.sat:
            u = u*self.sat/np.linalg.norm(u)
            self.prevIntegral = 0.0
        return u

    def onTarget(self, curVal):
        return abs(self.setpoint - curVal) < self.PID_THRESH
