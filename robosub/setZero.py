#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from setMode import SetControlMode
from setRCOutput import SetOutput

'''
A python script to reset motor output to zero.
'''

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node("SetZero")
    mode = SetControlMode()
    output = SetOutput()
    mode.send('output')
    zero = [1500]*8
    output.send(zero)
    print('Set zero output')
