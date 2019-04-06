#!/usr/bin/env python

from __future__ import division, print_function

import rospy
#import ControlMode
from setRCOutput import setMotor

'''
A python script to reset motor output to zero.
'''

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    #mode = SetControlMode()
    #ControlMode.sender.send('output')
    zero = [1500]*8
    setMotor.send(zero)
    print('Set zero output')
