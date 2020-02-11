#!/usr/bin/env python
import subprocess
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np
import odrive
from odrive.enums import *
import math
import time
import rosbag
import glob
import signal
import os
import psutil
import message_filters
import roslib; roslib.load_manifest('pndbt')
import rospy
from urdf_parser_py.urdf import URDF
import math

class pndbt():
    """docstring for ClassName"""
    def __init__(self):
        params = {'m1': 1.085, 'm2': 0.26, 'l1': 0.25, 'l2': 0.25, 'I1': 0.008, 'I2': 0.002, 'l_com1': 0.043, 'l_com2': 0.095} 
        theta1 = params['m1'] * (params['l_com1']**2) +  params['m2'] * (params['l1']**2) +  params['I1']
        theta2 = params['m2'] * (params['l_com2']**2) +  params['I2']
        theta3 = params['m2'] * params['l1'] * params['l_com2'] 
        theta4 = params['m1'] * params['l_com1'] +  params['m2'] * params['l1']
        theta5 = params['m2'] * params['l_com2']
        self.g = 9.81
        self.theta = np.array([theta1, theta2, theta3, theta4, theta5])  
         
        
    def D_mtrx(self, q, q_d):
        D = np.zeros((2,2))
        D[0,0] =  self.theta[1] + self.theta[2] + 2 * self.theta[3] * math.cos(q[1])
        D[0,1] =  self.theta[2] + self.theta[3] * math.cos(q[1])
        D[1,0] =  self.theta[2] + self.theta[3] * math.cos(q[1])
        D[1,1] =  self.theta[2]
        return D 
        
if __name__ == '__main__':

    #rospy.init_node('python_command', anonymous=True)
    q  = np.zeros(2)
    q_d  = np.zeros(2)

    pendubot  = pndbt()
    print(pendubot.D_mtrx(q, q_d))

