#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np
import math
import time
#import rosbag
import glob
import signal
import os
import psutil
import message_filters
import rospy
import math
import control

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
         
        
    def D_mtrx(self, q):
      D = np.zeros((2,2))
      D[0,0] =  self.theta[0] + self.theta[1] + 2 * self.theta[3] * math.cos(q[1])
      D[0,1] =  self.theta[1] + self.theta[2] * math.cos(q[1])
      D[1,0] =  self.theta[1] + self.theta[2] * math.cos(q[1])
      D[1,1] =  self.theta[2]
      return D 
        
    def C_mtrx(self, q, q_d):
      C = np.zeros((2,2))
      C[0,0] =  -self.theta[2] * math.sin(q[1]) * q_d[1]  
      C[0,1] =  -self.theta[2] * math.sin(q[1]) * q_d[1] -self.theta[2] * math.sin(q[1]) * q_d[0]
      C[1,0] =  self.theta[2] * math.sin(q[1]) * q_d[0]
      C[1,1] =  0
      return C 

    def g_vctr(self, q):
      g = np.zeros(2)
      g[0] =  self.theta[3] * self.g * math.cos(q[0]) + self.theta[4] * self.g * math.cos(q[0] + q[1])   
      g[1] =  self.theta[4] * self.g * math.cos(q[0] + q[1])
      return g 


    def A_lin(self):
      A = np.zeros((4,4))	
      A[0,2] = 1
      A[1,3] = 1
      A[2,0] = -(self.g * self.theta[1] * self.theta[3] - self.g * self.theta[2] * self.theta[4]) / (- self.theta[2] ** 2 + self.theta[0] * self.theta[1])
      A[2,1] = (self.g * self.theta[2] * self.theta[4]) / (-  self.theta[2]**2 + self.theta[0] * self.theta[1])
      A[3,0] =  (self.g * self.theta[0] * self.theta[4] + self.g * self.theta[1] * self.theta[3] - self.g * self.theta[2] * self.theta[3]  \
	- self.g * self.theta[2]* self.theta[4]) / (- self.theta[2] **2 + self.theta[0] * self.theta[1])
      A[3,1] = (self.g * self.theta[0] * self.theta[4] - self.g * self.theta[2] * self.theta[4]) / (- self.theta[2] ** 2 + self.theta[0] * self.theta[1])
      return A 

    def B_lin(self):
      B = np.zeros((4,1))
      B[1,0] = self.theta[1] / (- self.theta[2] ** 2 + self.theta[0] * self.theta[1])
      B[3,0] = -( self.theta[1]  -  self.theta[2] ) / (- self.theta[2] ** 2 + self.theta[0] * self.theta[1])
      return B 

if __name__ == '__main__':

    #rospy.init_node('python_command', anonymous=True)
    q  = np.zeros(2)
    q_d  = np.zeros(2)

    pendubot  = pndbt()
    #print(pendubot.g_vctr())
    
    Q = np.zeros((4, 4))
    Q[0,0] = 10
    Q[1,1] = 10
    R = 0.1		
    K, S, E = control.lqr(pendubot.A_lin(), pendubot.B_lin(), Q, R)
    print(K)

    
    










