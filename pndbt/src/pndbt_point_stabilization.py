#!/usr/bin/env python
import subprocess
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np
import math
# import time
#import rosbag
# import glob
# import signal
# import os
# import psutil
# import message_filters
# import control
# from control.matlab import *
from scipy.linalg import*

def torque_limit(torque, max_value):
  if torque > max_value:
    torque = max_value
  if torque < -max_value:
    torque = -max_value
  return torque


class pndbt():
    """docstring for ClassName"""
    def __init__(self, Q, R):
      params = {'m1': 1.085, 'm2': 0.26, 'l1': 0.25, 'l2': 0.25, 'I1': 0.008, 'I2': 0.002, 'l_com1': 0.043, 'l_com2': 0.095} 
      theta1 = params['m1'] * (params['l_com1'])**2 +  params['m2'] * (params['l1'])**2 +  params['I1']
      theta2 = params['m2'] * (params['l_com2']**2) +  params['I2']
      theta3 = params['m2'] * params['l1'] * params['l_com2'] 
      theta4 = params['m1'] * params['l_com1'] +  params['m2'] * params['l1']
      theta5 = params['m2'] * params['l_com2']
      self.g = 9.81
      self.theta = np.array([theta1, theta2, theta3, theta4, theta5])  
      # K, S, E = control.lqr(self.A_lin(), self.B_lin(), Q, R)
      S =np.matrix(solve_continuous_are(self.A_lin(), self.B_lin(), Q, R))
      K = np.matrix((self.B_lin().T*S)/R)
      self.K = K

      self.torque_pub = rospy.Publisher('/pndbt/shoulder_torque_controller/command', Float64, queue_size=10)
      self.joint_states_sub = rospy.Subscriber('/pndbt/joint_states', JointState, self.callback)
         
        
    def D_mtrx(self, q):
      D = np.zeros((2,2))
      D[0,0] =  self.theta[0] + self.theta[1] + 2 * self.theta[2] * math.cos(q[1])
      D[0,1] =  self.theta[1] + self.theta[2] * math.cos(q[1])
      D[1,0] =  self.theta[1] + self.theta[2] * math.cos(q[1])
      D[1,1] =  self.theta[1]
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
      B[2,0] = self.theta[1] / (- self.theta[2] ** 2 + self.theta[0] * self.theta[1])
      B[3,0] = -( self.theta[1]  -  self.theta[2] ) / (- self.theta[2] ** 2 + self.theta[0] * self.theta[1])
      return B 


    def callback(self,joint_states):
      q = np.array(joint_states.position)
      q_d = np.array(joint_states.velocity)
      limit_1 = 0
      limit_2 = -math.pi
      if (q[0] > limit_1):
        self.torque_pub.publish(0)
        rospy.sleep(0.5)
        rospy.signal_shutdown('Limits exceeded!')
      elif (q[0] < limit_2):
        self.torque_pub.publish(0)
        rospy.sleep(0.5)
        rospy.signal_shutdown('Limits exceeded!')
      else: 
        qf = np.array([-math.pi/2, math.pi])        
        x  = np.hstack(((q-qf,q_d)))
        u = np.dot(-self.K, x.transpose())
        u_in = torque_limit(u[0,0],8*0.123)
        self.torque_pub.publish(u_in)
        print(u_in)
        # print(q-qf)

def control():

  rospy.init_node('python_command', anonymous=True)

  Q = np.zeros((4, 4))
  Q[0,0] = 1
  Q[1,1] = 1
  R = 1
  pendubot  = pndbt(Q, R)    
  print(pendubot.K)

  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException:
    pass
   








