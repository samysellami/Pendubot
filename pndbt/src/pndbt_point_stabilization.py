#!/usr/bin/env python
import subprocess
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np
import math
import scipy.io
import os
from os.path import dirname, join as pjoin
from numpy import linalg as LA
import timeit
from pndbt_integral.srv import *
import control
from control.matlab import *
from scipy.linalg import*


def torque_limit(torque, max_value):
    if torque > max_value:
      torque = max_value
    if torque < -max_value:
      torque = -max_value
    return torque

def load_mat(mat_name, K):
    data_dir = pjoin(dirname(__file__))
    mat_fname = pjoin(data_dir, 'mat_files/'+str(mat_name))
    mat = scipy.io.loadmat(mat_fname)
    
    if K:
      mat = mat['K_mtrx']
    else:
      mat = mat['q_str']
    return mat

class pndbt():
    """docstring for ClassName"""
    def __init__(self, Q, R, k, phi0, thta0,  K_mtrces, q_strs):
      params = {'m1': 1.085, 'm2': 0.26, 'l1': 0.25, 'l2': 0.25, 'I1': 0.008, 'I2': 0.002, 'l_com1': 0.043, 'l_com2': 0.095} 
      self.p1 = params['m1'] * (params['l_com1'])**2 +  params['m2'] * (params['l1'])**2 +  params['I1']
      self.p2 = params['m2'] * (params['l_com2']**2) +  params['I2']
      self.p3 = params['m2'] * params['l1'] * params['l_com2'] 
      self.p4 = params['m1'] * params['l_com1'] +  params['m2'] * params['l1']
      self.p5 = params['m2'] * params['l_com2']
      self.g = 9.81
      # K, S, E = control.lqr(self.A_lin(), self.B_lin(), Q, R)
      # S =np.matrix(solve_continuous_are(self.A_lin(), self.B_lin(), Q, R))
      # K = np.matrix((self.B_lin().T*S)/R)
      # self.K = K
      self.k = k
      self.phi0 = phi0
      self.thta0 = thta0
      
      self.K_mtrces = K_mtrces
      self.q_strs = q_strs

      self.K_mtrx = K_mtrces[0]
      self.s_str =  q_strs[0][:,1]
      self.s_d_str =  q_strs[0][:,3]

      self.torque_pub = rospy.Publisher('/pndbt/shoulder_torque_controller/command', Float64, queue_size=10)
      self.joint_states_sub = rospy.Subscriber('/pndbt/joint_states', JointState, self.callback)
      self.joint_states = 0
      
      self.q = np.array([ -math.pi/2 , 0])
      self.q_d = np.array([ 0 , 0])
        
    def D_mtrx(self, q):
      D = np.zeros((2,2))
      D[0,0] =  self.p1 + self.p2 + 2 * self.p3 * math.cos(q[1])
      D[0,1] =  self.p2 + self.p3 * math.cos(q[1])
      D[1,0] =  self.p2 + self.p3 * math.cos(q[1])
      D[1,1] =  self.p2
      return D 
        
    def C_mtrx(self, q, q_d):
      C = np.zeros((2,2))
      C[0,0] =  -self.p3 * math.sin(q[1]) * q_d[1]  
      C[0,1] =  -self.p3 * math.sin(q[1]) * q_d[1] -self.p3 * math.sin(q[1]) * q_d[0]
      C[1,0] =  self.p3 * math.sin(q[1]) * q_d[0]
      C[1,1] =  0
      return C 

    def g_vctr(self, q):
      g = np.zeros(2)
      g[0] =  self.p4 * self.g * math.cos(q[0]) + self.p5 * self.g * math.cos(q[0] + q[1])   
      g[1] =  self.p5 * self.g * math.cos(q[0] + q[1])
      return g 

    def A_lin(self):
      A = np.zeros((4,4)) 
      A[0,2] = 1
      A[1,3] = 1
      A[2,0] = -(self.g * self.p2 * self.p4 - self.g * self.p3 * self.p5) / (- self.p3 ** 2 + self.p1 * self.p2)
      A[2,1] = (self.g * self.p3 * self.p5) / (-  self.p3**2 + self.p1 * self.p2)
      A[3,0] =  (self.g * self.p1 * self.p5 + self.g * self.p2 * self.p4 - self.g * self.p3 * self.p4  \
      - self.g * self.p3* self.p5) / (- self.p3 **2 + self.p1 * self.p2)
      A[3,1] = (self.g * self.p1 * self.p5 - self.g * self.p3 * self.p5) / (- self.p3 ** 2 + self.p1 * self.p2)
      return A 

    def B_lin(self):
      B = np.zeros((4,1))
      B[2,0] = self.p2 / (- self.p3 ** 2 + self.p1 * self.p2)
      B[3,0] = -( self.p2  -  self.p3 ) / (- self.p3 ** 2 + self.p1 * self.p2)
      return B 

    def Uff(self,s, s_d, y, y_d):  

      U =  ((self.p1*self.p2 - self.p3**2*math.cos(s)**2) * (self.k*(((self.p2 + self.p3*math.cos(s)) * ((981*self.p5*math.cos(self.phi0 + s + y + self.k*(s - self.thta0))) / 100 \
      - s_d*(s_d*self.p3*math.sin(s) + self.p3*math.sin(s) * (y_d + self.k*s_d) + self.k*s_d*self.p3*math.sin(s)) + (981*self.p4*math.cos(self.phi0 + y + self.k*(s - self.thta0))) / 100 \
      - s_d*self.p3*y_d*math.sin(s))) / (self.p1*self.p2 - self.p3**2*math.cos(s)**2) - (((981*self.p5*math.cos(self.phi0 + s + y + self.k*(s - self.thta0))) / 100 \
      + self.p3*y_d*math.sin(s) * (y_d + self.k*s_d) + self.k*s_d*self.p3*math.sin(s) * (y_d + self.k*s_d))*(self.p1 + self.p2 + 2*self.p3*math.cos(s)))/ \
      (self.p1*self.p2 - self.p3**2*math.cos(s)**2)) - ((self.p2 + self.p3*math.cos(s))*((981*self.p5*math.cos(self.phi0 + s + y + self.k*(s - self.thta0))) / 100 \
      + self.p3*y_d*math.sin(s) * (y_d + self.k*s_d) + self.k*s_d*self.p3*math.sin(s)*(y_d + self.k*s_d)))/(self.p1*self.p2 - self.p3**2*math.cos(s)**2) \
      + (self.p2 * ((981*self.p5*math.cos(self.phi0 + s + y + self.k*(s - self.thta0)))/ 100 - s_d * (s_d*self.p3*math.sin(s) + self.p3*math.sin(s) * (y_d + self.k*s_d) + \
      self.k*s_d*self.p3*math.sin(s)) + (981*self.p4*math.cos(self.phi0 + y + self.k*(s - self.thta0)))/100 - s_d*self.p3*y_d*math.sin(s)))/ \
      (self.p1*self.p2 - self.p3**2*math.cos(s)**2))) / (self.p2 + self.k*self.p2 + self.k*self.p3*math.cos(s))
      return U  

    def inv_N(self, s):
      N = (self.p1 * self.p2 - self.p3**2*math.cos(s)**2)/(self.p2 + self.k * self.p2 + self.k * self.p3*math.cos(s))
      return N

    def U_full(self,s, s_d, y, y_d, v):
      U_f = self.Uff(s, s_d, y, y_d) + self.inv_N(s) * v
      return U_f

    def y_trnsv(self, phi, s):
      y = phi - (self.phi0 + self.k * (s - self.thta0))
      return y

    def y_d_trnsv(self, phi_d, s_d):
      y_d = phi_d - self.k * s_d
      return y_d

    def alpha(self, s):
      a  = self.p2 + self.k * (self.p2 + self.p3 * math.cos(s))
      return a

    def beta(self, s):
      b = self.k**2 * self.p3 * math.sin(s)
      return b 

    def gamma(self, s):
      g = (self.g * self.p5 * math.cos(self.phi0 + s + self.k*(s - self.thta0)))
      return g 
  
    def simpsons( self, func, ll, ul, n ):     
        # Calculating the value of h 
        h = ( ul - ll )
        h = h / n 
        # List for storing value of x and f(x) 
        x = []
        fx = [] 
        # Calcuting values of x and f(x) 
        i = 0
        while i<= n: 
            x.append(ll + i * h) 
            fx.append(func(x[i]))
            i += 1 
        # Calculating result 
        res = 0
        i = 0
        while i<= n: 
            if i == 0 or i == n: 
                res+= fx[i] 
            elif i % 2 != 0: 
                res+= 4 * fx[i] 
            else: 
                res+= 2 * fx[i] 
            i+= 1
        res = res * (h / 3) 
        return res 

    def int_psi(self, s0, s):
      if s == s0:
        return 0
      fun = lambda s: (self.beta(s) / self.alpha(s))
      int1 = self.simpsons( fun,s0,s,1000 )
      psi = math.exp(-2*int1)
      return psi  

    def int_fi(self, s0, s):
      func = lambda x: (self.gamma(s) / self.alpha(s))
      fi = self.int_psi(s,s0) * 2 * func(s);
      return fi
    
    def int(self, s,s_d,s_0,s_d0):
      if s == s_0:
          I = s_d**2 - s_d0**2;
          return

      f = lambda s: (self.int_fi(s_0, s))
      int1 = self.simpsons( f,s_0,s,1000 )
      I = s_d**2 - self.int_psi(s_0,s) * (s_d0**2 - int1)
      return I

    def orbital_stabilization(self):

      start = timeit.default_timer()
            
      theta = self.q[1]
      theta_d = self.q_d[1]
      phi = self.q[0]
      phi_d = self.q_d[0]

      y = self.y_trnsv(phi, theta)
      y_d = self.y_d_trnsv(phi_d, theta_d)
      # I = self.int(theta, theta_d, self.s_str[0], self.s_d_str[0])
     
      rospy.wait_for_service('Intg')
      Integral = rospy.ServiceProxy('Intg', Intg)
      intg = Integral(theta, theta_d, self.s_str[0], self.s_d_str[0])
      I = intg.I
      x_trsv = np.array([I, y , y_d])
      delta  = np.subtract( np.array([theta, theta_d]).reshape(2,1) , np.array([self.s_str, self.s_d_str]))
      delta_norm = LA.norm(delta, axis = 0)  
      idx = np.argmin(delta_norm)
      stop = timeit.default_timer()

      u_fbck = (self.K_mtrx[:,:,idx][0]).dot(x_trsv) 
      u_in = self.U_full(theta, theta_d, y, y_d, u_fbck)
      self.torque_pub.publish(u_in)
      
      print("the control input is equal to " +str(u_in))
      #print('Time: ', stop - start)  


    def linear_stabilization(self):

      limit_1 = 0
      limit_2 = -math.pi

      if (self.q[0] > limit_1) or (self.q[0] < limit_2):
        self.torque_pub.publish(0)
        rospy.sleep(0.5)
        rospy.signal_shutdown('Limits exceeded!')
            
      else: 
        qf = np.array([-math.pi/2, math.pi])        
        x  = np.hstack(((self.q-qf,self.q_d)))
        u = np.dot(-self.K, x.transpose())
        u_in = torque_limit(u[0,0],8*0.123)
        self.torque_pub.publish(u_in)
        print("the control input is equal to " +str(u_in))
  

    def callback(self,joint_states):
      self.q = np.array(joint_states.position)
      self.q_d = np.array(joint_states.velocity)


def control():

    rospy.init_node('python_command', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    Q = np.zeros((4, 4))
    Q[0,0] = 1
    Q[1,1] = 1
    R = 1
    k = 0.5
    phi0 = -math.pi/2
    thta0 = 0

    K_mtrx = load_mat('K_mtrx.mat', 1)
    q_str = load_mat('q_str.mat', 0)

    K_mtrx1 = load_mat('K_mtrx1.mat', 1)
    q_str1 = load_mat('q_str1.mat', 0)

    K_mtrx2 = load_mat('K_mtrx2.mat', 1)
    q_str2 = load_mat('q_str2.mat', 0)
      

    K_mtrces = [K_mtrx1, K_mtrx2]
    q_strs = [q_str1, q_str2]
    
  
    pendubot  = pndbt(Q, R, k, phi0, thta0, K_mtrces, q_strs)

    while not rospy.is_shutdown():
      """if (abs(pendubot.q[1]-math.pi) < math.pi/20) and (abs(pendubot.q[0] + math.pi/2) < math.pi/20): 
                              print("linear stabilization!!!")
                              pendubot.linear_stabilization()  
                          else:"""  
      #print("orbital stabilization!!!!")
      pendubot.orbital_stabilization()

      rate.sleep()
      #rospy.spin()


if __name__ == '__main__':
    try:

      control()
    except rospy.ROSInterruptException:
      pass
   








