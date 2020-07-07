#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32
from maccepa_msgs.msg import EncRawData2
import math
import serial
import numpy as np
from collections import deque
from scipy import zeros, signal, random
from threading import Lock

NODE_NAME = "encoder_to_joint_state"
 
class EncoderRawData:
    def __init__(self):
        self.pos = 0
        self.vel = 0
        self.mutex = Lock()
        self.read_status = 0

    
    def setData(self, pos, vel):
        self.mutex.acquire()

        self.pos = pos
        self.vel = vel

        self.mutex.release()
        self.read_status = 1

    def getData(self):

        self.mutex.acquire()
        pos = self.pos
        vel = self.vel

        self.mutex.release()

        return pos, vel

    def isDataReady(self):
        if self.read_status == 1:
            return 1
        return 0

class Encoder:
    def __init__(self, offset=0):
        self.offset = offset

        self.prev_pos = 0.
        self.prev_velocity = 0.
        self.prev_time = 0.
        self.us_in_sec = 1000000.

        self.b, self.a = signal.butter(4, 0.3, 'low')
        self.z = signal.lfilter_zi(self.b, self.a)
        
    def getPosition(self, raw_position, raw_vel):

        enc = raw_position + self.offset

        return -float(enc)

    def getVelocity(self, raw_position, raw_vel):

        return -float(raw_vel)

def callback(data):
    global raw_data_list
    raw_data_list[0].setData(data.pose_1, data.vel_1)
    raw_data_list[1].setData(data.pose_2, data.vel_2)
    
def initMsgJointState():
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['shoulder', 'elbow']
    msg.position = [0,0]
    msg.velocity = [0,0]
    msg.effort = [0,0]

    return msg


def listener():
    rospy.init_node(NODE_NAME, anonymous=True)

    rospy.Subscriber("raw_encoders_data2", EncRawData2, callback)
    pub = rospy.Publisher('/pndbt/joint_states', JointState, queue_size=10)
    
    rate = rospy.Rate(200) # 10hz

    msg = initMsgJointState()

    encoder_1 = Encoder(offset = math.pi/2) #encoder value of zero position
    encoder_2 = Encoder(offset  = 0.03) #encoder value of zero position
    while not rospy.is_shutdown():
        if(raw_data_list[0].isDataReady() == 1):
            pos1, vel1 = raw_data_list[0].getData()
            msg.position[0] = encoder_1.getPosition(pos1, vel1)
            msg.velocity[0] = encoder_1.getVelocity(pos1, vel1)

        if(raw_data_list[1].isDataReady() == 1):
            pos2, vel2 = raw_data_list[1].getData()
            msg.position[1] = encoder_2.getPosition(pos2, vel2)
            msg.velocity[1] = encoder_2.getVelocity(pos2, vel2)
	    msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

raw_data_list = [EncoderRawData(), EncoderRawData()]

if __name__ == '__main__':
    listener()
