#!/usr/bin/env python
import rospy
from maccepa_msgs.msg import MotorsPose
import math
import numpy as np
import time

position = 0.0
transfer = []
n = 0
f = open("/home/oleg/catkin_ws/src/maccepa_manipulator/maccepa_hardware/encoders/data_transfer.txt","w+")
start_time = time.time()
def callback(data):
    global position, transfer, f, n,start_time
    n+=1
    f.write(str(time.time() - start_time) + " " + str(position)+ " " + str(data.motor_3_pose) + "\n")
    transfer.append([position, data.motor_3_pose])
    print("f")
    if n == 300:
        rospy.signal_shutdown("time")
        f.close()

def callback_set(data):
    global position
    position = data.motor_3_pose

def listener():
    rospy.init_node("transfer", anonymous=True)

    rospy.Subscriber("/maccepa/motors/states", MotorsPose, callback)
    rospy.Subscriber("/maccepa/motors/set_position", MotorsPose, callback_set)
    
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():

        rate.sleep()

if __name__ == '__main__':
    listener()