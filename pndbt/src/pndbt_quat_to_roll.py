#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from imu_sensor.msg import ImuData
import math

def convert(w,x,y,z):
    roll = 180*math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)/math.pi
    return roll

def callback(imu):
    roll = convert(imu.q0,imu.q1,imu.q2,imu.q3)
    pub.publish(roll)

def converter():
    rospy.spin()



if __name__ == '__main__':
    rospy.init_node('quat_to_roll', anonymous=True)    
    pub = rospy.Publisher('/pndbt/roll', Float64, queue_size=10)
    rospy.Subscriber("/pndbt/imu_sensor", ImuData, callback)
    try:
        converter()
    except rospy.ROSInterruptException:
        pass
