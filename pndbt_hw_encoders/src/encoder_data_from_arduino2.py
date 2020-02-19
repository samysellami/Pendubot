#!/usr/bin/env python

import rospy
from maccepa_msgs.msg import EncRawData2
import serial

def talker():
    pub = rospy.Publisher('raw_encoders_data2', EncRawData2, queue_size=10)

    rospy.init_node('encoders_data_from_arduino2', anonymous=True)
    port = rospy.get_param("raw_encoders_data/port_adress")
    ser = serial.Serial(port, 115200)
    ser.readline()

    msg = EncRawData2()

    while not rospy.is_shutdown():
        raw_data = [float(i) for i in ser.readline().strip().split(" ")]
        msg.pose_1 = raw_data[0]
        msg.vel_1 = raw_data[1]
        msg.pose_2 = raw_data[2]
        msg.vel_2 = raw_data[3]
        
        pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

