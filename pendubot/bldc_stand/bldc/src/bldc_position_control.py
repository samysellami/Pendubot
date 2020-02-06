#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np
import odrive
from odrive.enums import *
import math
import time


def callback_command(msg):
    print(msg.data)
    my_drive.axis0.controller.pos_setpoint = -16384*msg.data/2/math.pi + 16384*2.15/2/math.pi

def listener():
    my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    # my_drive.axis0.controller.config.pos_gain = 20.0 
    start_time  = int(round(time.time() * 1000))

    rate = rospy.Rate(500) #  500hz
    while not rospy.is_shutdown():
        now_ = int(round(time.time() * 1000))
        time_ = now_ - start_time

        pub_current.publish(my_drive.axis0.motor.current_control.Iq_measured)
        pub_position.publish(my_drive.axis0.encoder.pos_abs_rad)
        pub_velocity.publish(my_drive.axis0.encoder.vel_abs_filter)      

        rate.sleep()

    my_drive.reboot()
#--------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node("bldc_stend", anonymous=True)
    print("Node ODrive init...")

    pub_current = rospy.Publisher('current', Float64, queue_size=1)
    pub_position = rospy.Publisher('position', Float64, queue_size=1)
    pub_velocity = rospy.Publisher('velocity', Float64, queue_size=1)
    rospy.Subscriber("command", Float64, callback_command)

    my_drive = odrive.find_any()
    print("ODrive found.")
    
    try:
            listener()
    except rospy.ROSInterruptException:
            my_drive.reboot()
