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

def callback_joint_states(msg):
    pass

def callback_wrench(msg):
    pass

def callback_command(msg):
    print(msg.data)
    my_drive.axis0.controller.pos_setpoint = msg.data

def listener():
    my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    start_time  = int(round(time.time() * 1000))

    rate = rospy.Rate(500) #  500hz
    while not rospy.is_shutdown():
        now_ = int(round(time.time() * 1000))
        time_ = now_ - start_time

        pub_current.publish(my_drive.axis0.motor.current_control.Iq_measured)
        # pub_target_current.publish(my_drive.axis0.controller.current_setpoint)
        pub_position.publish(my_drive.axis0.encoder.pos_abs_rad)
        pub_velocity.publish(my_drive.axis0.encoder.vel_abs_filter)
        pub_target_position.publish(my_drive.axis0.controller.pos_setpoint)        

        rate.sleep()

    my_drive.reboot()
#--------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node("bldc_stend", anonymous=True)
    print("Node ODrive init...")

    pub_current = rospy.Publisher('current', Float64, queue_size=1)
    pub_position = rospy.Publisher('position', Float64, queue_size=1)
    pub_target_position = rospy.Publisher('target_position', Float64, queue_size=1)
    pub_velocity = rospy.Publisher('velocity', Float64, queue_size=1)
    # pub_target_current = rospy.Publisher('target_current', Float64, queue_size=1)
    # pub_abs = rospy.Publisher('measured_abs', Float64, queue_size=1)
    rospy.Subscriber("joint_states", JointState, callback_joint_states)
    rospy.Subscriber("netft_data", WrenchStamped, callback_wrench) 
    rospy.Subscriber("command", Float64, callback_command)

    my_drive = odrive.find_any()
    print("ODrive found.")
    
    try:
            listener()
    except rospy.ROSInterruptException:
            my_drive.reboot()
