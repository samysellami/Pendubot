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
    my_drive.axis0.controller.current_setpoint = -msg.data/0.123  # coefficient torque / current


def listener():

    my_drive.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
    my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    my_drive.axis0.controller.current_setpoint = 0

    start_time = int(round(time.time() * 1000))

    rate = rospy.Rate(500)  # 500hz
    while not rospy.is_shutdown():
        
        pub_current.publish(my_drive.axis0.motor.current_control.Iq_measured)
        pub_position.publish(my_drive.axis0.encoder.pos_abs_rad)
        pub_velocity.publish(my_drive.axis0.encoder.vel_abs_filter)

        rate.sleep()

    my_drive.reboot()


# --------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node("bldc_stend", anonymous=True)
    print("Node ODrive init...")    

    pub_current = rospy.Publisher('current', Float64, queue_size=1)
    pub_position = rospy.Publisher('position', Float64, queue_size=1)
    pub_velocity = rospy.Publisher('velocity', Float64, queue_size=1)
    rospy.Subscriber("/pndbt/shoulder_torque_controller/command", Float64, callback_command)

    my_drive = odrive.find_any()
    print("ODrive found.")

    try:
        listener()
    except rospy.ROSInterruptException:
        my_drive.reboot()
