#!/usr/bin/env python
import subprocess
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np
import odrive
from odrive.enums import *
import math
import time
import rosbag
import glob
import signal
import os
import psutil
import message_filters

path = os.path.dirname(os.path.realpath(__file__))
os.chdir(path)


def terminate_process_and_children(p):
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()  # we wait for children to terminate


def change_name(name):
    path = os.path.dirname(os.path.realpath(__file__))
    os.chdir(path)
    File_name = str(glob.glob('*.bag')[0])
    print(File_name)
    os.rename(os.path.join(path, File_name), os.path.join(path+'/Bags', name))

def callback(cur, ft, comm, pos,vel,js):
    pub_current.publish(cur)
    pub_torque.publish(ft)
    pub_command.publish(comm)
    pub_position.publish(pos)
    pub_velocity.publish(vel)
    pub_joint_states.publish(js)
    #if  abs(js.position[1]) > math.pi/4:
        #rospy.signal_shutdown('limits reached !!!!')



if __name__ == '__main__':

    rospy.init_node('rotate', anonymous=True)
    # Enter the parameters

    v = 1  # frequency in sec^-1
    amp = 1.2   # Amplitude
    name = 'ramp_A_'+str(amp)+'_v_' + str(v)+'.bag'
    # name = 'current_ct_A_'+str(amp)+'_v_' + str(v)+'.bag'
    time_exec = 20
    rate = rospy.Rate(500)  # 500hz 

    start_time = time.time()
    time_loop = start_time
    pub = rospy.Publisher('/bldc/command', Float64, queue_size=10)
    start_time = time.time()

    current_sub = message_filters.Subscriber('/bldc/current', Float64)
    ft_sub = message_filters.Subscriber('/bldc/netft_data', WrenchStamped)
    command_sub = message_filters.Subscriber('/bldc/command', Float64)
    position_sub = message_filters.Subscriber('/bldc/position', Float64)
    velocity_sub = message_filters.Subscriber('/bldc/velocity', Float64)
    joint_states_sub = message_filters.Subscriber('/bldc/joint_states', JointState)

    pub_current = rospy.Publisher('current_synch', Float64, queue_size=1)
    pub_torque = rospy.Publisher('netft_data_synch', WrenchStamped, queue_size=1)
    pub_command = rospy.Publisher('command_synch', Float64, queue_size=1)
    pub_position = rospy.Publisher('position_synch', Float64, queue_size=1)
    pub_velocity = rospy.Publisher('velocity_synch', Float64, queue_size=1)
    pub_joint_states = rospy.Publisher('joint_states_synch', JointState, queue_size=1)


    ts = message_filters.ApproximateTimeSynchronizer([current_sub, ft_sub,command_sub,position_sub,velocity_sub,joint_states_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)


    try:
        command = "rosbag record -a"
        process = subprocess.Popen(
            'rosbag record -a'.format("my_rosbag_prefis"), stdout=subprocess.PIPE, shell=True, cwd=path)
        #for I in range(1,3,3):
        while time_loop-start_time < time_exec:
            # pos = amp*math.sin(2*math.pi*v*(time_loop-start_time))
            # pos = amp*math.sin(2*math.pi*v*(time_loop-start_time)) + amp*math.sin(2*math.pi*(2*v)*(time_loop-start_time))/2  + 0*amp*math.sin(2*math.pi*(3*v)*(time_loop-start_time))/3 + 0*amp*math.sin(2*math.pi*(4*v)*(time_loop-start_time))/4
            # pos = 2* amp * ((time_loop - start_time) /(time_exec)) -amp
            if time_loop-start_time < 0.5*time_exec:
                pos = 1.2
            else:
                pos = -1.2
            pub.publish(pos)
            time_loop = time.time()
            rate.sleep()
            if rospy.ROSInterruptException:
                pass


    finally:
        current_sub.unregister()
        ft_sub.unregister()
        command_sub.unregister()
        position_sub.unregister()
        velocity_sub.unregister()
        joint_states_sub.unregister()

        terminate_process_and_children(process)
        time.sleep(1)
        change_name(name)
        print("Bag File stopped recording...")
