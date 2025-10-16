#!/usr/bin/env python3

# from easyUR import UR
import geometry_msgs.msg
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
import geometry_msgs
from franges import drange, frange
from ros_numpy import numpify
import pandas as pd
import rtde_control
import rtde_receive
import time


if __name__ == '__main__':

    rospy.init_node('gelbelt_control')

    rtde_c = rtde_control.RTDEControlInterface("192.168.1.12")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.12")
    rospy.Rate(100)
    
    target_pos = [0.10, -0.85, 0.14]
    target_rot_euler = [90,0,180]
    r = R.from_euler('ZYX', target_rot_euler, degrees = True)
    target_rot_vec = r.as_rotvec()
    target = np.concatenate((target_pos, target_rot_vec))
    rtde_c.moveL(target, 0.01, 0.01, False)

