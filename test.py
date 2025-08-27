import geometry_msgs.msg
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
import geometry_msgs
from franges import drange, frange
from ros_numpy import numpify
import rtde_control
import rtde_receive


rtde_c = rtde_control.RTDEControlInterface("192.168.1.12")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.12")
# print(ur_rtde)
starting = rtde_r.getActualTCPPose()

print(starting)