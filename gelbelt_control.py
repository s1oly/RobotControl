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


start_time = time.time()

def pose_msg_to_arrays (pose_stamped):
                                      
    pos = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z])
    quat = np.array([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
    
    return pos, quat

def world_eef_init(R,t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def world_gelbelt_init(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def gelbelt_old_gelbelt_new(R,t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def end_effector_to_gelbelt(t):
    T = np.eye(4)
    T[:3, 3] = t
    return T

def inverse_end_effector_to_gelbelt(t):
    T = np.linalg.inv(end_effector_to_gelbelt(t))
    return T


def force_callback(msg):
    global force
    force = numpify(msg.wrench.force) # have to figure out weight in air without contact with table and then subtract that to get a 0

    
def angle_callback(msg):
    global angle 
    angle = np.array(msg.data)

def pid_controller(Kp, setpoint, measurement):
    error = setpoint - measurement
    p = Kp*error
    mv = p 
    return mv

def simple_moving_average(data, window_size):
    if not isinstance(data, pd.Series):
        data = pd.Series(data)
    
    smoothed_data = data.rolling(window=window_size).mean()
    return smoothed_data.iloc[-1] if len(smoothed_data) > 1 else 0


if __name__ == '__main__':

    rospy.init_node('gelbelt_control')

    rtde_c = rtde_control.RTDEControlInterface("192.168.1.12")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.12")
    # print(ur_rtde)
    ground_truth = rtde_r.getActualTCPPose().copy()
    ground_rot = ground_truth[3:]
    b = R.from_rotvec(ground_rot)
    ground_pose_euler = b.as_euler('ZYX', degrees = True)
    print(ground_pose_euler)

    t = np.array([0,0,0.130])
    angle = np.array([0,0,0])
    force = np.array([0,0,0])


    rospy.Subscriber('/target_angle', std_msgs.msg.Float32MultiArray, angle_callback, queue_size=1)
    rospy.Subscriber('/wrist_sensor/wrench', geometry_msgs.msg.WrenchStamped, force_callback, queue_size=1)

    r = rospy.Rate(100)

    init_force = force[2].copy()

    data = []
    check_count = 0

    while not rospy.is_shutdown():

        data.append(force[2].copy())
        smoothed_average_force = simple_moving_average(data, window_size= 5)

        #Getting the matrix intialized
        init = rtde_r.getActualTCPPose()
        init_pos = init[0:3]
        init_rot = init[3:]

  
        if(check_count % 250 == 0):
            end_time = time.time()
            delta_t = end_time - start_time
            a = R.from_rotvec(init_rot)
            check_pose_euler = a.as_euler('ZYX', degrees = True)
            comparison_df = pd.DataFrame({
                'time_elasped': delta_t,
                'Robot_Angle Z': [check_pose_euler],
                'Ground_Truth': [ground_pose_euler]
            })
            output_path = "/home/shubaniyer/catkin_ws/src/rocky_scripts/src/data.csv" # change to data after debugging
            comparison_df.to_csv(output_path, mode= 'a', index=False)
            print(f"Data saved to {output_path}")
            if(delta_t > 25):
                rospy.signal_shutdown("Steady State")
        check_count = check_count + 1

        r = R.from_rotvec(init_rot)
        init_euler_eef = r.as_euler('ZYX', degrees = True)
        init_rotMatrix_eef = r.as_dcm() 

        T_world_eef_init = world_eef_init(init_rotMatrix_eef, init_pos)
        T_eef_init_to_gelbelt = end_effector_to_gelbelt(t)
        T_eef_init_to_geleblt_inverse = inverse_end_effector_to_gelbelt(t)


        init_gelbelt_rotMatrix = np.matmul(T_world_eef_init, T_eef_init_to_gelbelt)[:3, :3]
        init_gelbelt_pos = np.matmul(T_world_eef_init, T_eef_init_to_gelbelt)[:3, 3]

        T_world_gelbelt_old = world_gelbelt_init(init_gelbelt_rotMatrix, init_gelbelt_pos)

        # print("correcting")
        if(smoothed_average_force - init_force > -10):
            x_rotation = pid_controller(Kp=0.045, setpoint=0, measurement=0) #angle[2]
            y_rotation = pid_controller(Kp=0.055, setpoint= 0, measurement=0)
        else:
            x_rotation = pid_controller(Kp=0.045, setpoint=0, measurement=angle[2]) #angle[2]
            y_rotation = pid_controller(Kp=0.055, setpoint= 0, measurement=angle[1]) 

        if(x_rotation <= 0):
            x_rotation = np.max(np.array([-10.0,x_rotation]))
        elif(x_rotation > 0):
            x_rotation = np.min(np.array([10.0,x_rotation]))

        if(y_rotation <= 0):
            y_rotation = np.max(np.array([-6,y_rotation]))
        elif(x_rotation > 0):
            y_rotation = np.min(np.array([6,y_rotation]))


        correct = [0,y_rotation, x_rotation]
        r  = R.from_euler('ZYX', correct, degrees = True)
        gelbelt_correction_rotMatrix = r.as_dcm()
        target_gelbelt_pos_difference = np.array([0,0,0])

        T_gelbelt_old_gelbelt_new = gelbelt_old_gelbelt_new(gelbelt_correction_rotMatrix, target_gelbelt_pos_difference)
        T_world_gelbelt_new = np.matmul(T_world_gelbelt_old, T_gelbelt_old_gelbelt_new)

        T_world_eef_new = np.matmul(T_world_gelbelt_new, T_eef_init_to_geleblt_inverse)

        #The matrix after the correction
        r = R.from_dcm(T_world_eef_new[:3,:3])
        target_rotvec = r.as_rotvec()
        # force was -25 and -40, going to test different values to check how it looks 
        if(smoothed_average_force - init_force > -20):
            correction = pid_controller(Kp= 0.0001, setpoint= -20, measurement= force[2])
            correction = np.min(np.array([correction, -0.0001]))
        elif(smoothed_average_force - init_force < -40):
            correction = pid_controller(Kp= 0.0001, setpoint= -40, measurement= force[2])
            correction = np.max(np.array([correction, 0.0001]))
        # print(correction)
        target_pos = T_world_eef_new[:3, 3] + np.array([0,0.0000, correction]) # x movement was 0.0005, gonna make 0 for now just to test images/angle correction
        target = np.concatenate((target_pos, target_rotvec))
        rtde_c.servoL(target, 0.01, 0.01, 0.1, 0.05, 100)
        
        
  
        rospy.sleep(0.001)
        # rospy.signal_shutdown("A")

        # MATRIX MATH 
        #CASE 1 --> want to move down and correct --> target Z changes based on the end effector math, as well as 