#!/usr/bin/env python3
from easyUR import UR
import geometry_msgs.msg
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
import geometry_msgs
from franges import drange, frange
from ros_numpy import numpify

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



if __name__ == '__main__':
    robot = UR()
    starting_pos,starting_quat = pose_msg_to_arrays(robot.cur_pos)

    robot.set_ee_speed(0.01)
    robot.set_ee_acceleration(0.01)

    # t = np.array([0,0,0.130])
    angle = np.array([0,0,0])
    force = np.array([0,0,0])

    rospy.Subscriber('/target_angle', std_msgs.msg.Float32MultiArray, angle_callback, queue_size=1)
    rospy.Subscriber('/wrist_sensor/wrench', geometry_msgs.msg.WrenchStamped, force_callback, queue_size=1)

  
    while not rospy.is_shutdown():
   
        if(angle[2] <-0.3):
            t = np.array([0,0.032,0.130])
        elif(angle[2] > 0.3):
            t = np.array([0,-0.032, 0.130])
        else:
            t = np.array([0,0,0.130])


        #Getting the matrix intialized
        init_pos, init_quat = pose_msg_to_arrays(robot.cur_pos)

        r = R.from_quat(init_quat)
        init_euler_eef = r.as_euler('ZYX', degrees = True)
        init_rotMatrix_eef = r.as_dcm() 

        T_world_eef_init = world_eef_init(init_rotMatrix_eef, init_pos)
        T_eef_init_to_gelbelt = end_effector_to_gelbelt(t)
        T_eef_init_to_geleblt_inverse = inverse_end_effector_to_gelbelt(t)


        init_gelbelt_rotMatrix = np.matmul(T_world_eef_init, T_eef_init_to_gelbelt)[:3, :3]
        init_gelbelt_pos = np.matmul(T_world_eef_init, T_eef_init_to_gelbelt)[:3, 3]

        T_world_gelbelt_old = world_gelbelt_init(init_gelbelt_rotMatrix, init_gelbelt_pos)

        #Setting the Angle
        # rotation = [0, -2, -6]
        # r = R.from_euler('ZYX', rotation, degrees = True)
        # start_angle_matrix = r.as_dcm()
        # target_gelbelt_pos_difference = np.array([0,0,0])
        # T_gelbelt_old_gelbelt_new = gelbelt_old_gelbelt_new(start_angle_matrix, target_gelbelt_pos_difference)
        # T_world_gelbelt_new = np.matmul(T_world_gelbelt_old, T_gelbelt_old_gelbelt_new)
        # T_world_eef_new = np.matmul(T_world_gelbelt_new, T_eef_init_to_geleblt_inverse)
        # r = R.from_dcm(T_world_eef_new[:3,:3])
        # target_quat = r.as_quat()
        # target_pos = T_world_eef_new[:3,3]
        # robot.set_pose(target_pos, target_quat)
        # rospy.signal_shutdown("A")
        
        

        # #Moving the robot down the correct increment --> Best contact is 0.1045 --> also dependent on angle and stuff

        if(force[2] > -5):
            print(force)
            pos_difference = np.array([0,0,-0.0005])
            init_pos = init_pos + pos_difference
            robot.set_pose(init_pos, init_quat)
        # elif(force[2] < -40):
        #     print(force)
        #     pos_difference = np.array([0,0,0.00025])
        #     init_pos = init_pos + pos_difference
        #     robot.set_pose(init_pos, init_quat)
        else:
            # then set pose with correction
            print(angle)
            r  = R.from_euler('ZYX', angle, degrees = True)
            gelbelt_correction_rotMatrix = r.as_dcm()
            target_gelbelt_pos_difference = np.array([0,0,0])

            T_gelbelt_old_gelbelt_new = gelbelt_old_gelbelt_new(gelbelt_correction_rotMatrix, target_gelbelt_pos_difference)
            T_world_gelbelt_new = np.matmul(T_world_gelbelt_old, T_gelbelt_old_gelbelt_new)

            T_world_eef_new = np.matmul(T_world_gelbelt_new, T_eef_init_to_geleblt_inverse)

            r = R.from_dcm(T_world_eef_new[:3,:3])
            target_quat = r.as_quat()
            target_pos = T_world_eef_new[:3, 3]

            robot.set_pose(target_pos, target_quat)
        
  
        rospy.sleep(0.1)
        # rospy.signal_shutdown("A")





        #for angle1 in drange(x_min, x_max + 1, x_step):
        #     for angle2 in drange(y_min, y_max + 0.5, y_step):
        #         rotation = [0,angle2, angle1]
        #         r = R.from_euler('ZYX', rotation, degrees = True)
        #         start_angle_matrix = r.as_dcm()
        #         target_gelbelt_pos_difference = np.array([0,0,0])
        #         T_gelbelt_old_gelbelt_new = gelbelt_old_gelbelt_new(start_angle_matrix, target_gelbelt_pos_difference)
        #         T_world_gelbelt_new = np.matmul(T_world_gelbelt_old, T_gelbelt_old_gelbelt_new)
        #         T_world_eef_new = np.matmul(T_world_gelbelt_new, T_eef_init_to_geleblt_inverse)
        #         r = R.from_dcm(T_world_eef_new[:3,:3])
        #         target_quat = r.as_quat()
        #         target_pos = T_world_eef_new[3,:3]
        #         robot.set_pose(target_pos, target_quat)
        #         # set this rotation to rotate 

        #         #now move the gelbelt down till it makes contact with the ground 
        #         # if(init_pos[2] > 0.130):
        #         #     target_gelbelt_pos_differrence = np.array([0,0,0.005])

        #         #set pose here