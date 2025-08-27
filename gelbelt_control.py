# from easyUR import UR
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

def pid_controller(Kp, Ki, Kd, setpoint, measurement, final_time):
    global integral, previous_error, intial_time
    error = setpoint - measurement
    p = Kp*error
    integral = integral + Ki*error*(final_time - intial_time + 0.1)
    d = Kd*(error - previous_error)/(final_time - intial_time + 0.1)
    mv = p + integral + d
    previous_error = error
    intial_time = final_time
    return mv



if __name__ == '__main__':
    rtde_c = rtde_control.RTDEControlInterface("192.168.1.12")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.12")
    # print(ur_rtde)
    starting = rtde_r.getActualTCPPose()

    t = np.array([0,0,0.130])
    angle = np.array([0,0,0])
    force = np.array([0,0,0])

    rospy.Subscriber('/target_angle', std_msgs.msg.Float32MultiArray, angle_callback, queue_size=1)
    rospy.Subscriber('/wrist_sensor/wrench', geometry_msgs.msg.WrenchStamped, force_callback, queue_size=1)

    init_force = force[2]
  
    while not rospy.is_shutdown():


        #Getting the matrix intialized
        init = rtde_r.getActualTCPPose()
        init_pos = init[0:3]
        init_rot = init[3:]

        r = R.from_rotvec(init_rot)
        init_euler_eef = r.as_euler('ZYX', degrees = True)
        init_rotMatrix_eef = r.as_dcm() 

        T_world_eef_init = world_eef_init(init_rotMatrix_eef, init_pos)
        T_eef_init_to_gelbelt = end_effector_to_gelbelt(t)
        T_eef_init_to_geleblt_inverse = inverse_end_effector_to_gelbelt(t)


        init_gelbelt_rotMatrix = np.matmul(T_world_eef_init, T_eef_init_to_gelbelt)[:3, :3]
        init_gelbelt_pos = np.matmul(T_world_eef_init, T_eef_init_to_gelbelt)[:3, 3]

        T_world_gelbelt_old = world_gelbelt_init(init_gelbelt_rotMatrix, init_gelbelt_pos)
        
        print(force[2])
        print(init_force)
        print(angle)

        # #Moving the robot down the correct increment --> Best contact is 0.1045 --> also dependent on angle and stuff

        if(force[2] - init_force > -45):
            print("moving down")
            pos_difference = np.array([0,0,-0.0005])
            target_pos = init_pos + pos_difference
            target = np.concatenate((target_pos, init_rot))
            rtde_c.servoL(target,0.01, 0.01, 0.1,0.05, 100)
            print(force[2] - init_force)
        elif(force[2] - init_force < -50):
            print("moving up")
            pos_difference = np.array([0,0,0.0005])
            target_pos = init_pos + pos_difference
            target = np.concatenate((target_pos, init_rot))
            rtde_c.servoL(target, 0.01, 0.01, 0.1, 0.05, 100)
        else:
            print("correcting")
            # then set pose with correction
            x_rotation = pid_controller(Kp=0.1, Ki=0, Kd=0, setpoint=0, measurement=angle[2], final_time=0)
            y_rotation = pid_controller(Kp=0.1, Ki = 0, Kd= 0, setpoint= 0, measurement=angle[1], final_time=0)
            angle = [0,y_rotation, x_rotation]
            r  = R.from_euler('ZYX', angle, degrees = True)
            gelbelt_correction_rotMatrix = r.as_dcm()
            target_gelbelt_pos_difference = np.array([0,0,0])

            T_gelbelt_old_gelbelt_new = gelbelt_old_gelbelt_new(gelbelt_correction_rotMatrix, target_gelbelt_pos_difference)
            T_world_gelbelt_new = np.matmul(T_world_gelbelt_old, T_gelbelt_old_gelbelt_new)

            T_world_eef_new = np.matmul(T_world_gelbelt_new, T_eef_init_to_geleblt_inverse)

            r = R.from_dcm(T_world_eef_new[:3,:3])
            target_rotvec = r.as_rotvec()
            target_pos = T_world_eef_new[:3, 3]
            target = np.concatenate((target_pos, target_rotvec))
            rtde_c.servoL(target, 0.01, 0.01, 0.1, 0.05, 100)
        
  
        # rospy.sleep(0.1)
        # rospy.signal_shutdown("A")