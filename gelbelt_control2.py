import geometry_msgs.msg
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
from ros_numpy import numpify
import geometry_msgs
import copy
import time

DH_theta = np.array([0,0,0,0,0,0])
DH_a = np.array([0,-0.425,-0.39225,0,0,0])
DH_d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.082])
DH_alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
sensor_width = 0.060
sensor_length = 0.105

integral = 0
previous_error = 0
intial_time = 0



class Gelbelt(object):
    def __init__(self):
        self.rate = rospy.Rate(100)

        #self.up_pose = np.array()

        # States
        self.joint_state = None
        self.joint_state_new = None
        self.T = None
        # Data
        self.force = [0,0,0]
        self.offset = []
        self.angle = [0,0,0]

        # Subscribers
        rospy.Subscriber('/wrist_sensor/wrench', geometry_msgs.msg.WrenchStamped, self.force_callback)
        rospy.Subscriber('/joint_states', JointState, self.cb_joint_states)
        rospy.Subscriber('/target_angle', std_msgs.msg.Float32MultiArray, self.angle_callback)
        # Publishers
        self.pos_controller = rospy.Publisher('/scaled_pos_joint_traj_controller/command',
                                                JointTrajectory, queue_size=20)

    def cb_joint_states(self, msg): # updates joint_state
        # self.joint_state = np.array(msg.position)
        self.joint_state = np.array([msg.position[2], msg.position[1], msg.position[0],
                                        msg.position[3], msg.position[4], msg.position[5]])
        
    def force_callback(self, msg):
        self.force = numpify(msg.wrench.force) # have to figure out weight in air without contact with table and then subtract that to get a 0

    def angle_callback(self, msg):
        self.angle = np.array(msg.data)

    # def cb_force_torque(self, msg): # updates force_torque
    #     """ Force Torque data callback. """
    
        # self.force_torque = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
        #                      msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        # if self.offset != []:
        #     for i in range(6):
        #         self.force_torque[i] = self.force_torque[i] - self.offset[i]

    def forward_kinematic(self, joint_angle): # updates T
        self.T = np.eye(4)
        for theta_idx in range(0,6):
            stheta = np.sin(joint_angle[theta_idx])
            ctheta = np.cos(joint_angle[theta_idx])
            salpha  =np.sin(DH_alpha[theta_idx])
            calpha = np.cos(DH_alpha[theta_idx])
            self.T = np.dot(self.T,np.array([[ctheta, -stheta*calpha, stheta*salpha, DH_a[theta_idx]*ctheta],
                                             [stheta, ctheta*calpha, -ctheta*salpha, DH_a[theta_idx]*stheta],
                                             [0, salpha, calpha, DH_d[theta_idx]],
                                             [0,0,0,1]]))

    def inverse_kinematic (self,current_joint_angle, target_pos, rotate_R):
        """
        calculate the inverse kinematic of UR5E
        :param current_joint_angle: ndarray; the current 6 joint angles
        :param target_pos: ndarray; the target position of the end-effector
        :param rotate_R: ndarray; the target orientation of the end-effector

        nx,ox,ax;
        ny,oy,ay;
        nz,oz,az;

        """

        self.joint_state_new =  copy.deepcopy(current_joint_angle)
        nx = rotate_R[0,0]
        ny = rotate_R[1,0]
        nz = rotate_R[2,0]
        ox = rotate_R[0,1]
        oy = rotate_R[1,1]
        oz = rotate_R[2,1]
        ax = rotate_R[0,2]
        ay = rotate_R[1,2]
        az = rotate_R[2,2]
        
        m = DH_d[5]*ay-target_pos[1]
        n = DH_d[5]*ax - target_pos[0]
        # print("m,n",m,n,m**2+n**2-DH_d[3]**2)
        theta1_1 = np.arctan2(m,n) - np.arctan2(DH_d[3], np.sqrt(m**2+n**2-DH_d[3]**2))
        theta1_2 = np.arctan2(m,n) - np.arctan2(DH_d[3], -np.sqrt(m**2+n**2-DH_d[3]**2))
        if theta1_1<-np.pi:
            theta1_1 += 2*np.pi
        if theta1_2 < np.pi:
            theta1_2 += 2*np.pi
        # print(theta1_1,theta1_2)
        if abs(self.joint_state_new[0]-theta1_1)<abs(self.joint_state_new[0]-theta1_2):
            self.joint_state_new[0] = theta1_1
        else:
            self.joint_state_new[0] = theta1_2

        theta5_1 = np.arccos(ax*np.sin(self.joint_state_new[0])-ay*np.cos(self.joint_state_new[0]))
        theta5_2 = -theta5_1
        # print(theta5_1,theta5_2)
        if abs(self.joint_state_new[4]-theta5_1)<abs(self.joint_state_new[4]-theta5_2):
            self.joint_state_new[4] = theta5_1
        else:
            self.joint_state_new[4] = theta5_2

        mm = nx*np.sin(self.joint_state_new[0]) - ny*np.cos((self.joint_state_new[0]))
        nn = ox*np.sin(self.joint_state_new[0]) - oy*np.cos((self.joint_state_new[0]))
        self.joint_state_new[5] = np.arctan2(mm,nn)-np.arctan2(np.sin(self.joint_state_new[4]),0)


        mmm = DH_d[4]*(np.sin(self.joint_state_new[5])*(nx*np.cos(self.joint_state_new[0])+ny*np.sin(self.joint_state_new[0]))+
                       np.cos(self.joint_state_new[5])*(ox*np.cos(self.joint_state_new[0])+oy*np.sin(self.joint_state_new[0])))+\
                        target_pos[0]*np.cos(self.joint_state_new[0])- DH_d[5]*(ax*np.cos(self.joint_state_new[0])+ ay*np.sin(self.joint_state_new[0]))+\
                            target_pos[1]*np.sin(self.joint_state_new[0])
        nnn = DH_d[4]* (oz*np.cos(self.joint_state_new[5])+nz*np.sin(self.joint_state_new[5])) + target_pos[2] - DH_d[0] - az*DH_d[5]
        theta3_1 = np.arccos((mmm**2+nnn**2-DH_a[1]**2-DH_a[2]**2)/(2*DH_a[1]*DH_a[2]))
        theta3_2 = -theta3_1
        # print(theta3_1,theta3_2)

        if abs(self.joint_state_new[2]-theta3_1)<abs(self.joint_state_new[2]-theta3_2):
            self.joint_state_new[2] = theta3_1
        else:
            self.joint_state_new[2] = theta3_2


        s2 = ((DH_a[2]*np.cos(self.joint_state_new[2])+DH_a[1])*nnn - DH_a[2]*np.sin(self.joint_state_new[2])*mmm)/(DH_a[1]**2+DH_a[2]**2+2*DH_a[1]*DH_a[2]*np.cos(self.joint_state_new[2]))
        c2 = (mmm+DH_a[2]*np.sin(self.joint_state_new[2])*s2)/(DH_a[2]*np.cos(self.joint_state_new[2])+DH_a[1])
        self.joint_state_new[1] =np.arctan2(s2,c2)
        # print("theta2",self.joint_state_new[1])
        # self.joint_state_new[1] = np.arctan(((DH_a[2]*np.cos(self.joint_state_new[2])+DH_a[1])*nnn - DH_a[2]*np.sin(self.joint_state_new[2])*mmm)/(mmm*(DH_a[1]+DH_a[2]*np.cos(self.joint_state_new[2]))+nnn*DH_a[2]*np.sin(self.joint_state_new[2])))
        # print("theta2",self.joint_state_new[1])

        self.joint_state_new[3] = np.arctan2(-np.sin(self.joint_state_new[5])*(nx*np.cos(self.joint_state_new[0])+ny*np.sin(self.joint_state_new[0]))
                                             -np.cos(self.joint_state_new[5])*(ox*np.cos(self.joint_state_new[0])+oy*np.sin(self.joint_state_new[0])),
                                             oz*np.cos(self.joint_state_new[5])+nz*np.sin(self.joint_state_new[5])) - self.joint_state_new[1] - self.joint_state_new[2]
        


        if np.isnan(self.joint_state_new[0]) or np.isnan(self.joint_state_new[1]) or np.isnan(self.joint_state_new[2]) or np.isnan(self.joint_state_new[3]) or np.isnan(self.joint_state_new[4]) or np.isnan(self.joint_state_new[5]):
            raise ValueError("Input position is unreachable")

    def publish(self, duration, sleeptime):
        pos_message = JointTrajectory()
        # pos_message.header.stamp = rospy.Time.now()
        pos_message.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        pos_message.points.append(JointTrajectoryPoint(positions=self.joint_state_new , time_from_start=rospy.Duration(duration)))
        self.pos_controller.publish(pos_message)
        rospy.sleep(sleeptime)

    def rotate_x(self, angle, initial_T):
        tetx = np.radians(angle)
        # z_offset = (sensor_width/2*np.sin(tetx) + sensor_height * (np.cos(tetx)-1))* 0.9
        z_offset = sensor_width / 2* abs(np.sin(tetx))
        
        Mx = np.array([[1.0, 0.0, 0.0, 0.0],
                       [0.0, np.cos(tetx), -np.sin(tetx), 0.0],
                       [0.0, np.sin(tetx), np.cos(tetx), 0.0],
                       [0.0, 0.0, 0.0, 1.0]])
        
        new_T = np.dot(initial_T, Mx)
        
        return z_offset, new_T
    
    def rotate_y(self, angle, initial_T):
        tety = np.radians(angle)
        z_offset = sensor_length / 2 * abs(np.sin(tety))
        My = np.array([[np.cos(tety), 0.0, np.sin(tety), 0.0],
                       [0.0, 1.0, 0.0, 0.0],
                       [-np.sin(tety), 0.0, np.cos(tety), 0.0],
                       [0.0, 0.0, 0.0, 1.0]])
        new_T = np.dot(initial_T, My)
        
        return z_offset, new_T

    def rotate_xy(self, angle_x, angle_y, initial_T):
        tetx = np.radians(angle_x)
        tety = np.radians(angle_y)

        Mx = np.array([[1.0, 0.0, 0.0, 0.0],
                       [0.0, np.cos(tetx), -np.sin(tetx), 0.0],
                       [0.0, np.sin(tetx), np.cos(tetx), 0.0],
                       [0.0, 0.0, 0.0, 1.0]])
        
        My = np.array([[np.cos(tety), 0.0, np.sin(tety), 0.0],
                       [0.0, 1.0, 0.0, 0.0],
                       [-np.sin(tety), 0.0, np.cos(tety), 0.0],
                       [0.0, 0.0, 0.0, 1.0]])
        
        new_T = np.dot(np.dot(initial_T, Mx), My)

        offset_x = sensor_width / 2 * abs(np.sin(tetx))
        offset_y = sensor_length / 2 * abs(np.sin(tety))
        angle_offset = np.sqrt(offset_x**2 + offset_y**2)

        return angle_offset, new_T
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
    rospy.init_node('control')
    rospy.sleep(1)

    gelbelt = Gelbelt()
    rospy.sleep(1)

    init_force_z = gelbelt.force[2]

    while not rospy.is_shutdown():
        try:
            gelbelt.forward_kinematic(gelbelt.joint_state)
            # print(init_matrix)

            # x_rotation = 10
            # y_rotation = -3
            # angle_offset, new_T = gelbelt.rotate_xy(x_rotation, y_rotation, gelbelt.T)
            # gelbelt.inverse_kinematic(gelbelt.joint_state, target_pos= new_T[:3, 3], rotate_R=new_T[:3, :3])
            # gelbelt.publish(duration=1, sleeptime=2)


            # rospy.signal_shutdown("A")


            #TODO
            #add boundary cases such that the robot doesn't move unexpectedly, but have to figure out where it makes most sense in code

            #-50 -55

            # print(gelbelt.angle)

            if(gelbelt.force[2] - init_force_z > -45):
                # print(gelbelt.force)
                print("down")
                gelbelt.inverse_kinematic(gelbelt.joint_state, target_pos= gelbelt.T[:3, 3] + np.array([0,0,-0.0005]), rotate_R= gelbelt.T[:3, :3])
                gelbelt.publish(duration=0.2, sleeptime= 0.2)
            elif(gelbelt.force[2] - init_force_z < -50):
                print("up")
                gelbelt.inverse_kinematic(gelbelt.joint_state, target_pos= gelbelt.T[:3, 3] + np.array([0,0,0.0005]), rotate_R= gelbelt.T[:3, :3])
                gelbelt.publish(duration=0.2, sleeptime=0.2)
            else:
                current_time = time.time()
                x_rotation = pid_controller(Kp=0.1, Ki=0, Kd=0, setpoint=0, measurement=gelbelt.angle[2], final_time=current_time)
                y_rotation = pid_controller(Kp=0.1, Ki = 0, Kd= 0, setpoint= 0, measurement=gelbelt.angle[1], final_time=current_time)
                angle_offset, new_T = gelbelt.rotate_xy(x_rotation, y_rotation, gelbelt.T)
                gelbelt.inverse_kinematic(gelbelt.joint_state, target_pos=new_T[:3, 3], rotate_R=new_T[:3,:3])
                gelbelt.publish(duration=0.2, sleeptime=0.1)
        except:
            print("Error Happened")
            continue

