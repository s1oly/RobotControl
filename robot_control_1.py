#!/usr/bin/env python

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


def follow_trajectory(pos_publisher, joint_space_goals, times):

    pos_message = JointTrajectory()
    pos_message.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    
    for joint_space_goal, time in zip(joint_space_goals, times):
        pos_message_point = JointTrajectoryPoint()
        pos_message_point.positions = joint_space_goal
        pos_message_point.time_from_start = rospy.Duration(time)
        pos_message.points.append(pos_message_point)

    pos_publisher.publish(pos_message)

    
def move_to_joint(pos_publisher, joint_space_goal, time = 2.0):

    pos_message = JointTrajectory()

    pos_message.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    pos_message_point = JointTrajectoryPoint()
    pos_message_point.positions = joint_space_goal
    pos_message_point.time_from_start = rospy.Duration(time)
    pos_message.points.append(pos_message_point)

    pos_publisher.publish(pos_message)

joint_states = None

def cb_joint_states (msg):
    global joint_states
    joint_states= np.array(msg.position)


if __name__ == '__main__':

    rospy.init_node('test')
    pos_publisher = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=20)
    pos_subsriber = rospy.Subscriber('/joint_states', JointState, cb_joint_states)
    rospy.sleep(1.0)  # this has to be here otherwise it doesn't work looool

    joint_init = joint_states.copy()

    # rotate joint 4 by -0.1 radians
    joint_target = joint_init + np.array([0.0, 0.0, 0.0, -0.1, 0.0, 0.0])

    # send move command; it should take 5 seconds to complete this move
    move_to_joint(pos_publisher, joint_target, time=5.0)
    
    # always sleep the same amount of time since move_to_joint is non-blocking
    rospy.sleep(5.0)

    # rotate joint 4 back
    joint_target = joint_init

    move_to_joint(pos_publisher, joint_target, time=5.0)
    rospy.sleep(5.0)