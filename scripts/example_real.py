#! /usr/bin/env python

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

rospy.init_node("planning")

pub = rospy.Publisher("/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size = 10)

rate = rospy.Rate(10)

joint_value = np.loadtxt("joint_value")

i = 0
while not rospy.is_shutdown():
    
    joint = JointTrajectory()
    joint.header.stamp = rospy.Time.now()
    joint.header.frame_id = ''
    joint.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    point = JointTrajectoryPoint()
    # value = joint_value[i].arr
    point.positions = [((joint_value[i][1]-180)*np.pi)/180, ((joint_value[i][0]-180)*np.pi)/180, 0, -np.pi/2, 0, 0]
    # point.positions = [(value[1]-180)/np.pi, (value[0]-180)/np.pi, 0, -np.pi/2, 0, 0]
    point.velocities = []
    point.accelerations = []
    point.effort = []
    point.time_from_start = rospy.Duration(1)

    joint.points.append(point)

    
    pub.publish(joint)
    if i == len(joint_value)-1:
        i=i

    i+=1
    rate.sleep()