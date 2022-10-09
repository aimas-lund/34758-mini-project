#!/usr/bin/env python
import roslib
roslib.load_manifest('mini_1')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
 
global message
global bucket_place # tuple
global pub

# sets the fingers joints values to open or close the gripper
# used in point 5 and 8 on flow diagram
def move_gripper(tmp):
  currentJointState = rospy.wait_for_message("/joint_states",JointState, 10)
  currentJointState.header.stamp = rospy.get_rostime()
  currentJointState.position = tuple(list(currentJointState.position[:6]) 
                + [tmp] + [tmp]+ [tmp])
  for i in range(3):
    pub.publish(currentJointState)
    rospy.sleep(0.1)
  rospy.sleep(2)

def open_gripper():
  move_gripper(0.005)

def close_gripper():
  move_gripper(0.7)