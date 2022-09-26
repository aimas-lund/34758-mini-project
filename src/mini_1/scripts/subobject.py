#!/usr/bin/env python
# This program publishes randomly-generated velocity
# messages for turtlesim.
import rospy
import numpy as np # For random numbers
 
 
from std_msgs.msg import String
from turtlesim import msg # we need to import hte turtlesim msgs in order to use them
 

#I AAAAAAM AA WOOOORK IN PROOOOGREEEESSS i.e. this file is fucked up


#Create callback. This is what happens when a new message is received
def sub_cal(msg):
    x = msg.position.x
    y = msg.position.y
    rospy.loginfo("position=( %f, %f)", x, y)
    global pos_x
    global pos_y
    pos_x = x
    pos_y = y
 
#Initialize publisher
rospy.Subscriber('turtle1/pose', msg.Pose, sub_cal, queue_size=1000)
 
# Initialize node
rospy.init_node('cmd_vel_listener')
rospy.spin()
rospy.loginfo()