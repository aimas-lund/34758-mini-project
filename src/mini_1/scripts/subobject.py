#!/usr/bin/env python
# This program publishes randomly-generated velocity
# messages for turtlesim.
import rospy
import numpy as np # For random numbers
import rostopic
 
 
from std_msgs.msg import String
from turtlesim import msg # we need to import the turtlesim msgs in order to use them
 


poses = []

#Create callback. This is what happens when a new message is received
def sub_cal(msg):
    x = msg.position.x
    y = msg.position.y
    rospy.loginfo("position=( %f, %f)", x, y)
    poses.append(msg)

def get_cube_poses():

    #pubs, subs = rostopic.get_topic_list(master=master)
    #print(pubs)

    #for p in pubs:
    rospy.Subscriber('cube0/pose', msg.Pose, sub_cal, queue_size=1000)
    
    return poses



 
#Initialize publisher
#rospy.Subscriber('turtle1/pose', msg.Pose, sub_cal, queue_size=1000)