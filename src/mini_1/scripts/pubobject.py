#!/usr/bin/env python
# This program publishes the item_name and item_pose of a spawned cube
# messages for turtlesim.
import rospy
import numpy as np # For random numbers
 
 
from std_msgs.msg import String
from geometry_msgs.msg import Pose


def publish_object(item_name,item_pose):

    #Initialize publisher
    p = rospy.Publisher(item_name+'/pose', Pose, queue_size=1000)
    
    # Initialize node
    r = rospy.Rate(2) # Set Frequency
    
    #ROS INFO of the commands
    rospy.loginfo("{0}.position.x is {1}".format(item_name, item_pose.position.x))
    rospy.loginfo("{0}.position.y is {1}".format(item_name, item_pose.position.y))
    
    #publish the message
    p.publish(item_pose)
    r.sleep()