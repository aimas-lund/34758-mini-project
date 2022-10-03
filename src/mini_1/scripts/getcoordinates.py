#!/usr/bin/env python
# This program publishes the item_name and item_pose of a spawned cube
# messages for turtlesim.
import rospy
import numpy as np # For random numbers
 
 
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties


def get_model_coordinates(model_name):
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
        pose = model_coordinates('world',model_name)
        return(pose.pose)
    except rospy.ServiceException as e:
        print("Service call failed:"+e)

        