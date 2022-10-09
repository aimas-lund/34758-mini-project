#!/usr/bin/env python
# This program publishes the item_name and item_pose of a spawned cube
# messages for turtlesim.
import rospy
import numpy as np # For random numbers
from geometry_msgs.msg import *
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
import tf_conversions
 
 
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties

global message
def sub_cal(msg):
    print('########################')
    message = msg


if __name__ == '__main__':
    #Subscribing to the topic /gazebo/model_states to read the positions of the cube and bucket
    model =rospy.Subscriber('/gazebo/model_states', ModelStates, sub_cal , 
                                                            queue_size=1000)
    message = model
    rospy.sleep(3)
    print('@@@@@@@@@@@@@@@@@')

    print(message.name )