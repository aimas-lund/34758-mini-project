#!/usr/bin/env python
# This program publishes the item_name and item_pose of a spawned cube
# messages for turtlesim.
import rospy
import numpy as np # For random numbers
from geometry_msgs.msg import *
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
   
 
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties

class model_coordinates:
    def sub_cal(self,msg):
        self.message = msg

    def __init__(self):
        self.bucket_pos = 0

    #check if cube is in the bucket be comparing the distance
    def cube_in_bucket(self,cube_name):
        distance = 100.0
        for i in range(len(self.message.name)):
            if self.message.name[i] == cube_name:
                distance = np.sqrt(
                                np.power((self.message.pose[i].position.x - 
                                            self.bucket_pos.position.x),2) +  
                                np.power(((self.message.pose[i].position.y - 
                                            self.bucket_pos.position.y)),2) ) 
                rospy.loginfo(distance)
                if(distance < 0.15):
                    return True
        return False 

    def get_coordinates(self,p, scene):
        cube_place = [] # array of tuples
        cube_name=[]

        for i in range(len(self.message.name)):

            if self.message.name[i] == "bucket":
                p.pose.position.x = self.message.pose[i].position.x
                p.pose.position.y = self.message.pose[i].position.y
                p.pose.position.z = self.message.pose[i].position.z + 0.09
                #0.09 is used as the bucket height is 0.18 and the reference frame is found in the middle 
                q = Quaternion(*tf_conversions.transformations.quaternion_from_euler(
                                                                0., 0.0, 0.785398))
                p.pose.orientation.x = q.x
                p.pose.orientation.y = q.y
                p.pose.orientation.z = q.z
                p.pose.orientation.w = q.w
                # Add the bucket position to the bucket_pos tuple
                self.bucket_pos= self.message.pose[i] 
                
                # Add box to MoveIt! scene
                scene.add_box(self.message.name[i], p, (0.21, 0.21, 0.18)) 

            if "cube" in self.message.name[i]:
                p.pose.position.x = self.message.pose[i].position.x
                p.pose.position.y = self.message.pose[i].position.y
                p.pose.position.z = self.message.pose[i].position.z
                p.pose.orientation.x = self.message.pose[i].orientation.x
                p.pose.orientation.y = self.message.pose[i].orientation.y 
                p.pose.orientation.z = self.message.pose[i].orientation.z 
                p.pose.orientation.w = self.message.pose[i].orientation.w
                # Add the cube position to the cube_place tuple
                cube_place.append(self.message.pose[i])
                # Add the cube name to the cube_name tuple
                cube_name.append(self.message.name[i])
                #add box to MoveIt! scene
                scene.add_box(self.message.name[i], p, (0.05, 0.05, 0.05)) 
        
        return cube_place, cube_name