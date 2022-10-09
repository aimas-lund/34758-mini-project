#!/usr/bin/env python

from distutils.spawn import spawn
import rospy
import subobject
import getcoordinates
import sys
import time

from navigation import Arm
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelStates
from cube_spawn import Spawner
from argument_handler import ArgumentHandler

spawner = Spawner()
arg_handler = ArgumentHandler()

def init(refresh=False):
  rospy.init_node('mini_1')

  if not refresh:
    spawner.spawn_models()
    print("Spawned Models!")
    return
    
  spawner.despawn_cubes()
  spawner.spawn_cubes()


if __name__ == '__main__':
  try:
    # Spawn environment
    init(arg_handler.should_run_refresh_mode())

    # we should possibly add a little extra in the z-direction
    # TODO: Get these poses!

    #   world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    #   print(world_properties)
    #   model_list = world_properties()
    #   print(model_list)

    #   cubes = []
    #   for item in model_list.model_names:
    #     if 'cube' in item:
    #       cubes.append(item)

    #   #bucket_pose = Pose() 

    #   # get cube poses from subscription
    arm = Arm()
     #   pos = Pose()
    arm.move()


  #   model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  #   pose = model_coordinates('ground_plane','jaco_on_table::jaco_6_hand_limb')
  #   pose = pose.pose
  #  # print(pose.position)
  #   pose.position.z = pose.position.z+0.4
  #   #arm.move(pose)
  #   #print('######################')
  #   model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  #   new = model_coordinates('ground_plane','jaco_on_table::jaco_6_hand_limb')
  #   new = new.pose
  #   #print(new.position)
  #   #print(round(pose.position.y - new.position.y,4))
  #   #print(round(pose.position.x - new.position.x,4))
  #   #print(round(pose.position.z - new.position.z,4))
  #   for cube in cubes:
  #     # navigate to cube
  #     # get cube pos
  #     pos = getcoordinates.get_model_coordinates(cube)
      #arm.move_two_step(pos, z_offset=1)

      # grab cube
      # arm.close_gripper()

      # move cube to bucket
      #bucket_pose = getcoordinates.get_model_coordinates('bucket')
      # arm.move_two_step(bucket_pose)

      # let cube go
      #arm.open_gripper()

  except rospy.ROSInterruptException:
    print("Something went wrong - please try again")