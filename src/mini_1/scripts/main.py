#!/usr/bin/env python

import rospy
import cube_spawn
import subobject
import getcoordinates

from navigation import Arm
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties

import sys

def init():
  rospy.init_node('mini_1')
  cube_spawn.spawn_cubes()


if __name__ == '__main__':
  try:
    # Spawn environment
    init()

    # we should possibly add a little extra in the z-direction
    # TODO: Get these poses!

    world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    model_list = world_properties()

    cubes = []
    for item in model_list.model_names:
      if 'cube' in item:
        cubes.append(item)

    #bucket_pose = Pose() 

    # get cube poses from subscription
    arm = Arm()
    pos = Pose()
    pos.position.x = 0
    pos.position.y = 0
    pos.position.z = 1
    arm.move_two_step(pos, z_offset=0.1)
    print('######################')
    arm.move_two_step(getcoordinates.get_model_coordinates('jaco_on_table::jaco_8_finger_thumb'), z_offset=0.5)
    print('######################')
    arm.move_two_step(getcoordinates.get_model_coordinates('jaco_on_table::jaco_8_finger_thumb'), z_offset=-1)
    print('######################')
    arm.move_two_step(getcoordinates.get_model_coordinates('jaco_on_table::jaco_8_finger_thumb'), z_offset=-0.5)
    for cube in cubes:
      # navigate to cube
      # get cube pos
      pos = getcoordinates.get_model_coordinates(cube)
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