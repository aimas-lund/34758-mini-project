#!/usr/bin/env python

import rospy
import cube_spawn
import subobject

from navigation import Arm
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelStates

import sys

def init():
  rospy.init_node('mini_1')
  cube_spawn.spawn_cubes()

if __name__ == '__main__':
  try:
    # Spawn environment
    init()

    arm = Arm()
    arm.move()

  except rospy.ROSInterruptException:
    print("Something went wrong - please try again")