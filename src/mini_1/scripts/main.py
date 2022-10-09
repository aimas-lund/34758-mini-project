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

    arm = Arm()
    arm.move()

  except rospy.ROSInterruptException:
    print("Something went wrong - please try again")