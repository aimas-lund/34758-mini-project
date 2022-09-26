#!/usr/bin/env python

import rospy
import cube_spawn
from navigation import close_gripper, open_gripper, move_arm
from geometry_msgs.msg import Pose

def init():
  rospy.init_node('mini_1')
  cube_spawn.spawn_cubes()

def locate_cubes():
  return

if __name__ == '__main__':
  try:
    # Spawn environment
    init()

    # we should possibly add a little extra in the z-direction
    # TODO: Get these poses!
    bucket_pose = Pose()
    cube_poses = []

    # get cube poses from subscription

    for pose in cube_poses:
      # navigate to cube
      move_arm(pose)

      # grab cube
      close_gripper()

      # move cube to bucket
      move_arm(bucket_pose)

      # let cube go
      open_gripper()

  except rospy.ROSInterruptException:
    print "Something went wrong - please try again"