#!/usr/bin/env python

from distutils.spawn import spawn
import rospy
import wander
import sys
import time

def init():
  rospy.init_node('final')

if __name__ == '__main__':
  try:
      init()
      while True:
        wander.move(True)

  except rospy.ROSInterruptException:
    print("Something went wrong - please try again")