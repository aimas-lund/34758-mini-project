#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class wander:
  """
  randomly drive around the map without bumping into objects
  """


  def __init__(self):
    self.g_range_ahead = 1

  def scan_callback(self,msg):
    tmp=[msg.ranges[0]]
    for i in range(1,21):
      tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
      tmp.append(msg.ranges[i])
    self.g_range_ahead = min(tmp)
 

  def move(self,drive = False):
    driving_forward = True
    
    twist = Twist()
    if drive:
      if self.g_range_ahead < 0.8:
        driving_forward = False
      
      else: # we're not driving_forward
        driving_forward = True # we're done spinning, time to go forward!
      
      if driving_forward:
        twist.linear.x = 0.4
        twist.angular.z = 0.0
      else:
        twist.linear.x = 0.0
        twist.angular.z = 0.4
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    return twist