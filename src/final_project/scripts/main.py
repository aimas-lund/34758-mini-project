#!/usr/bin/env python
from distutils.spawn import spawn
import rospy
from wander import wander
from qr_position_handler import QRkanker
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from navigator import Navigator

if __name__ == '__main__':
  
  word = ["", "", "", "", ""]
  qr_found = [False, False, False, False, False]
  QR = QRkanker(word,qr_found,0,0)  
  QR.qr_reader()

  # All QR markers have been found when this is true ->   all(i is True for i in QR.qr_found)

  wander = wander()
  nav = Navigator()

  rospy.init_node('final')  

  scan_sub = rospy.Subscriber('scan', LaserScan, nav.scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  state_change_time = rospy.Time.now() + rospy.Duration(1)
  
  rospy.sleep(2)
  rate = rospy.Rate(60)

  twist = Twist()
  while not rospy.is_shutdown():
    while True:
      twist = nav.move(True)
      cmd_vel_pub.publish(twist)
    #for pose in nav._waypoints:   
    #    nav.move_to_pose(pose)
    #rate.sleep()

  