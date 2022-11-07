#!/usr/bin/env python
from distutils.spawn import spawn
import rospy, tf, actionlib, tf_conversions
from wander import wander
from qr_position_handler import QRHandler
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from navigator import Navigator
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *

if __name__ == '__main__':
  # initialize QR reader and final word variable
  word = ["", "", "", "", ""]
  qr_found = [False, False, False, False, False]
  qr_handler = QRHandler(word, qr_found)  
  qr_handler.qr_reader()
  # All QR markers have been found when this is true ->   all(i is True for i in QR.qr_found)

  # setup wander and Navigator objects
  wan = wander()
  nav = Navigator()        

  # init node so that topics can be published (log level logdebug,logwarn,loginfo,logerr,logfatal)
  rospy.init_node('final', log_level=rospy.DEBUG)

  # initialize navigator with ROS move_base
  nav.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  nav.client.wait_for_server()
  print('--- Navigator module initialized ---')

  # setup subscribers and publishers
  scan_sub = rospy.Subscriber('scan', LaserScan, wan.scan_callback)
  scan_sub = rospy.Subscriber('scan', LaserScan, wan.scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  state_change_time = rospy.Time.now() + rospy.Duration(1)

  #Subscribing to the topic /gazebo/model_states to read the positions of the cube and bucket
  rospy.Subscriber('/gazebo/model_states', ModelStates, nav.sub_cal, queue_size=1000)
  
  rospy.sleep(2)
  rate = rospy.Rate(60)

  # start main loop (until all QR codes are found)
  twist = Twist()
  while not rospy.is_shutdown() or not all(qr_handler.qr_found):

    robot_pose = nav.get_coordinates()
    qr_handler.update_robot_pose(robot_pose)

    # if no single QR code has been found randomly wander around
    if not any(qr_handler.qr_found):
      rospy.logdebug("wandering")

      twist = wan.move(not qr_handler.stop_wandering)
      cmd_vel_pub.publish(twist)

    # once at leastone QR has been found navigate to the next QR
    else:
      rospy.logdebug("QR found, moving to next QR: " + str(qr_handler.next_qr))

      next_qr = qr_handler.next_qr  
      #nav.move_to_pose(next_qr)

    rate.sleep()

  rospy.loginfo("All QR's have been discovered!")
  rospy.loginfo("Code word was: " + qr_handler.print_word())
  