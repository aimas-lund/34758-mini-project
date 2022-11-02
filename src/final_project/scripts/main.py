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
  
  word = ["", "", "", "", ""]
  qr_found = [False, False, False, False, False]
  QR = QRHandler(word,qr_found,0,0)  
  QR.qr_reader()

  # All QR markers have been found when this is true ->   all(i is True for i in QR.qr_found)

  wander = wander()
  nav = Navigator()        

  rospy.init_node('final')  

  nav.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  nav.client.wait_for_server()
  print('--- Navigator module initialized ---')

  scan_sub = rospy.Subscriber('scan', LaserScan, wander.scan_callback)
  scan_sub = rospy.Subscriber('scan', LaserScan, wander.scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  state_change_time = rospy.Time.now() + rospy.Duration(1)

  #Subscribing to the topic /gazebo/model_states to read the positions of the cube and bucket
  rospy.Subscriber('/gazebo/model_states', ModelStates, nav.sub_cal , 
                                                        queue_size=1000)
  
  rospy.sleep(2)
  rate = rospy.Rate(60)

  twist = Twist()
  robot_pose = geometry_msgs.msg.PoseStamped()
  while not rospy.is_shutdown() or not all(QR.qr_found):
    if not any(QR.qr_found):
      print('wander')
      twist = wander.move(True)
      cmd_vel_pub.publish(twist)
    else:
      print('qr found')
      next_qr = QR.next_qr  
      print('##################################')
      print(next_qr)
      nav.move_to_pose(next_qr)
      robot_pose = nav.get_coordinates(robot_pose)
      print('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$')
      print(robot_pose)
    rate.sleep()

  print("All QR's have been discovered!")
  print("Code word was")
  QR.print_word()

  