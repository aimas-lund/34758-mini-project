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

"""
- - - software structure - - -

main.py:
- initialize objects
- main loop
  - looking for first QR
  - looking for next QR
  - done

navigator.py:
- keep track of robot position
- navigate to positio

qr_position_handler.py:
- reading QR codes
- using util for coordinate conversion

final_util.py:
- coordinate conversion
- coordinate system calculation

- - - other - - -
key_navigation.py:  control robot manually
key_publisher.py:   control robot manually
range_ahead.py:     see how much distance is free in front of robot
spawn_barriers.py:  spawn barriers
spawn_markers.py:   spawn markers
wander.py:          move around randomly
"""

if __name__ == '__main__':
  # initialize QR reader and final word variable
  qr_handler = QRHandler()  
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

  # setup subscribers and publishers
  scan_sub = rospy.Subscriber('scan', LaserScan, wan.scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  state_change_time = rospy.Time.now() + rospy.Duration(1)

  #Subscribing to the topic /gazebo/model_states to read the positions of the cube and bucket
  rospy.Subscriber('/gazebo/model_states', ModelStates, nav.sub_cal, queue_size=1000)
  
  rospy.sleep(2)
  rate = rospy.Rate(60)

  # start main loop (until all QR codes are found)
  twist = Twist()
  for pose in nav._waypoints:
    nav.move_to_pose(pose)

  while not rospy.is_shutdown():    
    if sum(qr_handler.qr_found)<2:
      twist = wan.move(True)
      cmd_vel_pub.publish(twist)
    else:
        print(qr_handler.qr_found)
        qr_handler.map_qr_transformation()
        current_x, current_y, next_x, next_y = qr_handler.getNextPos()
        print(current_x, current_y, next_x, next_y)
        x_target,y_target = qr_handler.calculate_QR_XY(next_x, next_y)
        print(x_target,y_target)
        x_target,y_target, rot = qr_handler.calculate_destination(x_target, y_target)
        print(x_target,y_target,rot)
        pose = [( round(x_target, 2), round(y_target, 2), 0.0), (0.0, 0.0, 0.0, 1.0)]
        print(pose)
        nav.move_to_pose(pose)
        print('#####################################################')

    rate.sleep()

  rospy.loginfo("All QR's have been discovered!")
  rospy.loginfo("Code word was: " + qr_handler.print_word())
  