#!/usr/bin/env python
from distutils.spawn import spawn
import rospy, tf, actionlib, tf_conversions
from wander import wander
from qr_position_handler import QRHandler
import os
import argparse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from navigator import Navigator
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
import tf
import tf_conversions
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, concatenate_matrices, translation_matrix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String
from nav_msgs.msg import Odometry

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
- navigate to position

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

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--debug", action='store_true', 
                    help = "Configure extra publishers for more information in rviz for debugging purposes.")
args = parser.parse_args()

if __name__ == '__main__':
  # init node so that topics can be published (log level logdebug,logwarn,loginfo,logerr,logfatal)
  rospy.init_node('final', log_level=rospy.DEBUG)

  # init transformlistener
  listener = tf.TransformListener()

  # initialize QR reader and final word variable
  qr_handler = QRHandler(listener)  
  qr_handler.qr_reader()
  # All QR markers have been found when this is true ->   all(i is True for i in QR.qr_found)

  # setup wander and Navigator objects
  wan = wander()
  nav = Navigator()        

  # initialize navigator with ROS move_base
  nav.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  nav.client.wait_for_server()
  rospy.loginfo('--- Navigator module initialized ---')

  # setup subscribers and publishers
  scan_sub = rospy.Subscriber('scan', LaserScan, wan.scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  state_change_time = rospy.Time.now() + rospy.Duration(1)

  #Subscribing to the topic /gazebo/model_states to read the positions of the cube and bucket
  rospy.Subscriber('/gazebo/model_states', ModelStates, nav.sub_cal, queue_size=1000)

  #ob_postion_relative = PoseWithCovarianceStamped
  #qr_status = rospy.Subscriber('visp_auto_tracker/status', Int8, qr_handler.qr_status_callback)
  #qr_obj_pos_cov = rospy.Subscriber('visp_auto_tracker/object_position_covariance', PoseWithCovarianceStamped, qr_handler.qr_tf_cov_callback)
  #qr_msg = rospy.Subscriber('visp_auto_tracker/code_message', String, qr_handler.qr_msg_callback)
  #odom = rospy.Subscriber('odom', Odometry, qr_handler.odom_callbak)
  #client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
  
  rospy.sleep(2)
  rate = rospy.Rate(60)

  # start main loop (until all QR codes are found)
  twist = Twist()
  while not rospy.is_shutdown():

    # every update pass the robot position to the qr_handler
    robot_pose = nav.get_coordinates()
    qr_handler.update_robot_pose(robot_pose)

    # if less than 2 QR codes have been found randomly wander around
    if sum(qr_handler.qr_found) < 2:

      twist = wan.move(True)
      cmd_vel_pub.publish(twist)

    # execute translation computation once when 2 QRs have been found
    elif sum(qr_handler.qr_found) == 2 and qr_handler.translation == None:
      # compute translation matrix
      qr_handler.transInit()

    # once at least 2 QRs has been found navigate to the next QR
    elif sum(qr_handler.qr_found) < qr_handler._NUM_OF_QR_MARKERS:
      # find qr real position for which real=None and hidden=[x,y]
      indices = [i for i in range(self._NUM_OF_QR_MARKERS) if (self.qr_hidden[i] != None and self.qr_real[i] == None)]
      for i in indices:
        qr_handler.calculate_real(i)

      # navigate to some qr for which we calculated the real position but don't have the letter for yet
      indices = [i for i in range(qr_handler._NUM_OF_QR_MARKERS) if (qr_handler.word == "")]
      # TODO: navigate to indices[0]

    else:
      print("ALL QR FOUND, word: " + str(qr_handler.word))

    rate.sleep()
  