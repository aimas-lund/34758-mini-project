#!/usr/bin/env python
from distutils.spawn import spawn
import rospy, tf, actionlib, tf_conversions
from wander import wander
from qr_position_handler import QRHandler
import os, argparse, time

from sensor_msgs.msg import LaserScan
from navigator import Navigator
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Quaternion, Point, Twist
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


expected default positions (in array order)
[[-4.65, 2.95], [-6, 2.95], [-6.7, 0.4], [-5.5, -3], [-3.5, -3]]
hidden:
[x,[2.67, 3.23],[0.1, 3.5],[-3.08, 1.95],[-3.08, 0.0]]


rot mat:
[[ 0.9995821  -0.02890712]
 [ 0.02890712  0.9995821 ]]

trans mat:
[-1.489408242232296, 0.7119754831249485]



# TODO LIST:
- sometimes robot might get a little stuck, needs a bit more testing to see if we can fix this in the navigator
- maybe initial 2 QR position estimation can be improved a bit having robot pos and qr pos time stamps aligned

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

  # setup wander and Navigator objects
  wan = wander()
  nav = Navigator()        

  # initialize QR reader and final word variable
  qr_handler = QRHandler(listener, nav)  
  qr_handler.qr_reader()

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

  rospy.logdebug("test message", logger_name="main.py")

  # start main loop (until all QR codes are found)
  twist = Twist()
  prev_visit = 99
  while not rospy.is_shutdown():
    # print 3 arrays every 5 seconds
    rospy.loginfo_throttle(0.5, "qr_hidden: " + str(qr_handler.qr_hidden))
    rospy.loginfo_throttle(0.5, "qr_real: " + str(qr_handler.qr_real))
    rospy.loginfo_throttle(0.5, "word: " + str(qr_handler.word))

    # if less than 2 QR codes have been found randomly wander around
    if (qr_handler.number_of_qr_markers - qr_handler.qr_real.count(None)) < 2:

      twist = wan.move(True)
      cmd_vel_pub.publish(twist)

      # uncommented this to start with 2 already found QR points, to test just the transformation and navigation behavior
      # qr_handler.qr_real[4] = [-3.5, -3]
      # qr_handler.qr_real[1] = [-6, 2.95]
      # qr_handler.qr_hidden[4] = [-3.08, 0.0]
      # qr_handler.qr_hidden[1] = [2.67, 3.23]
      # qr_handler.qr_hidden[2] =[0.1, 3.5]
      # qr_handler.word[4] = 't'
      # qr_handler.word[1] = 'M'

    # execute translation computation once when 2 QRs have been found
    elif (qr_handler.number_of_qr_markers - qr_handler.qr_real.count(None) >= 2) and (qr_handler.translation == None):
      # compute translation matrix
      qr_handler.calculateTranslationAndRotation()

    # once at least 2 QRs has been found navigate to the next QR
    elif not all(qr_handler.word):
      # rospy.logdebug("Current letters:" + str(qr_handler.word))
      # find qr real position for which real=None and hidden=[x,y]
      indices_missing_transform = [i for i in range(qr_handler.number_of_qr_markers) if (qr_handler.qr_hidden[i] != None and qr_handler.qr_real[i] == None)]
      rospy.logdebug("Missing transformed indices: " + str(indices_missing_transform))
      for i in indices_missing_transform:
        qr_handler.calculate_real(i)

      # navigate to some qr for which we calculated the real position but don't have the letter for yet
      indices_missing_word = [i for i in range(qr_handler.number_of_qr_markers) if (qr_handler.word[i] == "" and qr_handler.qr_real[i] != None)]
      rospy.logdebug("Missing located indices: " + str(indices_missing_word))
      
      if indices_missing_word[0] == prev_visit and len(indices_missing_word) > 1:
        goal = qr_handler.qr_real[indices_missing_word[1]]
        prev_visit = indices_missing_word[1]
      else:
        goal = qr_handler.qr_real[indices_missing_word[0]]
        prev_visit = indices_missing_word[0]

      rospy.logdebug("Navigating to QR marker at ({}, {})".format(goal[0], goal[1]))
      # TODO: currently move to pose hangs the while loop, it waits until moving is finished, this probably isn't what we want
      nav.move_to_pose(Pose(Point(goal[0], goal[1], 0), nav.get_coordinates().orientation))

    else:
      rospy.loginfo("ALL QR FOUND, word: " + str(qr_handler.word))
      break

    rate.sleep()
  