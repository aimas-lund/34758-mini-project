#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class wander:
  def __init__(self):
    self.g_range_ahead = 1
    self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

  def scan_callback(self,msg):
    tmp=[msg.ranges[0]]
    for i in range(1,21):
      tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
      tmp.append(msg.ranges[i])
    self.g_range_ahead = min(tmp)
 

  def move(self,stop = False):

    rospy.init_node('wander')
    state_change_time = rospy.Time.now() + rospy.Duration(1)
    driving_forward = True
    rate = rospy.Rate(60)
    
    if not stop:
      if self.g_range_ahead < 0.8:
        # TURN
        driving_forward = False
      
      else: # we're not driving_forward
        driving_forward = True # we're done spinning, time to go forward!
        #DRIVE
      
      twist = Twist()
      if driving_forward:
        twist.linear.x = 0.4
        twist.angular.z = 0.0
      else:
        twist.linear.x = 0.0
        twist.angular.z = 0.4
      self.cmd_vel_pub.publish(twist)
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

