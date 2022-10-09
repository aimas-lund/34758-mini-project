#!/usr/bin/env python
import math
import roslib
import numpy as np
#roslib.load_manifest('mini_project_1')
import sys
import copy
import rospy
import tf
import tf_conversions
import moveit_commander
from geometry_msgs.msg import *
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil


global message
global bucket_place # tuple
global pub

# sets the fingers joints values to open or close the gripper
# used in point 5 and 8 on flow diagram
def move_gripper(tmp):
  currentJointState = rospy.wait_for_message("/joint_states",JointState, 10)
  currentJointState.header.stamp = rospy.get_rostime()
  currentJointState.position = tuple(list(currentJointState.position[:6]) 
                + [tmp] + [tmp]+ [tmp])
  for i in range(3):
    pub.publish(currentJointState)
    rospy.sleep(0.1)
  rospy.sleep(2)

def open_gripper():
  move_gripper(0.005)

def close_gripper():
  move_gripper(0.7)

#makes message from gazebo model states global
def sub_cal(msg):
  global message
  message = msg

#definition of vertical movements in cartesian space 
#used in point 4 and 6 on flow diagram
#vertical_offset - z offset from the final_point
def up_down(vertical_offset, final_pos, group):
  pose_goal = group.get_current_pose().pose
  waypoint=[]
  waypoint.append(pose_goal)
  pose_goal.position.x =final_pos.position.x
  pose_goal.position.y =final_pos.position.y
  pose_goal.position.z =final_pos.position.z + vertical_offset
  waypoint.append(pose_goal)
  (plan, fraction) = group.compute_cartesian_path(waypoint,0.01, 0.0) 
  return plan

#pose movement to the final_pos with vertical offset
#used in points 3 and 7
def pose_command(group, final_pos, vertical_offset):
  pose_goal = group.get_current_pose().pose    
  pose_goal.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(
                                                    0.,  -math.pi/2, 0.))
  pose_goal.position.x = final_pos.position.x
  pose_goal.position.y = final_pos.position.y
  pose_goal.position.z = final_pos.position.z+vertical_offset
  
  return pose_goal

#updating the scene by precessing new data from /gazebo/model_states 
def set_scene(p, scene):
  global bucket_place # tuple
  cube_place = [] # array of tuples
  cube_name=[]

  for i in range(len(message.name)):

    if message.name[i] == "bucket":
      p.pose.position.x = message.pose[i].position.x
      p.pose.position.y = message.pose[i].position.y
      p.pose.position.z = message.pose[i].position.z + 0.09
      #0.09 is used as the bucket height is 0.18 and the reference frame is found in the middle 
      q = Quaternion(*tf_conversions.transformations.quaternion_from_euler(
                                                    0., 0.0, 0.785398))
      p.pose.orientation.x = q.x
      p.pose.orientation.y = q.y
      p.pose.orientation.z = q.z
      p.pose.orientation.w = q.w
      # Add the bucket position to the bucket_place tuple
      bucket_place = message.pose[i] 
      
      # Add box to MoveIt! scene
      scene.add_box(message.name[i], p, (0.21, 0.21, 0.18)) 

    if "cube" in message.name[i]:
      p.pose.position.x = message.pose[i].position.x
      p.pose.position.y = message.pose[i].position.y
      p.pose.position.z = message.pose[i].position.z
      p.pose.orientation.x = message.pose[i].orientation.x
      p.pose.orientation.y = message.pose[i].orientation.y 
      p.pose.orientation.z = message.pose[i].orientation.z 
      p.pose.orientation.w = message.pose[i].orientation.w
      # Add the cube position to the cube_place tuple
      cube_place.append(message.pose[i])
      # Add the cube name to the cube_name tuple
      cube_name.append(message.name[i])
      #add box to MoveIt! scene
      scene.add_box(message.name[i], p, (0.05, 0.05, 0.05)) 
  
  return cube_place, cube_name

#check if cube is in the bucket be comparing the distance
def cube_in_bucket(cube_name):
  global bucket_place
  distance = 100.0
  for i in range(len(message.name)):
    if message.name[i] == cube_name:
      distance = np.sqrt(
                      np.power((message.pose[i].position.x - 
                                bucket_place.position.x),2) +  
                      np.power(((message.pose[i].position.y - 
                                bucket_place.position.y)),2) ) 
      rospy.loginfo(distance)
      if(distance < 0.15):
        return True
  return False

def Rviz_visualizing(robot, plan, display_trajectory_publisher):
      print "============ Visualizing trajectory in RViz"
      display_trajectory = moveit_msgs.msg.DisplayTrajectory()
      display_trajectory.trajectory_start = robot.get_current_state()
      display_trajectory.trajectory.append(plan)
      display_trajectory_publisher.publish(display_trajectory)
      print "============ Waiting while cube approach is visualized..."

def move():
  global bucket_place # tuple

  ## Moveit setup
  moveit_commander.roscpp_initialize(sys.argv)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")
  
  # Trajectories for RVIZ to visualize published on the topic '/move_group/display_planned_path'
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, 
                                      queue_size=10)
  # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(2)
  
  # Message definition 
  p = geometry_msgs.msg.PoseStamped()
  #The header is generally used to communicate timestamped data in a particular coordinate frame.
  p.header.frame_id = robot.get_planning_frame()

  # Set up the planner
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)

  # Set initial starting position over bucket with open grippe. Hard coded position.
  open_gripper()
  group.set_joint_value_target([-0.38807541740850837, -2.2543007891023263, 
                    0.7487280207426998, -2.785033596945393, 2.121855026824953,
                    -0.1252666708712793])
  plan1 = group.plan()
  group.go(wait=True)
  rospy.sleep(3.0)

  cube_place, cube_name = set_scene(p, scene)
  num_cubes = len(cube_name)
  
  for i in range(0, num_cubes):
    num_attempts = 0
    # place cube in bucket - if more than 2 attempts, assume out of range
    while not cube_in_bucket(cube_name[i]) and num_attempts < 2:
      # clear the scene
      scene.remove_world_object()
      # reset the scene and store positions of 
      #cubes and bucket for later reference
      # (inside for loop in case cube gets moved accidentally)
      cube_place, cube_name = set_scene(p, scene)

      # Set position :: above cube
      pose_goal = pose_command(group, cube_place[i], 0.3)
      group.set_pose_target(pose_goal)
      
      # Place above bucket to above cube
      plan1 = group.plan()  
      
      #RViz vizualization
      Rviz_visualizing(robot, plan1, display_trajectory_publisher)
    
      # Moving to above cube
      group.go(wait=True)
      
      # Moving gripper straight down
      group.set_pose_target(pose_goal)
      plan1= up_down(0.15, cube_place[i], group)
      group.execute(plan1,wait=True)
      #rospy.sleep(3.)
      close_gripper()
      
      # Moving gripper straight up
      plan2 = up_down(0.5, cube_place[i], group)

      #RViz vizualization
      Rviz_visualizing(robot, plan2, display_trajectory_publisher)
    
      # Moving to a pose goal
      group.execute(plan2,wait=True)
      
      # Set position :: above bucket
      pose_goal = pose_command(group, bucket_place, 0.5)
      group.set_pose_target(pose_goal)
  
      plan3 = group.plan()
      #RViz vizualization
      Rviz_visualizing(robot, plan3, display_trajectory_publisher)
      # Moving to a pose goal
      group.go(wait=True)  
      open_gripper()
      num_attempts += 1

  moveit_commander.roscpp_shutdown()
  
if __name__ == '__main__':
  global pub
  #Initialization of a node called pick_and_place
  rospy.init_node('pick_and_place', anonymous=True)
  rospy.loginfo("Starting... ")
 
  #Publishing to the topic /jaco/joint_control to send movement commands to the robot
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=10)
  
  #Subscribing to the topic /gazebo/model_states to read the positions of the cube and bucket
  rospy.Subscriber('/gazebo/model_states', ModelStates, sub_cal, 
                                                        queue_size=1000)
  rospy.sleep(3)
  #Call move() function
  print(message)
  print('##########################')
  move()

  
  rospy.spin()