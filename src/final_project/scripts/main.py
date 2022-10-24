#!/usr/bin/env python

from distutils.spawn import spawn
import rospy
import subobject
import sys
import time


from navigation import Arm
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelStates
from cube_spawn import Spawner
from argument_handler import ArgumentHandler
from modelcoordinates import model_coordinates
import moveit_msgs.msg

spawner = Spawner()
arg_handler = ArgumentHandler()

def init(refresh=False):
  rospy.init_node('mini_1')

  if not refresh:
    spawner.spawn_models()
    print("Spawned Models!")
    return
    
  spawner.despawn_cubes()
  spawner.spawn_cubes()


if __name__ == '__main__':
  try:
    # Spawn environment
    init(arg_handler.should_run_refresh_mode())

    arm = Arm()
    model_coordinates = model_coordinates()

    ## trajectories for RVIZ to visualize.
    arm.display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, 
                                    queue_size=10)
    #Subscribing to the topic /gazebo/model_states to read the positions of the cube and bucket
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_coordinates.sub_cal , 
                                                          queue_size=1000)
    rospy.sleep(2)

    cube_place, cube_name = model_coordinates.get_coordinates(arm.pose, arm.scene)
    num_cubes = len(cube_name)
    for i in range(0, num_cubes):
        num_attempts = 0
        # place cube in bucket - if more than 2 attempts, assume out of range
        while not model_coordinates.cube_in_bucket(cube_name[i]) and num_attempts < 2:
            # clear the scene
            arm.scene.remove_world_object()
            # reset the scene and store positions of 
            #cubes and bucket for later reference
            # (inside for loop in case cube gets moved accidentally)
            cube_place, cube_name = model_coordinates.get_coordinates(arm.pose, arm.scene)

            # Set position :: above cube
            pose_goal = arm.pose_command(cube_place[i], 0.3)
            arm.group.set_pose_target(pose_goal)
            
            # Place above bucket to above cube
            plan1 = arm.group.plan()  
            
            #RViz vizualization
            arm.Rviz(plan1)
            
            # Moving to above cube
            arm.group.go(wait=True)
            
            # Moving gripper straight down
            arm.group.set_pose_target(pose_goal)
            plan1= arm.move_vertical(0.15, cube_place[i])
            arm.group.execute(plan1,wait=True)
            #rospy.sleep(3.)
            arm.close_gripper()
            
            # Moving gripper straight up
            plan2 = arm.move_vertical(0.5, cube_place[i])

            #RViz vizualization
            arm.Rviz(plan2)
            
            # Moving to a pose goal
            arm.group.execute(plan2,wait=True)
            
            # Set position :: above bucket
            pose_goal = arm.pose_command(model_coordinates.bucket_pos, 0.5)
            arm.group.set_pose_target(pose_goal)
        
            plan3 = arm.group.plan()
            #RViz vizualization
            arm.Rviz(plan3)
            # Moving to a pose goal
            arm.group.go(wait=True)  
            arm.open_gripper()
            num_attempts += 1

  except rospy.ROSInterruptException:
    print("Something went wrong - please try again")