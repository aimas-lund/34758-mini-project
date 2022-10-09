
import roslib
roslib.load_manifest('mini_1')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from getcoordinates import model_coordinates
import math
import numpy as np

class Arm:

    # gripper parameters
    TMP_OPEN = 0.7
    TMP_CLOSED = 0.005

    # publisher parameters
    PUBLISH_RATE = 10       # 10 Hz

    # group parameters
    ORIENTATION_TOLERANCE = 0.01
    GOAL_TOLERANCE = 0.01
    GOAL_JOINT_TOLERANCE = 0.01
    PLANNING_ATTEMPTS = 100
    MAX_VELOCITY_SCALING_FACTOR = 1.0
    MAX_ACCELERATION_SCALING_FACTOR = 1.0
    EEF_STEP = 0.1
    JUMP_THRESHOLD = 0.0


    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("Arm")
        
        ## trajectories for RVIZ to visualize.
        self.display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, 
                                        queue_size=10)
        rospy.sleep(2)

        self.p = geometry_msgs.msg.PoseStamped()
        #The header is generally used to communicate timestamped data in a particular coordinate frame.
        self.p.header.frame_id = self.robot.get_planning_frame()

        self.group.set_goal_orientation_tolerance(self.ORIENTATION_TOLERANCE)
        self.group.set_goal_tolerance(self.GOAL_TOLERANCE)
        self.group.set_goal_joint_tolerance(self.GOAL_JOINT_TOLERANCE)
        self.group.set_num_planning_attempts(self.PLANNING_ATTEMPTS)

        self.open_gripper()
        self.group.set_joint_value_target([-0.38807541740850837, -2.2543007891023263, 
                    0.7487280207426998, -2.785033596945393, 2.121855026824953,
                    -0.1252666708712793])
        plan1 = self.group.plan()
        self.group.go(wait=True)
        rospy.sleep(3.0)

        self.coordinates = model_coordinates()


    def _handle_gripper(self, tmp):
        #Publishing to the topic /jaco/joint_control to send movement commands to the robot
        pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=10)
        currentJointState = rospy.wait_for_message("/joint_states",JointState, 10)
        currentJointState.header.stamp = rospy.get_rostime()
        currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
        for i in range(3):
            pub.publish(currentJointState)
            rospy.sleep(0.1)
        rospy.sleep(2)


    def open_gripper(self):
        self._handle_gripper(self.TMP_OPEN)
        print("Opened gripper!")


    def close_gripper(self):
        self._handle_gripper(self.TMP_CLOSED)
        print("Closed gripper!")


    def move_two_step(self, pos, z_offset= 0.5):
        ## Moves arm above a Pose with a specified offset, after which it moves down to the Pose coordinates.
        final_pose = pos
        intermediate_pose = copy.deepcopy(pos)
        intermediate_pose.position.z = final_pose.position.z + z_offset

        self.move(intermediate_pose)
        self.move(final_pose)

    def pose_command(self, group, final_pos, vertical_offset):
        pose_goal = group.get_current_pose().pose    
        pose_goal.orientation = geometry_msgs.msg.Quaternion(
                    *tf_conversions.transformations.quaternion_from_euler(0.,  -math.pi/2, 0.))
        pose_goal.position.x = final_pos.position.x
        pose_goal.position.y = final_pos.position.y
        pose_goal.position.z = final_pos.position.z+vertical_offset    
        return pose_goal

    def Rviz(self, plan):
        print("============ Visualizing trajectory in RViz")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        print("============ Waiting while cube approach is visualized...")

    def move_vertical(self,vertical_offset, final_pos):
        pose_goal = self.group.get_current_pose().pose
        waypoint=[]
        waypoint.append(pose_goal)
        pose_goal.position.x =final_pos.position.x
        pose_goal.position.y =final_pos.position.y
        pose_goal.position.z =final_pos.position.z + vertical_offset
        waypoint.append(pose_goal)
        (plan, fraction) = self.group.compute_cartesian_path(waypoint,0.01, 0.0) 
        return plan

    def move(self):
        
        cube_place, cube_name = self.coordinates.get_coordinates(self.p, self.scene)
        num_cubes = len(cube_name)
        for i in range(0, num_cubes):
            num_attempts = 0
            # place cube in bucket - if more than 2 attempts, assume out of range
            while not self.coordinates.cube_in_bucket(cube_name[i]) and num_attempts < 2:
                # clear the scene
                self.scene.remove_world_object()
                # reset the scene and store positions of 
                #cubes and bucket for later reference
                # (inside for loop in case cube gets moved accidentally)
                cube_place, cube_name = self.coordinates.get_coordinates(self.p, self.scene)

                # Set position :: above cube
                pose_goal = self.pose_command(self.group, cube_place[i], 0.3)
                self.group.set_pose_target(pose_goal)
                
                # Place above bucket to above cube
                plan1 = self.group.plan()  
                
                #RViz vizualization
                self.Rviz( plan1)
                
                # Moving to above cube
                self.group.go(wait=True)
                
                # Moving gripper straight down
                self.group.set_pose_target(pose_goal)
                plan1= self.move_vertical(0.15, cube_place[i])
                self.group.execute(plan1,wait=True)
                #rospy.sleep(3.)
                self.close_gripper()
                
                # Moving gripper straight up
                plan2 = self.move_vertical(0.5, cube_place[i])

                #RViz vizualization
                self.Rviz(plan2)
                
                # Moving to a pose goal
                self.group.execute(plan2,wait=True)
                
                # Set position :: above bucket
                pose_goal = self.pose_command(self.group, self.coordinates.bucket_pos, 0.5)
                self.group.set_pose_target(pose_goal)
            
                plan3 = self.group.plan()
                #RViz vizualization
                self.Rviz(self.robot, plan3)
                # Moving to a pose goal
                self.group.go(wait=True)  
                open_gripper()
                num_attempts += 1

        moveit_commander.roscpp_shutdown()

