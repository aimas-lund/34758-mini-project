
import roslib
roslib.load_manifest('mini_1')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
from geometry_msgs.msg import *
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
import math
import numpy as np
import moveit_msgs.msg

class Arm:

    # gripper parameters
    TMP_OPEN = 0.005
    TMP_CLOSED = 0.7

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
        

        self.pose = geometry_msgs.msg.PoseStamped()
        #The header is generally used to communicate timestamped data in a particular coordinate frame.
        self.pose.header.frame_id = self.robot.get_planning_frame()

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


    def pose_command(self, final_pos, vertical_offset):
        pose_goal = self.group.get_current_pose().pose    
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


        moveit_commander.roscpp_shutdown()

