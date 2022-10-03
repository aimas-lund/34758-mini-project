import roslib
roslib.load_manifest('mini_1')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState

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
        try:
            rospy.init_node('arm_group_node',
                            anonymous=True)
            #rospy.init_node('test_publish')
            
        except Exception:
            pass

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("Arm")
        
        ## trajectories for RVIZ to visualize.
        self.display_trajectory_publisher = rospy.Publisher(
                                        ('/move_group/display_planned_path'),
                                            moveit_msgs.msg.DisplayTrajectory)


    def _handle_gripper(self, tmp):
        # setup publisher
        pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

        currentJointState = rospy.wait_for_message("/joint_states",JointState)
        currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print('Received!')
        currentJointState.header.stamp = rospy.get_rostime()
        currentJointState.position = tuple(list(currentJointState.position[:6]) + 3*[tmp])
        rate = rospy.Rate(self.PUBLISH_RATE)
        for _ in range(3):
            pub.publish(currentJointState)
            # rate.sleep()


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
        

    def move(self, pos):

                ## We can get the name of the reference frame for this robot
        print("============ Reference frame: %s" % self.group.get_planning_frame())
        
        ## We can also print the name of the end-effector link for this group
        print("============ Reference frame: %s" % self.group.get_end_effector_link())

        # print ("============ Robot state:")
        # print (self.robot.get_current_state())
        # print ("============")

        ## ----- Planning phase -----
        #group.set_planning_time(0.0)
        self.group.set_goal_orientation_tolerance(self.ORIENTATION_TOLERANCE)
        self.group.set_goal_tolerance(self.GOAL_TOLERANCE)
        self.group.set_goal_joint_tolerance(self.GOAL_JOINT_TOLERANCE)
        self.group.set_num_planning_attempts(self.PLANNING_ATTEMPTS)
        self.group.set_max_velocity_scaling_factor(self.MAX_VELOCITY_SCALING_FACTOR)
        self.group.set_max_acceleration_scaling_factor(self.MAX_ACCELERATION_SCALING_FACTOR)

        waypoints = []

        # creating waypoints
        waypoints.append(pos)

        #createcartesian  plan
        (plan, _) = self.group.compute_cartesian_path(
                                            waypoints,          # waypoints to follow
                                            self.EEF_STEP,           # eef_step
                                            self.JUMP_THRESHOLD)     # jump_threshold
        #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

        print ("============ Waiting while RVIZ displays plan...")
        rospy.sleep(0.5)

        ## Visualizing tradjectory in RVIZ
        print ("============ Visualizing tradjectory in RVIZ")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(2.)

        #If we're coming from another script we might want to remove the objects
        if "table" in self.scene.get_known_object_names():
            self.scene.remove_world_object("table")
        if "table2" in self.scene.get_known_object_names():
            self.scene.remove_world_object("table2")
        if "groundplane" in self.scene.get_known_object_names():
            self.scene.remove_world_object("groundplane")

        ## Moving to a pose goal
        print("--- Executing tradjectory plan ---")
        self.group.execute(plan,wait=True)
        rospy.sleep(4.)

        ## When finished shut down moveit_commander.
        # moveit_commander.roscpp_shutdown()

        print("--- Arm moved ---")
        # print ("============ STOPPING")
        # R = rospy.Rate(self.PUBLISH_RATE)
        # while not rospy.is_shutdown():
        #     R.sleep()