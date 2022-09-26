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


def _handle_gripper(tmp) -> None:
    rospy.init_node('test_publish')

    # setup publisher
    pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

    currentJointState = rospy.wait_for_message("/joint_states",JointState)
    currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print 'Received!'
    currentJointState.header.stamp = rospy.get_rostime()
    currentJointState.position = tuple(list(currentJointState.position[:6]) + 3*[tmp])
    rate = rospy.Rate(PUBLISH_RATE)
    for _ in range(3):
        pub.publish(currentJointState)
        print 'Published!'
        rate.sleep()

    print 'end!'


def open_gripper() -> None:
    _handle_gripper(TMP_OPEN)


def close_gripper() -> None:
    _handle_gripper(TMP_CLOSED)


def move_arm(pos: Pose) -> None:
    ## First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")

    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory)

    print "============ Robot state:"
    print robot.get_current_state()
    print "============"

    ## ----- Planning phase -----
    #group.set_planning_time(0.0)
    group.set_goal_orientation_tolerance(ORIENTATION_TOLERANCE)
    group.set_goal_tolerance(GOAL_TOLERANCE)
    group.set_goal_joint_tolerance(GOAL_JOINT_TOLERANCE)
    group.set_num_planning_attempts(PLANNING_ATTEMPTS)
    group.set_max_velocity_scaling_factor(MAX_VELOCITY_SCALING_FACTOR)
    group.set_max_acceleration_scaling_factor(MAX_ACCELERATION_SCALING_FACTOR)

    waypoints = []

    # creating waypoints
    waypoints.append(pos)

    #createcartesian  plan
    (plan, _) = group.compute_cartesian_path(
                                        waypoints,          # waypoints to follow
                                        EEF_STEP,           # eef_step
                                        JUMP_THRESHOLD)     # jump_threshold
    #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

    print "============ Waiting while RVIZ displays plan..."
    rospy.sleep(0.5)

    ## Visualizing tradjectory in RVIZ
    print "============ Visualizing tradjectory in RVIZ"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    rospy.sleep(2.)

    #If we're coming from another script we might want to remove the objects
    if "table" in scene.get_known_object_names():
        scene.remove_world_object("table")
    if "table2" in scene.get_known_object_names():
        scene.remove_world_object("table2")
    if "groundplane" in scene.get_known_object_names():
        scene.remove_world_object("groundplane")

    ## Moving to a pose goal
    group.execute(plan,wait=True)
    rospy.sleep(4.)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    print "============ STOPPING"
    R = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():
        R.sleep()