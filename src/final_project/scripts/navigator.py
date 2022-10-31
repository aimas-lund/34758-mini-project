import rospy, tf, actionlib, tf_conversions
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Navigator:

    _MAP_FRAME_ID = 'map'
    _ROSPY_NODE_NAME = 'navigator'

    _waypoints = [  
        [(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
        [(0.0, 0.0, 0.0), (0.0, 0.0, -0.64003024, -0.76812292098)]
    ]

    def __init__(self):
        rospy.init_node(self._ROSPY_NODE_NAME)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        print('--- Navigator module initialized ---')


    def _goal_pose(self, pose, frame_id=_MAP_FRAME_ID):  
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = frame_id
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
    
        return goal_pose


    def move_to_pose(self, pose, frame_id=_MAP_FRAME_ID):
        goal = self._goal_pose(pose)
        self.client.send_goal(goal)
        print('Moving to pose:')
        print(pose)
        
        self.client.wait_for_result()
        rospy.sleep(3)

        return
