import rospy, tf, actionlib, tf_conversions
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Navigator:

    _MAP_FRAME_ID = 'map'

    _waypoints = [  
        [(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
        [(0.0, 0.0, 0.0), (0.0, 0.0, -0.64003024, -0.76812292098)]
    ]

    def __init__(self):
        self.client = 0


    def _goal_pose(self, pose, frame_id=_MAP_FRAME_ID):  
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = frame_id
        goal_pose.target_pose.pose.position.x = pose.position.x
        goal_pose.target_pose.pose.position.y = pose.position.y
        goal_pose.target_pose.pose.position.z = pose.position.z
        goal_pose.target_pose.pose.orientation.x = pose.orientation.x
        goal_pose.target_pose.pose.orientation.y = pose.orientation.y
        goal_pose.target_pose.pose.orientation.z = pose.orientation.z
        goal_pose.target_pose.pose.orientation.w = pose.orientation.w
    
        return goal_pose


    def move_to_pose(self, pose, frame_id=_MAP_FRAME_ID):
        goal = self._goal_pose(pose)
        self.client.send_goal(goal)
        
        self.client.wait_for_result()
        rospy.sleep(3)
