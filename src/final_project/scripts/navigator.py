import rospy, tf, actionlib, tf_conversions
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
 
from geometry_msgs.msg import *

class Navigator:
    """
    Navigate to given position (e.g. QR code)
    """

    _MAP_FRAME_ID = 'map'

    _waypoints = [  
        [(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
        [(0.0, 0.0, 0.0), (0.0, 0.0, -0.64003024, -0.76812292098)]
    ]

    def __init__(self):
        self.client = 0
        self.robot_pos = 0

    def sub_cal(self,msg):
        """
        subscribes to topic /gazebo/model_states
        """
        self.message = msg

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
        print('reached goal')

    def get_coordinates(self):
        """
        loop through all models in scene and get position of robot
        """
        pose = geometry_msgs.msg.Pose()

        for i in range(len(self.message.name)):

            if self.message.name[i] == "turtlebot3_burger":
                pose.position.x = self.message.pose[i].position.x
                pose.position.y = self.message.pose[i].position.y
                pose.position.z = self.message.pose[i].position.z + 0.09
                #0.09 is used as the bucket height is 0.18 and the reference frame is found in the middle 
                q = Quaternion(*tf_conversions.transformations.quaternion_from_euler(
                                                                0., 0.0, 0.785398))
                pose.orientation.x = q.x
                pose.orientation.y = q.y
                pose.orientation.z = q.z
                pose.orientation.w = q.w
                # Add the bucket position to the bucket_pos tuple
                self.robot_pos= self.message.pose[i]             
        
        return pose
