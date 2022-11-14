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
    [(-5, 0.2, 0.0), (0.0, 0.0, 0.0, 1.0)]]

    def __init__(self):
        self.client = 0
        self.robot_pos = 0

    def sub_cal(self,msg):
        """
        subscribes to topic /gazebo/model_states
        """
        self.message = msg

    def _goal_pose(self, pose,angle=0):  
        goal_pose = MoveBaseGoal()
        frame_id = self._MAP_FRAME_ID
        goal_pose.target_pose.header.frame_id = frame_id
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        goal_pose.target_pose.pose.orientation.x = quaternion[0]
        goal_pose.target_pose.pose.orientation.y = quaternion[1]
        goal_pose.target_pose.pose.orientation.z = quaternion[2]
        goal_pose.target_pose.pose.orientation.w = quaternion[3]
    
        return goal_pose


    def move_to_pose(self, pose,angle=0):
        goal = self._goal_pose(pose,angle)
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

