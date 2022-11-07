
#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Pose

import final_util

class coordinateTransformer:
    """
    object of qr_position_handler
    """

    translation = [np.nan,np.nan]
    rotation = np.nan

        
    def hidden_cord_to_world_cord(self,x,y):
        #if (self.translation == None) or (self.rotation == None):
         #   raise Exception("Translation and rotation has not been calculated yet")

        vector_hidden = np.array([x,y])
        
        vector_world = np.dot(np.array([self.rotation, 0]), vector_hidden) + self.translation

        pose = Pose()
        pose.position.x = vector_world[0]
        pose.position.y = vector_world[1]
        pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose


    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        norm = np.linalg.norm(vector)
        assert(norm != 0.0)
        return vector / norm


    def calculate_real_qr_pose(self, robot_pose, qr_pose, hiddenX, hiddenY):
        """
        based on 
        """

        real_qr_position = final_util.point_sum(robot_pose.position, qr_pose.position)
        rospy.loginfo("robot_pose: " + str(robot_pose))
        rospy.loginfo("qr_pose: " + str(qr_pose))
        rospy.loginfo("real_qr_position: " + str(real_qr_position))

        # TODO: do we need to use robot_pose.rotation and qr_pose.rotation for a proper real_qr_position

        return

        world_vector = np.array([worldX, worldY])
        hidden_vector = np.array([hiddenX, hiddenY])

        # Calculate rotation:
        unitvector_world = self.unit_vector(world_vector)
        unitvector_hidden = self.unit_vector(hidden_vector)
        theta = np.arccos(
            np.clip(np.dot(unitvector_world, unitvector_hidden), -1.0, 1.0))

        theta_in_degrees = theta*180/3.14

        # Calculate translation:
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        translation = world_vector - rotation_matrix.dot(hidden_vector)

        #Set the translation and rotation
        self.translation = translation
        self.rotation = theta

        rospy.loginfo("Translation and rotation calculated as: " + str(translation) + ", " + str(theta))

