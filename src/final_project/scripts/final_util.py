import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose


def point_sum(a, b):
    return Point(a.x + b.x, a.y + b.y, a.z + b.z)

def hidden_cord_to_world_cord(x,y):
        #if (self.translation == None) or (self.rotation == None):
         #   raise Exception("Translation and rotation has not been calculated yet")

        # TODO: fix this
        rotation = None
        translation = None

        vector_hidden = np.array([x,y])
        
        vector_world = np.dot(np.array([rotation, 0]), vector_hidden) + translation

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


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    norm = np.linalg.norm(vector)
    assert(norm != 0.0)
    return vector / norm


def calculate_real_qr_pose(robot_pose, qr_robot_diff):
    """
    returns world position of QR
    """

    # TODO: we need to use robot_pose.rotation and qr_pose.rotation for this to work
    # simly adding these two isn't enough, we also tried adding z to x for example because this seemed promising
    # but that only seemed to work for certain rotations
    real_qr_position = point_sum(robot_pose.position, qr_robot_diff.position)

    return Pose(real_qr_position, Quaternion());

def calculate_next_qr_pose(robot_pose, qr_pose, hidden_x, hidden_y):

    # TODO: how we calculate this

    return Pose()
    world_vector = np.array([worldX, worldY])
    hidden_vector = np.array([hidden_x, hidden_y])

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