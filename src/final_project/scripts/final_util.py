import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, concatenate_matrices, translation_matrix


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


def calculate_real_qr_xy(robot_pose, qr_robot_diff, listener):
    """
    returns world position of QR
    """

    #rospy.loginfo("robot_pose: " + str(robot_pose))
    #rospy.loginfo("qr_pose: " + str(qr_robot_diff))

    pose_camera = [qr_robot_diff.position.x, qr_robot_diff.position.y, qr_robot_diff.position.z, 1]
    try:
        # look up for the transformation between camera and odometry
        (trans,rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))

        # apply the transformation
        t = translation_matrix(trans)
        matrix_rot = quaternion_matrix(rot)
        T_odomo_camera = concatenate_matrices(t,matrix_rot)
        p_qr = T_odomo_camera.dot(pose_camera)

        return [p_qr[0], p_qr[1]]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Error in the listener lookup transformations")

    return None

def calculate_next_qr_pose(robot_pose, qr_robot_diff, hidden_x, hidden_y, hidden_x_next, hidden_y_next):

    real_qr_pos = calculate_real_qr_xy(robot_pose, qr_robot_diff)



    # TODO: how we calculate this
    real_qr_next_pos = Point(hidden_x_next, hidden_y_next) # wrong

    return Pose(real_qr_next_pos, Quaternion())
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