
#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def broadcast_marker_tf(realX, realY, hiddenX, hiddenY):

    real = np.array([realX, realY])
    hidden = np.array([hiddenX, hiddenY])

    # Calculate rotation:
    unitvector_real = unit_vector(real)
    unitvector_hidden = unit_vector(hidden)
    theta = np.arccos(
        np.clip(np.dot(unitvector_real, unitvector_hidden), -1.0, 1.0))

    theta_in_degrees = theta*180/3.14
    print("Found rotation:", theta_in_degrees, "degrees")

    # Calculate translation:
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    translation = real - rotation_matrix.dot(hidden)
    print("Found translation:", translation)

    # Broadcast the found TF
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((translation[0], translation[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, theta),
                         rospy.Time.now(),
                         "marker_frame",
                         "world")
        rate.sleep()
