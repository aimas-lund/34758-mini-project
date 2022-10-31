
#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

class broadcaster:
    
    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)


    def broadcast_marker_tf(self, worldX, worldY, hiddenX, hiddenY):

        world_vector = np.array([worldX, worldY])
        hidden_vector = np.array([hiddenX, hiddenY])

        # Calculate rotation:
        unitvector_world = self.unit_vector(world_vector)
        unitvector_hidden = self.unit_vector(hidden_vector)
        theta = np.arccos(
            np.clip(np.dot(unitvector_world, unitvector_hidden), -1.0, 1.0))

        theta_in_degrees = theta*180/3.14
        print("Found rotation:", theta_in_degrees, "degrees")

        # Calculate translation:
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        translation = world_vector - rotation_matrix.dot(hidden_vector)
        print("Found translation:", translation)

        # Broadcast the found TF
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        br.sendTransform((translation[0], translation[1], 0),
                        tf.transformations.quaternion_from_euler(0, 0, theta),
                        rospy.Time.now(),
                        "marker_frame",
                        "world")
        rate.sleep()
